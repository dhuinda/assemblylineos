#!/usr/bin/env python3
"""
Raspberry Pi Browser Launcher Service

This script:
1. Scans for a website on 192.168.0.1xx:1111 (where xx is 00-99)
2. Once found, opens it in the browser in fullscreen/kiosk mode
3. Monitors the connection and reopens if the browser closes

Designed to run as a systemd service on a Raspberry Pi.
"""

import os
import sys
import time
import socket
import subprocess
import signal
from pathlib import Path


class BrowserLauncher:
    def __init__(self, ip_range_start=100, ip_range_end=199, port=1111, scan_interval=5, check_interval=30):
        """
        Initialize the browser launcher.
        
        Args:
            ip_range_start: Starting IP in the range (default: 100)
            ip_range_end: Ending IP in the range (default: 199)
            port: Port to check (default: 1111)
            scan_interval: Seconds between scan attempts (default: 5)
            check_interval: Seconds between connection checks when browser is open (default: 30)
        """
        self.ip_range_start = ip_range_start
        self.ip_range_end = ip_range_end
        self.port = port
        self.scan_interval = scan_interval
        self.check_interval = check_interval
        self.current_url = None
        self.browser_process = None
        self.running = True
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        print(f"\nReceived signal {signum}, shutting down...")
        self.running = False
        self.close_browser()
        sys.exit(0)
    
    def check_connection(self, host, port, timeout=2):
        """
        Check if a connection can be made to host:port.
        
        Args:
            host: Hostname or IP address
            port: Port number
            timeout: Connection timeout in seconds
            
        Returns:
            True if connection successful, False otherwise
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            result = sock.connect_ex((host, port))
            sock.close()
            return result == 0
        except Exception as e:
            return False
    
    def scan_for_website(self):
        """
        Scan the IP range for an available website.
        
        Returns:
            URL string if found, None otherwise
        """
        print(f"Scanning for website on 192.168.0.{self.ip_range_start}-{self.ip_range_end}:{self.port}...")
        
        for ip_last in range(self.ip_range_start, self.ip_range_end + 1):
            if not self.running:
                return None
                
            ip = f"192.168.0.{ip_last}"
            url = f"http://{ip}:{self.port}"
            
            print(f"Checking {url}...", end='\r')
            
            if self.check_connection(ip, self.port):
                print(f"\n✓ Found website at {url}")
                return url
        
        print(f"\n✗ No website found in range")
        return None
    
    def get_browser_command(self, url):
        """
        Get the browser command for fullscreen/kiosk mode.
        
        Tries different browsers in order of preference:
        1. Chromium (most common on Raspberry Pi)
        2. Chrome
        3. Firefox
        
        Args:
            url: URL to open
            
        Returns:
            List of command arguments, or None if no browser found
        """
        # Try Chromium first (most common on Raspberry Pi)
        browsers = [
            ('chromium-browser', ['--kiosk', '--noerrdialogs', '--disable-infobars', '--disable-session-crashed-bubble', '--disable-restore-session-state']),
            ('chromium', ['--kiosk', '--noerrdialogs', '--disable-infobars', '--disable-session-crashed-bubble', '--disable-restore-session-state']),
            ('google-chrome', ['--kiosk', '--noerrdialogs', '--disable-infobars', '--disable-session-crashed-bubble', '--disable-restore-session-state']),
            ('firefox', ['-kiosk']),
        ]
        
        for browser_name, args in browsers:
            # Check if browser exists
            try:
                result = subprocess.run(
                    ['which', browser_name],
                    capture_output=True,
                    check=True
                )
                if result.returncode == 0:
                    cmd = [browser_name] + args + [url]
                    print(f"Using browser: {browser_name}")
                    return cmd
            except (subprocess.CalledProcessError, FileNotFoundError):
                continue
        
        print("✗ Error: No suitable browser found")
        print("  Please install Chromium: sudo apt-get install chromium-browser")
        return None
    
    def open_browser(self, url):
        """
        Open the browser in fullscreen/kiosk mode.
        
        Args:
            url: URL to open
            
        Returns:
            True if browser opened successfully, False otherwise
        """
        if self.browser_process is not None:
            print("Browser already open, closing previous instance...")
            self.close_browser()
        
        cmd = self.get_browser_command(url)
        if cmd is None:
            return False
        
        try:
            # Set DISPLAY environment variable if not set (for X11)
            env = os.environ.copy()
            if 'DISPLAY' not in env:
                env['DISPLAY'] = ':0'
            
            print(f"Opening browser: {' '.join(cmd)}")
            self.browser_process = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            # Give browser a moment to start
            time.sleep(2)
            
            # Check if process is still running
            if self.browser_process.poll() is None:
                print(f"✓ Browser opened successfully (PID: {self.browser_process.pid})")
                self.current_url = url
                return True
            else:
                print("✗ Browser process exited immediately")
                self.browser_process = None
                return False
                
        except Exception as e:
            print(f"✗ Error opening browser: {e}")
            self.browser_process = None
            return False
    
    def close_browser(self):
        """Close the browser if it's open."""
        if self.browser_process is not None:
            try:
                print("Closing browser...")
                self.browser_process.terminate()
                # Wait up to 5 seconds for graceful shutdown
                try:
                    self.browser_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # Force kill if it doesn't terminate
                    self.browser_process.kill()
                    self.browser_process.wait()
                self.browser_process = None
                self.current_url = None
                print("✓ Browser closed")
            except Exception as e:
                print(f"⚠ Warning: Error closing browser: {e}")
                self.browser_process = None
    
    def is_browser_running(self):
        """Check if the browser process is still running."""
        if self.browser_process is None:
            return False
        
        # Check if process is still alive
        if self.browser_process.poll() is not None:
            return False
        
        return True
    
    def is_website_available(self):
        """Check if the current website is still available."""
        if self.current_url is None:
            return False
        
        # Extract IP from URL
        try:
            # URL format: http://192.168.0.xxx:1111
            ip = self.current_url.split('//')[1].split(':')[0]
            return self.check_connection(ip, self.port)
        except Exception:
            return False
    
    def run(self):
        """Main run loop."""
        print("=" * 60)
        print("Raspberry Pi Browser Launcher")
        print("=" * 60)
        print()
        
        while self.running:
            # If we don't have a browser open, scan for website
            if not self.is_browser_running():
                url = self.scan_for_website()
                
                if url and self.running:
                    if not self.open_browser(url):
                        print(f"Failed to open browser, will retry in {self.scan_interval} seconds...")
                        time.sleep(self.scan_interval)
                else:
                    # Website not found, wait before scanning again
                    if self.running:
                        print(f"Waiting {self.scan_interval} seconds before next scan...")
                        time.sleep(self.scan_interval)
            else:
                # Browser is open, check if website is still available
                if not self.is_website_available():
                    print(f"⚠ Website at {self.current_url} is no longer available")
                    self.close_browser()
                else:
                    # Everything is fine, wait before next check
                    time.sleep(self.check_interval)
        
        # Cleanup
        self.close_browser()
        print("Browser launcher stopped")


def main():
    """Main entry point."""
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(
        description='Launch browser in kiosk mode for Raspberry Pi'
    )
    parser.add_argument(
        '--ip-start',
        type=int,
        default=100,
        help='Starting IP in range (default: 100)'
    )
    parser.add_argument(
        '--ip-end',
        type=int,
        default=199,
        help='Ending IP in range (default: 199)'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=1111,
        help='Port to check (default: 1111)'
    )
    parser.add_argument(
        '--scan-interval',
        type=int,
        default=5,
        help='Seconds between scan attempts (default: 5)'
    )
    parser.add_argument(
        '--check-interval',
        type=int,
        default=30,
        help='Seconds between connection checks when browser is open (default: 30)'
    )
    
    args = parser.parse_args()
    
    # Create and run launcher
    launcher = BrowserLauncher(
        ip_range_start=args.ip_start,
        ip_range_end=args.ip_end,
        port=args.port,
        scan_interval=args.scan_interval,
        check_interval=args.check_interval
    )
    
    try:
        launcher.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        launcher.close_browser()
        sys.exit(0)


if __name__ == '__main__':
    main()
