#!/usr/bin/env python3
"""
Raspberry Pi Browser Launcher Service

This script:
1. Resolves a .local hostname to an IP address using ping
2. Opens http://<resolved-ip>:1111 in the browser in fullscreen/kiosk mode
3. Monitors the connection without repeatedly opening duplicate tabs

Designed to run as a systemd service on a Raspberry Pi.
"""

import os
import sys
import time
import socket
import subprocess
import signal
import re


class BrowserLauncher:
    BROWSER_PROCESS_PATTERN = r'chromium-browser|chromium|google-chrome|firefox'

    def __init__(self, hostname='hostname.local', port=1111, scan_interval=5, check_interval=30, ping_timeout=2):
        """
        Initialize the browser launcher.
        
        Args:
            hostname: .local hostname to resolve (default: hostname.local)
            port: Port to check (default: 1111)
            scan_interval: Seconds between hostname resolution attempts (default: 5)
            check_interval: Seconds between connection checks when browser is open (default: 30)
            ping_timeout: Seconds to wait for ping resolution (default: 2)
        """
        self.hostname = self.normalize_hostname(hostname)
        self.port = port
        self.scan_interval = scan_interval
        self.check_interval = check_interval
        self.ping_timeout = ping_timeout
        self.current_url = None
        self.browser_process = None
        self.last_opened_url = None
        self.last_open_attempt = 0
        self.running = True
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)

    def normalize_hostname(self, hostname):
        """Normalize hostnames so setup can pass either 'name' or 'name.local'."""
        hostname = hostname.strip()
        if not hostname.endswith('.local'):
            hostname = f'{hostname}.local'
        return hostname
    
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
    
    def resolve_target_ip(self):
        """
        Resolve the configured .local hostname to an IPv4 address using ping.
        
        Returns:
            IP address string if found, None otherwise
        """
        print(f"Pinging {self.hostname} to resolve target IP...")
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', str(self.ping_timeout), self.hostname],
                capture_output=True,
                text=True,
                check=False
            )
            output = f'{result.stdout}\n{result.stderr}'
            match = re.search(r'\b(?:\d{1,3}\.){3}\d{1,3}\b', output)
            if result.returncode == 0 and match:
                ip = match.group(0)
                print(f"✓ Resolved {self.hostname} to {ip}")
                return ip
        except FileNotFoundError:
            print("⚠ Warning: ping command not found; falling back to DNS resolution")
        except Exception as e:
            print(f"⚠ Warning: ping failed for {self.hostname}: {e}")

        try:
            ip = socket.gethostbyname(self.hostname)
            print(f"✓ Resolved {self.hostname} to {ip}")
            return ip
        except Exception as e:
            print(f"✗ Could not resolve {self.hostname}: {e}")
            return None

    def find_website(self):
        """
        Resolve the target host and check whether the web port is available.
        
        Returns:
            URL string if found, None otherwise
        """
        if not self.running:
            return None

        ip = self.resolve_target_ip()
        if not ip:
            return None

        url = f"http://{ip}:{self.port}/?raspi=1"
        print(f"Checking {url}...")

        if self.check_connection(ip, self.port):
            print(f"✓ Found website at {url}")
            return url

        print(f"✗ {url} is not accepting connections")
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
        chromium_args = [
            '--kiosk',
            '--noerrdialogs',
            '--disable-infobars',
            '--disable-session-crashed-bubble',
            '--disable-restore-session-state',
            '--no-first-run',
            '--disable-extensions',
            '--disable-component-update',
            '--disable-background-networking',
            '--disable-sync',
            '--disable-translate',
            '--disable-features=Translate,MediaRouter,AutofillServerCommunication,OptimizationHints',
            '--disable-pinch',
            '--overscroll-history-navigation=0',
            '--process-per-site',
            '--renderer-process-limit=2',
            '--enable-low-end-device-mode',
            '--autoplay-policy=no-user-gesture-required',
        ]
        browsers = [
            ('chromium-browser', chromium_args),
            ('chromium', chromium_args),
            ('google-chrome', chromium_args),
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

    def get_browser_processes(self):
        """Return command lines for currently running browser processes."""
        try:
            result = subprocess.run(
                ['pgrep', '-af', self.BROWSER_PROCESS_PATTERN],
                capture_output=True,
                text=True,
                check=False
            )
        except FileNotFoundError:
            return []
        except Exception as e:
            print(f"⚠ Warning: Could not inspect browser processes: {e}")
            return []

        if result.returncode != 0:
            return []

        processes = []
        for line in result.stdout.splitlines():
            if 'raspi_browser_launcher.py' not in line:
                processes.append(line)
        return processes

    def browser_has_url(self, url):
        """Check whether a running browser command line already references the target URL."""
        return any(url in process for process in self.get_browser_processes())
    
    def open_browser(self, url):
        """
        Open the browser in fullscreen/kiosk mode.
        
        Args:
            url: URL to open
            
        Returns:
            True if browser opened successfully, False otherwise
        """
        if self.current_url == url and self.is_browser_running():
            print(f"Browser is already open at {url}; not opening another tab")
            return True

        if self.browser_has_url(url):
            print(f"Browser already has {url} open; adopting existing process")
            self.current_url = url
            self.last_opened_url = url
            return True

        if self.last_opened_url == url and time.time() - self.last_open_attempt < self.check_interval:
            print(f"Recently opened {url}; waiting before another browser launch")
            self.current_url = url
            return True

        if self.browser_process is not None and self.browser_process.poll() is None:
            print("Browser already open, closing previous tracked instance...")
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
            self.last_opened_url = url
            self.last_open_attempt = time.time()
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
            elif self.browser_has_url(url) or self.is_browser_running():
                print("✓ Browser command handed off to an existing process")
                self.browser_process = None
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
                self.current_url = None
        else:
            self.current_url = None
    
    def is_browser_running(self):
        """Check if the browser process is still running."""
        if self.browser_process is None:
            return bool(self.get_browser_processes())
        
        # Check if process is still alive
        if self.browser_process.poll() is not None:
            self.browser_process = None
            return bool(self.get_browser_processes())
        
        return True
    
    def is_website_available(self):
        """Check if the current website is still available."""
        if self.current_url is None:
            return False
        
        # Extract IP from URL
        try:
            # URL format: http://<resolved-ip>:1111
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
                url = self.find_website()
                
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
                if self.current_url is None:
                    url = self.find_website()
                    if url:
                        print(f"Browser is already running; adopting {url} without opening another tab")
                        self.current_url = url
                        self.last_opened_url = url
                        time.sleep(self.check_interval)
                        continue
                    if self.running:
                        print(f"Waiting {self.scan_interval} seconds before next hostname check...")
                        time.sleep(self.scan_interval)
                        continue

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
        '--hostname',
        type=str,
        default='hostname.local',
        help='Target .local hostname to resolve (default: hostname.local)'
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
    parser.add_argument(
        '--ping-timeout',
        type=int,
        default=2,
        help='Seconds to wait for hostname ping resolution (default: 2)'
    )
    
    args = parser.parse_args()
    
    # Create and run launcher
    launcher = BrowserLauncher(
        hostname=args.hostname,
        port=args.port,
        scan_interval=args.scan_interval,
        check_interval=args.check_interval,
        ping_timeout=args.ping_timeout
    )
    
    try:
        launcher.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        launcher.close_browser()
        sys.exit(0)


if __name__ == '__main__':
    main()
