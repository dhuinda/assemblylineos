#!/usr/bin/env python3
"""
Setup script to configure raspi_browser_launcher.py as a systemd service
that starts automatically at boot on a Raspberry Pi.

This script:
1. Creates a systemd service file
2. Installs it to /etc/systemd/system/
3. Enables it to start at boot
4. Optionally starts the service immediately

Usage:
    sudo python3 setup_browser_service.py [--start]
"""

import os
import sys
import subprocess
import argparse
from pathlib import Path


def get_script_directory():
    """Get the directory where this setup script is located.
    
    This is used to find raspi_browser_launcher.py in the same directory.
    """
    script_path = Path(__file__).resolve()
    script_dir = script_path.parent
    return script_dir


def get_user_info():
    """Get current user and home directory."""
    # When running with sudo, SUDO_USER contains the original user
    # If not set, try to get the actual user from the environment
    user = os.environ.get('SUDO_USER')
    
    if not user:
        # Try to get user from whoami command (more reliable)
        try:
            result = subprocess.run(['whoami'], capture_output=True, text=True, check=True)
            current_user = result.stdout.strip()
            # If we're root, try to find a non-root user
            if current_user == 'root':
                # Try to get the first non-root user with a home directory
                import pwd
                for p in pwd.getpwall():
                    if p.pw_uid >= 1000 and p.pw_dir and Path(p.pw_dir).exists():
                        user = p.pw_name
                        break
            else:
                user = current_user
        except Exception:
            # Fallback to environment variable
            user = os.environ.get('USER', 'pi')  # Default to 'pi' for Raspberry Pi
    
    if not user or user == 'root':
        print("⚠ Warning: Could not determine non-root user.")
        print("  The service will run as root, which may cause permission issues.")
        print("  Consider specifying a user with: sudo -u <username> python3 setup_browser_service.py")
        user = 'root'
    
    # Get home directory for the user
    try:
        import pwd
        user_info = pwd.getpwnam(user)
        home = user_info.pw_dir
    except KeyError:
        # User doesn't exist, use default
        home = f'/home/{user}' if user != 'root' else '/root'
    
    return user, home


def ensure_launcher_script(script_dir, user):
    """Ensure raspi_browser_launcher.py exists and has proper permissions."""
    script_path = script_dir / 'raspi_browser_launcher.py'
    
    if not script_path.exists():
        print(f"✗ Error: raspi_browser_launcher.py not found at {script_path}")
        print("  This script should be in the workspace root directory.")
        return False
    
    # Make script executable
    try:
        os.chmod(script_path, 0o755)
        # Change ownership to the service user
        import pwd
        try:
            user_info = pwd.getpwnam(user)
            os.chown(script_path, user_info.pw_uid, user_info.pw_gid)
        except (KeyError, AttributeError):
            pass  # If user lookup fails, continue anyway
        return True
    except Exception as e:
        print(f"✗ Error setting permissions on raspi_browser_launcher.py: {e}")
        return False


def create_service_file(script_dir, user, home, ip_start=100, ip_end=199, port=1111):
    """Create the systemd service file content."""
    script_dir_str = str(script_dir)
    service_name = 'raspi-browser-launcher.service'
    script_path = script_dir / 'raspi_browser_launcher.py'
    
    service_content = f"""[Unit]
Description=Raspberry Pi Browser Launcher Service
After=network.target graphical.target
Wants=graphical.target

[Service]
Type=simple
User={user}
Group={user}
WorkingDirectory={script_dir_str}

# Environment variables for X11/display
Environment="HOME={home}"
Environment="USER={user}"
Environment="DISPLAY=:0"
Environment="XAUTHORITY=/home/{user}/.Xauthority"

# Wait for X server to be ready
ExecStartPre=/bin/sleep 10

ExecStart=/usr/bin/env python3 {script_path} --ip-start {ip_start} --ip-end {ip_end} --port {port}

# Restart policy
Restart=always
RestartSec=10

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=raspi-browser-launcher

[Install]
WantedBy=graphical.target
"""
    
    return service_content, service_name


def install_service(service_content, service_name):
    """Install the service file to systemd."""
    service_path = Path(f'/etc/systemd/system/{service_name}')
    
    print(f"Creating service file: {service_path}")
    try:
        service_path.write_text(service_content)
        print(f"✓ Service file created successfully")
        return True
    except PermissionError:
        print("✗ Error: Permission denied. Please run with sudo.")
        return False
    except Exception as e:
        print(f"✗ Error creating service file: {e}")
        return False


def reload_systemd():
    """Reload systemd daemon."""
    try:
        subprocess.run(['systemctl', 'daemon-reload'], check=True, capture_output=True)
        print("✓ Systemd daemon reloaded")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ Error reloading systemd: {e}")
        return False
    except FileNotFoundError:
        print("✗ Error: systemctl not found. Are you on a systemd-based system?")
        return False


def enable_service(service_name):
    """Enable the service to start at boot."""
    try:
        subprocess.run(['systemctl', 'enable', service_name], check=True, capture_output=True)
        print(f"✓ Service enabled to start at boot")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ Error enabling service: {e}")
        return False


def start_service(service_name):
    """Start the service immediately."""
    try:
        result = subprocess.run(['systemctl', 'start', service_name], 
                              check=True, capture_output=True, text=True)
        print(f"✓ Service started successfully")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ Error starting service: {e.stderr if hasattr(e, 'stderr') else e}")
        print("\nYou can check the service status with:")
        print(f"  sudo systemctl status {service_name}")
        print("\nYou can view logs with:")
        print(f"  sudo journalctl -u {service_name} -f")
        return False


def stop_service(service_name):
    """Stop the service if it's running."""
    try:
        # Check if service is active
        result = subprocess.run(
            ['systemctl', 'is-active', service_name],
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            print(f"Stopping service {service_name}...")
            subprocess.run(['systemctl', 'stop', service_name], 
                         check=False, capture_output=True)
            print("✓ Service stopped")
            return True
        else:
            print("Service is not running")
            return True
    except Exception as e:
        print(f"⚠ Warning: Could not stop service: {e}")
        return False


def disable_service(service_name):
    """Disable the service if it's enabled."""
    try:
        # Check if service is enabled
        result = subprocess.run(
            ['systemctl', 'is-enabled', service_name],
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            print(f"Disabling service {service_name}...")
            subprocess.run(['systemctl', 'disable', service_name], 
                         check=False, capture_output=True)
            print("✓ Service disabled")
            return True
        else:
            print("Service is not enabled")
            return True
    except Exception as e:
        print(f"⚠ Warning: Could not disable service: {e}")
        return False


def remove_old_service_file(service_name):
    """Remove the old service file if it exists."""
    service_path = Path(f'/etc/systemd/system/{service_name}')
    
    if service_path.exists():
        print(f"Removing old service file: {service_path}")
        try:
            service_path.unlink()
            print("✓ Old service file removed")
            return True
        except Exception as e:
            print(f"⚠ Warning: Could not remove old service file: {e}")
            return False
    else:
        print("No existing service file found")
        return True


def check_root():
    """Check if running as root (required for systemd operations)."""
    if os.geteuid() != 0:
        print("✗ Error: This script must be run with sudo")
        print("Usage: sudo python3 setup_browser_service.py [--start]")
        return False
    return True


def main():
    parser = argparse.ArgumentParser(
        description='Setup raspi_browser_launcher as a systemd service'
    )
    parser.add_argument(
        '--start',
        action='store_true',
        help='Start the service immediately after installation'
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
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("Raspberry Pi Browser Launcher - Systemd Service Setup")
    print("=" * 60)
    print()
    
    # Check for root privileges
    if not check_root():
        sys.exit(1)
    
    # Get paths and configuration
    script_dir = get_script_directory()
    user, home = get_user_info()
    
    print(f"Script directory: {script_dir}")
    print(f"User: {user}")
    print(f"Home: {home}")
    print(f"IP Range: 192.168.0.{args.ip_start}-{args.ip_end}")
    print(f"Port: {args.port}")
    print()
    
    # Ensure launcher script exists and has proper permissions
    print("Checking raspi_browser_launcher.py...")
    if not ensure_launcher_script(script_dir, user):
        sys.exit(1)
    print("✓ raspi_browser_launcher.py is ready")
    
    # Get service name
    service_content, service_name = create_service_file(
        script_dir, 
        user, 
        home,
        ip_start=args.ip_start,
        ip_end=args.ip_end,
        port=args.port
    )
    
    # Reinstall the service cleanly
    print()
    print("Reinstalling service...")
    print("-" * 60)
    
    # Stop the service if it's running
    stop_service(service_name)
    
    # Disable the service if it's enabled
    disable_service(service_name)
    
    # Remove old service file
    remove_old_service_file(service_name)
    
    # Reload systemd to ensure old service is gone
    subprocess.run(['systemctl', 'daemon-reload'], check=False, capture_output=True)
    
    print()
    print("Installing new service...")
    print("-" * 60)
    
    # Install service
    if not install_service(service_content, service_name):
        sys.exit(1)
    
    # Reload systemd
    if not reload_systemd():
        sys.exit(1)
    
    # Enable service
    if not enable_service(service_name):
        sys.exit(1)
    
    # Start service if requested
    if args.start:
        print()
        if not start_service(service_name):
            sys.exit(1)
    
    print()
    print("=" * 60)
    print("Setup Complete!")
    print("=" * 60)
    print()
    print("Service management commands:")
    print(f"  Start:   sudo systemctl start {service_name}")
    print(f"  Stop:    sudo systemctl stop {service_name}")
    print(f"  Restart: sudo systemctl restart {service_name}")
    print(f"  Status:  sudo systemctl status {service_name}")
    print(f"  Logs:    sudo journalctl -u {service_name} -f")
    print(f"  Disable: sudo systemctl disable {service_name}")
    print()
    
    if not args.start:
        print("To start the service now, run:")
        print(f"  sudo systemctl start {service_name}")
        print()
    
    print("Note: Make sure Chromium is installed:")
    print("  sudo apt-get update")
    print("  sudo apt-get install chromium-browser")
    print()


if __name__ == '__main__':
    main()
