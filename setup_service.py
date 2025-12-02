#!/usr/bin/env python3
"""
Setup script to configure assembly_line_control.launch.py as a systemd service
that starts automatically at boot.

This script:
1. Creates a systemd service file
2. Installs it to /etc/systemd/system/
3. Enables it to start at boot
4. Optionally starts the service immediately

Usage:
    sudo python3 setup_service.py [--start]
"""

import os
import sys
import subprocess
import argparse
from pathlib import Path


def get_workspace_path():
    """Get the workspace path (parent of src directory)."""
    script_path = Path(__file__).resolve()
    workspace_path = script_path.parent
    return workspace_path


def get_ros_distro():
    """Detect ROS 2 distribution."""
    # Common ROS 2 distributions
    ros_distros = ['humble', 'iron', 'jazzy', 'rolling']
    
    # Check if ROS_DISTRO is set
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        return ros_distro
    
    # Try to detect from common installation paths
    for distro in ros_distros:
        setup_path = Path(f'/opt/ros/{distro}/setup.bash')
        if setup_path.exists():
            return distro
    
    # Default to humble if not found
    print("Warning: Could not detect ROS distribution, defaulting to 'humble'")
    return 'humble'


def get_user_info():
    """Get current user and home directory."""
    user = os.environ.get('SUDO_USER') or os.environ.get('USER') or 'root'
    home = os.environ.get('HOME') or f'/home/{user}'
    return user, home


def create_startup_script(workspace_path, ros_distro, home):
    """Create the startup script that handles git pull, build, and launch."""
    workspace_str = str(workspace_path)
    script_path = workspace_path / 'start_service.sh'
    
    script_content = f"""#!/bin/bash
# Startup script for Assembly Line Control service
# This script sources ROS, pulls updates, builds, and launches the app

set -e

# Ensure HOME and related environment variables are set correctly (colcon needs these)
export HOME={home}
export USER="${{USER:-$(whoami)}}"
export XDG_CONFIG_HOME="${{XDG_CONFIG_HOME:-$HOME/.config}}"
export XDG_CACHE_HOME="${{XDG_CACHE_HOME:-$HOME/.cache}}"

# Ensure colcon directories exist with proper permissions
mkdir -p "$HOME/.colcon" 2>/dev/null || true

# Source ROS setup
source /opt/ros/{ros_distro}/setup.bash

# Change to workspace directory
cd {workspace_str}

# Try git pull if possible (only if working tree is clean)
if [ -d .git ]; then
    git fetch || true
    if [ -z "$(git status --porcelain)" ]; then
        git pull || true
    fi
fi

# Ensure install, build, and log directories exist and have correct permissions
mkdir -p {workspace_str}/install {workspace_str}/build {workspace_str}/log
chmod -R u+w {workspace_str}/install {workspace_str}/build {workspace_str}/log 2>/dev/null || true

# Build the workspace (without symlinks to avoid permission issues)
colcon build

# Source the workspace setup (if it exists)
if [ -f {workspace_str}/install/setup.bash ]; then
    source {workspace_str}/install/setup.bash
fi

# Launch the application
exec ros2 launch assembly_line_control assembly_line_control.launch.py
"""
    
    return script_path, script_content


def create_service_file(workspace_path, ros_distro, user, home):
    """Create the systemd service file content."""
    workspace_str = str(workspace_path)
    service_name = 'assembly-line-control.service'
    script_path = workspace_path / 'start_service.sh'
    
    service_content = f"""[Unit]
Description=Assembly Line Control ROS 2 Launch Service
After=network.target

[Service]
Type=simple
User={user}
Group={user}
WorkingDirectory={workspace_str}

# Environment variables
Environment="HOME={home}"
Environment="USER={user}"

ExecStart={script_path}

# Restart policy
Restart=always
RestartSec=10

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=assembly-line-control

[Install]
WantedBy=multi-user.target
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


def check_root():
    """Check if running as root (required for systemd operations)."""
    if os.geteuid() != 0:
        print("✗ Error: This script must be run with sudo")
        print("Usage: sudo python3 setup_service.py [--start]")
        return False
    return True


def main():
    parser = argparse.ArgumentParser(
        description='Setup assembly_line_control as a systemd service'
    )
    parser.add_argument(
        '--start',
        action='store_true',
        help='Start the service immediately after installation'
    )
    parser.add_argument(
        '--ros-distro',
        type=str,
        help='ROS 2 distribution (default: auto-detect)'
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("Assembly Line Control - Systemd Service Setup")
    print("=" * 60)
    print()
    
    # Check for root privileges
    if not check_root():
        sys.exit(1)
    
    # Get paths and configuration
    workspace_path = get_workspace_path()
    ros_distro = args.ros_distro or get_ros_distro()
    user, home = get_user_info()
    
    print(f"Workspace: {workspace_path}")
    print(f"ROS Distribution: {ros_distro}")
    print(f"User: {user}")
    print(f"Home: {home}")
    print()
    
    # Verify workspace structure
    if not (workspace_path / 'src' / 'assembly_line_control').exists():
        print("✗ Error: Could not find assembly_line_control package")
        print(f"  Expected at: {workspace_path}/src/assembly_line_control")
        sys.exit(1)
    
    if not (workspace_path / 'install' / 'setup.bash').exists():
        print("⚠ Warning: install/setup.bash not found")
        print("  The service will build the workspace on first start.")
        print()
    
    # Create startup script
    script_path, script_content = create_startup_script(workspace_path, ros_distro, home)
    print(f"Creating startup script: {script_path}")
    try:
        script_path.write_text(script_content)
        # Make script executable
        os.chmod(script_path, 0o755)
        # Change ownership to the service user
        import pwd
        try:
            user_info = pwd.getpwnam(user)
            os.chown(script_path, user_info.pw_uid, user_info.pw_gid)
            # Also fix ownership of install/build/log directories if they exist
            for dir_name in ['install', 'build', 'log']:
                dir_path = workspace_path / dir_name
                if dir_path.exists():
                    # Use subprocess to change ownership recursively (more reliable)
                    subprocess.run(['chown', '-R', f'{user}:{user}', str(dir_path)], 
                                 capture_output=True, check=False)
        except (KeyError, AttributeError):
            # If user lookup fails, continue anyway
            pass
        print("✓ Startup script created successfully")
    except Exception as e:
        print(f"✗ Error creating startup script: {e}")
        sys.exit(1)
    
    # Create service file
    service_content, service_name = create_service_file(workspace_path, ros_distro, user, home)
    
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


if __name__ == '__main__':
    main()

