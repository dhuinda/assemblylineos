#!/usr/bin/env python3
"""
Service runner for Assembly Line Control.

This script handles:
1. Setting up environment variables
2. Git pull (if working tree is clean)
3. Colcon build
4. Sourcing ROS and workspace setup
5. Launching the ROS 2 application

This replaces the manual steps a human would do.
"""

import os
import sys
import subprocess
from pathlib import Path


def get_workspace_path():
    """Get the workspace path (parent of src directory)."""
    script_path = Path(__file__).resolve()
    workspace_path = script_path.parent
    return workspace_path


def get_ros_distro():
    """Detect ROS 2 distribution."""
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


def setup_environment():
    """Set up environment variables for colcon and ROS."""
    home = os.environ.get('HOME')
    if not home:
        print("Error: HOME environment variable not set")
        sys.exit(1)
    
    # Set environment variables for colcon
    os.environ['COLCON_HOME'] = f'{home}/.colcon'
    os.environ.setdefault('XDG_CONFIG_HOME', f'{home}/.config')
    os.environ.setdefault('XDG_CACHE_HOME', f'{home}/.cache')
    os.environ.setdefault('XDG_DATA_HOME', f'{home}/.local/share')
    
    # Ensure .colcon directory exists with proper permissions
    colcon_home = Path(home) / '.colcon'
    colcon_mixin = colcon_home / 'mixin'
    colcon_log = colcon_home / 'log'
    
    for colcon_dir in [colcon_home, colcon_mixin, colcon_log]:
        colcon_dir.mkdir(parents=True, exist_ok=True)
        # Set permissions
        try:
            os.chmod(colcon_dir, 0o755)
        except Exception:
            pass  # Ignore permission errors if we can't set them


def git_pull(workspace_path):
    """Pull latest changes if working tree is clean."""
    git_dir = workspace_path / '.git'
    if not git_dir.exists():
        print("Not a git repository, skipping git pull")
        return
    
    try:
        # Check if working tree is clean
        result = subprocess.run(
            ['git', 'status', '--porcelain'],
            cwd=workspace_path,
            capture_output=True,
            text=True,
            check=True
        )
        
        if result.stdout.strip():
            print("Working tree has uncommitted changes, skipping git pull")
            return
        
        # Fetch and pull
        print("Pulling latest changes from git...")
        subprocess.run(['git', 'fetch'], cwd=workspace_path, check=False)
        subprocess.run(['git', 'pull'], cwd=workspace_path, check=False)
        print("✓ Git pull completed")
    except subprocess.CalledProcessError as e:
        print(f"Warning: Git operations failed: {e}")
    except Exception as e:
        print(f"Warning: Git pull error: {e}")


def colcon_build(workspace_path, ros_distro):
    """Build the workspace using colcon."""
    print("Building workspace with colcon...")
    
    # Ensure build directories exist
    for dir_name in ['install', 'build', 'log']:
        dir_path = workspace_path / dir_name
        dir_path.mkdir(exist_ok=True)
    
    try:
        # Run colcon build with ROS environment
        run_command_with_ros(ros_distro, workspace_path, 'colcon build', check=True)
        print("✓ Colcon build completed successfully")
    except subprocess.CalledProcessError as e:
        print(f"Error: Colcon build failed: {e}")
        sys.exit(1)
    except FileNotFoundError:
        print("Error: colcon command not found. Is ROS 2 properly sourced?")
        sys.exit(1)


def run_command_with_ros(ros_distro, workspace_path, command, check=True):
    """Run a command with ROS environment sourced."""
    ros_setup = Path(f'/opt/ros/{ros_distro}/setup.bash')
    workspace_setup = workspace_path / 'install' / 'setup.bash'
    
    # Build bash command that sources ROS and workspace setup
    bash_cmd = f'source {ros_setup}'
    if workspace_setup.exists():
        bash_cmd += f' && source {workspace_setup}'
    bash_cmd += f' && cd {workspace_path} && {command}'
    
    return subprocess.run(
        ['bash', '-c', bash_cmd],
        check=check
    )


def launch_application(workspace_path, ros_distro):
    """Launch the ROS 2 application."""
    print("Launching assembly_line_control...")
    
    try:
        # Run ros2 launch with ROS environment sourced
        # Use exec to replace this process
        ros_setup = Path(f'/opt/ros/{ros_distro}/setup.bash')
        workspace_setup = workspace_path / 'install' / 'setup.bash'
        
        bash_cmd = f'source {ros_setup}'
        if workspace_setup.exists():
            bash_cmd += f' && source {workspace_setup}'
        bash_cmd += f' && cd {workspace_path} && exec ros2 launch assembly_line_control assembly_line_control.launch.py'
        
        os.execv('/bin/bash', ['bash', '-c', bash_cmd])
    except FileNotFoundError:
        print("Error: ros2 command not found. Is ROS 2 properly sourced?")
        sys.exit(1)
    except Exception as e:
        print(f"Error launching application: {e}")
        sys.exit(1)


def main():
    """Main entry point."""
    print("=" * 60)
    print("Assembly Line Control - Service Runner")
    print("=" * 60)
    print()
    
    # Get workspace path
    workspace_path = get_workspace_path()
    print(f"Workspace: {workspace_path}")
    
    # Change to workspace directory
    os.chdir(workspace_path)
    
    # Setup environment
    setup_environment()
    
    # Detect ROS distribution
    ros_distro = get_ros_distro()
    print(f"ROS Distribution: {ros_distro}")
    print()
    
    # Git pull
    git_pull(workspace_path)
    print()
    
    # Colcon build (this will source ROS internally)
    colcon_build(workspace_path, ros_distro)
    print()
    
    # Launch application (this will source ROS and workspace setup)
    launch_application(workspace_path, ros_distro)


if __name__ == '__main__':
    main()

