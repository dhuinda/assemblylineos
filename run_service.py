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


def setup_environment(workspace_path):
    """Set up environment variables for colcon and ROS.
    
    Uses workspace-local directories to keep everything self-contained.
    """
    # Use workspace-local directory for colcon config (self-contained)
    colcon_home = workspace_path / '.colcon'
    colcon_mixin = colcon_home / 'mixin'
    colcon_log = colcon_home / 'log'
    
    # Set COLCON_HOME to workspace-local directory
    os.environ['COLCON_HOME'] = str(colcon_home)
    
    # Create colcon directories in workspace (self-contained, no home directory needed)
    print(f"Setting up workspace-local colcon directory at {colcon_home}")
    for colcon_dir in [colcon_home, colcon_mixin, colcon_log]:
        try:
            colcon_dir.mkdir(parents=True, exist_ok=True)
            os.chmod(colcon_dir, 0o755)
        except Exception as e:
            print(f"⚠ Warning: Could not create {colcon_dir}: {e}")
    
    print(f"✓ Environment setup complete (COLCON_HOME={colcon_home})")


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
    """Run a command with ROS environment sourced.
    
    Uses workspace-local COLCON_HOME to keep everything self-contained.
    """
    ros_setup = Path(f'/opt/ros/{ros_distro}/setup.bash')
    workspace_setup = workspace_path / 'install' / 'setup.bash'
    
    # Use workspace-local colcon directory (self-contained)
    colcon_home = workspace_path / '.colcon'
    
    # Build bash command that sources ROS and workspace setup
    # Export COLCON_HOME to workspace-local directory
    bash_cmd = f'export COLCON_HOME="{colcon_home}" && '
    bash_cmd += f'source {ros_setup}'
    if workspace_setup.exists():
        bash_cmd += f' && source {workspace_setup}'
    bash_cmd += f' && cd {workspace_path} && {command}'
    
    # Pass the current environment to the subprocess
    env = os.environ.copy()
    env['COLCON_HOME'] = str(colcon_home)
    
    return subprocess.run(
        ['bash', '-c', bash_cmd],
        check=check,
        env=env
    )


def launch_application(workspace_path, ros_distro):
    """Launch the ROS 2 application."""
    print("Launching assembly_line_control...")
    
    try:
        # Run ros2 launch with ROS environment sourced
        # Use exec to replace this process
        ros_setup = Path(f'/opt/ros/{ros_distro}/setup.bash')
        workspace_setup = workspace_path / 'install' / 'setup.bash'
        
        # Use workspace-local colcon directory (self-contained)
        colcon_home = workspace_path / '.colcon'
        
        bash_cmd = f'export COLCON_HOME="{colcon_home}" && '
        bash_cmd += f'source {ros_setup}'
        if workspace_setup.exists():
            bash_cmd += f' && source {workspace_setup}'
        bash_cmd += f' && cd {workspace_path} && exec ros2 launch assembly_line_control assembly_line_control.launch.py'
        
        # Set environment for the exec
        env = os.environ.copy()
        env['COLCON_HOME'] = str(colcon_home)
        
        os.execve('/bin/bash', ['bash', '-c', bash_cmd], env)
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
    
    # Log current user info (for debugging)
    current_user = os.environ.get('USER', 'unknown')
    current_uid = os.geteuid()
    
    print(f"Running as user: {current_user} (UID: {current_uid})")
    
    # Get workspace path
    workspace_path = get_workspace_path()
    print(f"Workspace: {workspace_path}")
    
    # Change to workspace directory
    os.chdir(workspace_path)
    
    # Setup environment (workspace-local, self-contained)
    setup_environment(workspace_path)
    
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

