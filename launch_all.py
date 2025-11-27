#!/usr/bin/env python3
"""
Script to launch all required nodes in separate terminals.
Opens 5 terminals in the ROS2 workspace and runs the necessary commands.
"""

import subprocess
import os
import sys
from pathlib import Path

def find_workspace_root():
    """Find the ROS2 workspace root by looking for 'src' and 'install' directories."""
    current = Path.cwd()
    
    # Check current directory
    if (current / 'src').exists() and (current / 'install').exists():
        return current
    
    # Check parent directories
    for parent in current.parents:
        if (parent / 'src').exists() and (parent / 'install').exists():
            return parent
    
    # Default fallback - assume we're in a subdirectory
    # Try common patterns
    if 'ws_2_assignments' in str(current):
        parts = str(current).split('ws_2_assignments')
        ws_path = parts[0] + 'ws_2_assignments'
        return Path(ws_path)
    
    print("ERROR: Could not find ROS2 workspace root!")
    print("Please run this script from within your workspace directory.")
    sys.exit(1)

def open_terminal_with_command(workspace_path, command, title):
    """
    Open a new terminal window and execute a command.
    Works on Linux with gnome-terminal, konsole, or xterm.
    """
    workspace_str = str(workspace_path)
    
    # Full command: change directory, source setup, then run the command
    full_cmd = f"cd {workspace_str} && source install/setup.bash && {command}"
    
    # Try different terminal emulators
    terminal_commands = [
        # GNOME Terminal (Ubuntu default)
        ['gnome-terminal', '--title', title, '--', 'bash', '-c', f'{full_cmd}; exec bash'],
        # Konsole (KDE)
        ['konsole', '--title', title, '-e', 'bash', '-c', f'{full_cmd}; exec bash'],
        # xterm (fallback)
        ['xterm', '-T', title, '-e', f'bash -c "{full_cmd}; exec bash"'],
    ]
    
    for term_cmd in terminal_commands:
        try:
            subprocess.Popen(term_cmd)
            return True
        except FileNotFoundError:
            continue
    
    print(f"ERROR: Could not find a suitable terminal emulator!")
    print(f"Tried: gnome-terminal, konsole, xterm")
    return False

def main():
    print("=== ROS2 Assignment Launcher ===\n")
    
    # Find workspace
    workspace = find_workspace_root()
    print(f"Workspace found: {workspace}\n")
    
    # Define commands to run
    commands = [
        {
            'title': '1-AprilTag Gazebo',
            'command': f"ros2 launch {workspace}/src/apriltag_ros/launch/apriltag_gazebo.launch.yml"
        },
        {
            'title': '2-Assignment 1',
            'command': f"ros2 launch {workspace}/src/ir_2526/ir_launch/launch/assignment_1.launch.py"
        },
        {
            'title': '3-AprilTag Manager',
            'command': 'ros2 run two_assignment_1 apriltag_manager_node'
        },
        {
            'title': '4-Driver Node',
            'command': 'ros2 run two_assignment_1 driver_node'
        },
        {
            'title': '5-Table Detector',
            'command': 'ros2 run two_assignment_1 table_detector_node'
        }
    ]
    
    print("Opening terminals...\n")
    
    success_count = 0
    for cmd_info in commands:
        print(f"Launching: {cmd_info['title']}")
        if open_terminal_with_command(workspace, cmd_info['command'], cmd_info['title']):
            success_count += 1
        else:
            print(f"  └─ Failed to open terminal for {cmd_info['title']}")
    
    print(f"\n=== Done ===")
    print(f"Successfully opened {success_count}/{len(commands)} terminals")
    
    if success_count < len(commands):
        print("\nIf terminals didn't open, you can run commands manually:")
        for cmd_info in commands:
            print(f"\n# {cmd_info['title']}")
            print(f"cd {workspace}")
            print("source install/setup.bash")
            print(cmd_info['command'])

if __name__ == '__main__':
    main()
