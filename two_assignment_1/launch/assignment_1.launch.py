from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from pathlib import Path
import shutil
import os

def find_terminal_cmd():
    if shutil.which('gnome-terminal'):
        return ('gnome-terminal', ['--', 'bash', '-c'], 'exec bash')
    if shutil.which('konsole'):
        return ('konsole', ['-e', 'bash', '-c'], 'exec bash')
    if shutil.which('xterm'):
        return ('xterm', ['-hold', '-e', 'bash', '-c'], None)
    raise RuntimeError('No supported terminal emulator found (gnome-terminal, konsole, xterm).')

def make_terminal_execute(workspace_path: str, node_cmd: str, title: str):
    terminal, prefix_args, keep_open = find_terminal_cmd()
    workspace_path = str(Path(workspace_path).resolve())
    source_file = os.path.join(workspace_path, 'install', 'setup.bash')
   
    if keep_open:
        shell_cmd = f'source "{source_file}" && {node_cmd}; {keep_open}'
    else:
        shell_cmd = f'source "{source_file}" && {node_cmd}'

    cmd = [terminal] + prefix_args + [shell_cmd]
    return ExecuteProcess(
        cmd=cmd,
        name=f"terminal-{title}",
        output='screen',
        shell=False,
    )

def detect_workspace_root():
    cwd = Path.cwd().resolve()
    for p in [cwd] + list(cwd.parents):
        if (p / 'src').exists() and (p / 'install').exists():
            return str(p)
    return str(cwd)

def generate_launch_description():
    workspace_path = detect_workspace_root()

    ld = LaunchDescription()
    ld.add_action(LogInfo(msg=f"Detected workspace: {workspace_path}"))

    commands = [
        ("Map-Odom Transform Publisher", "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map odom"),
        ("Apriltag Gazebo", "ros2 launch apriltag_ros apriltag_gazebo.launch.yml"),
        ("Apriltag Manager", "ros2 run two_assignment_1 apriltag_manager_node"),
        ("Driver Node", "ros2 run two_assignment_1 driver_node"),
        ("Table Detector", "ros2 run two_assignment_1 table_detector_node")
    ]

    for title, cmd in commands:
        try:
            ld.add_action(make_terminal_execute(workspace_path, cmd, title.replace(' ', '_')))
        except RuntimeError as e:
            ld.add_action(LogInfo(msg=f"Could not open terminal for {title}: {e}"))

    return ld
