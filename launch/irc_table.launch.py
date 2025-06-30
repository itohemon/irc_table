import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    # for spawn turtlebot3
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-0.25')
    y_pose = LaunchConfiguration('y_pose', default='-0.25')
    z_pose = LaunchConfiguration('z_pose', default='0.29')

    # for spawn turtlebot3
    turtlebot3_gazebo_launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch')

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(
        get_package_share_directory('irc_table'),
        'worlds',
        'irc_table.sdf'
    )

    gz_launch_path = PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    # for spawn turtlebot3
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_launch_file_dir,
            'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # for spawn turtlebot3
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('irc_table'),
                'launch',
                'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose
        }.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('irc_table'),
                'models'))

    spawn_balls_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('irc_table'),
                'launch',
                'spawn_random_balls.launch.py'
            )
        )
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)       # for spawn turtlebot3
    ld.add_action(robot_state_publisher_cmd) # for spawn turtlebot3
    ld.add_action(spawn_balls_cmd)

    return ld

