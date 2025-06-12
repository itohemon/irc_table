from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    irc_table_pkg_path = FindPackageShare('irc_table')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([irc_table_pkg_path, 'models'])
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            PathJoinSubstitution([irc_table_pkg_path, 'plugins'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([irc_table_pkg_path, 'worlds/irc_table.sdf'])],
                'on_exit_shutdown': 'True'
            }.items(),
        ),

        ## Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
        #Node(
        #    package='ros_gz_bridge',
        #    executable='parameter_bridge',
        #    arguments=['/example_imu_topic@sensor_msgs/msg/Imu@gz.msgs.IMU',],
        #    remappings=[('/example_imu_topic',
        #                 '/remapped_imu_topic'),],
        #    output='screen'
        #),
    ])
