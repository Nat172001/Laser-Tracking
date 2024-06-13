from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    bag_file = DeclareLaunchArgument(
        'bag',
        default_value='/home/nataraj/ros2_ws/src/laser/bags/7/example7.db3',
        description='Path to bag file'
    )

    # RViz configuration files
    rviz_config_1 = DeclareLaunchArgument(
        'rviz_config_1',
        default_value='/home/nataraj/.rviz2/config1.rviz',
        description='Path to RViz config for rviz2_1'
    )
    
    rviz_config_2 = DeclareLaunchArgument(
        'rviz_config_2',
        default_value='/home/nataraj/.rviz2/config2.rviz',
        description='Path to RViz config for rviz2_2'
    )

    # Launch rviz2_1
    rviz2_node_1= Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_1',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_1')]  # Load RViz config file
    )

    # Play the bag file
    bag_play_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag')],
        output='screen'
    )

    # Subscribe to /scan topic
    scan_node = Node(
        package='laser',
        executable='scan',
        name='scan_values',
        output='screen'
    )
    
    # run node
    run_node = Node(
        package='laser',
        executable='run',
        name='run_values',
        output='screen'
    )
    
    # Launch rviz2_2
    rviz2_node_2= Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_2')]  # Load RViz config file
    )

    # Record a bag file
    bag_record_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'record','/scan','/centroids','/moving_points','/stationary_points','/tracked_objects', '-o', 'recorded_bag2'],
        output='screen'
    )

    # Ensure the bag record process is terminated when bag play finishes
    bag_play_exit_event = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play_node,
            on_exit=[
                ExecuteProcess(cmd=['pkill', '-f', 'ros2 bag record'])
            ]
        )
    )

    return LaunchDescription([
        bag_file,
        rviz_config_1,
        rviz_config_2,
        rviz2_node_1,
        rviz2_node_2,
        bag_play_node,
        bag_record_node,
        scan_node,
        run_node,
        bag_play_exit_event
    ])

