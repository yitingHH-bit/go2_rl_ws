# # launch/subscribe_height_map.launch.py
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# def generate_launch_description():
#     nic_arg = DeclareLaunchArgument(
#         "nic",
#         description="Network interface name used by Unitree DDS (e.g., enp3s0, eth0)"
#     )

#     odom_arg = DeclareLaunchArgument(
#         "odom_topic",
#         default_value="/utlidar/robot_odom",
#         description="Odometry topic providing robot pose in odom frame"
#     )

#     node = Node(
#         package="pointcloud_to_laserscan",
#         executable="subscribe_height_map",
#         name="subscribe_height_map_Node",
#         output="screen",
#         # 将网卡名作为程序参数传入（你的 main(argc, argv) 里会读取）
#         arguments=[LaunchConfiguration("nic")],
#         # 允许通过 launch 参数重映射里程计话题
#         remappings=[
#             ("/utlidar/robot_odom", LaunchConfiguration("odom_topic")),
#         ],
#     )

#     return LaunchDescription([
#         nic_arg,
#         odom_arg,
#         node
#     ])
