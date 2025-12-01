# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='pointcloud_to_laserscan',
#             executable='PointCloudToScanNode',   # 若你是组件容器跑法，换成 composable 用法
#             name='pointcloud_to_scan',
#             output='screen',
#             parameters=[
#                 {'num_scan_samples': 360},     # 目标 360×1
#                 {'raw_bins': 1440},            # 原始角分辨率 R(≥360)，如 1440/1800
#                 {'range_min': 0.20},           # 部署一致的近端裁剪
#                 {'range_max': 3.0},            # 量程上限
#                 {'normalize': True},           # 与训练保持一致
#                 {'min_points_per_raw_bin': 0}, # 与仿真等价时设 0（空桶=最远）
#             ],
#             remappings=[
#                 ('input_cloud', '/notGround_pointCloud'),
#                 ('processed_scan', '/processed_scan'),
#             ],
#         ),
#     ])
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='PointCloudToScanNode',   # 你的独立可执行名
            name='pointcloud_to_scan',
            output='screen',
            parameters=[{
                'num_scan_samples': 360,
                'raw_bins': 1440,
                'range_min': 0.20,           # ★
                'range_max': 5.0,
                'normalize': True,
                'min_points_per_raw_bin': 0,
                'angle_offset_deg': 0.0,
                'invert_yaw': False,
            }],
            remappings=[
                ('input_cloud', '/notGround_pointCloud'),
                ('processed_scan', '/processed_scan'),
            ],
        ),
    ])
