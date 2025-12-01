#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

REAL_LIDAR_DISTANCE_CAP = 3
REAL_N_SCAN_SAMPLES = 360

# Robot dimensions (meters)
ROBOT_LENGTH = 0.70
ROBOT_WIDTH = 0.37
LIDAR_OFFSET_FRONT = 0.14  # LIDAR from front end

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')

        # 1) 设置 ROS 订阅
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/processed_scan',
            self.listener_callback,
            10)

        # 2) 初始化数据
        self.scan_ranges = np.ones(REAL_N_SCAN_SAMPLES)
        self.angles     = np.linspace(-np.pi, np.pi, REAL_N_SCAN_SAMPLES)

        # 3) 构建 matplotlib 图形 —— 一定要在这里创建 fig、ax
        self.fig = plt.figure(figsize=(6, 6))
        self.ax  = self.fig.add_subplot(111, polar=True)
        self.line, = self.ax.plot([], [], lw=2, label='LiDAR Scan')
        self.robot_outline, = self.ax.plot([], [], 'r--', lw=1.5, label='Robot Body')
        self.closest_point = self.ax.scatter([], [], c='red', s=40, label='Closest Obstacle')
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(1)
        self.ax.set_rmax(REAL_LIDAR_DISTANCE_CAP)
        self.ax.grid(True)
        self.ax.legend()
        self.robot_theta, self.robot_r = self.compute_robot_outline()

        # 4) 在 fig 和 ax 都已定义后再创建动画
        self.ani = animation.FuncAnimation(
            self.fig,
            self.update_plot,
            interval=100,
            blit=True,
            cache_frame_data=False
        )

    # … listener_callback 不变 …

    def update_plot(self, frame):
        distances = self.scan_ranges * REAL_LIDAR_DISTANCE_CAP
        self.line.set_data(self.angles, distances)
        self.robot_outline.set_data(self.robot_theta, self.robot_r)

        # 找出最近点
        min_idx = np.argmin(distances)
        min_distance = distances[min_idx]

        if min_distance < REAL_LIDAR_DISTANCE_CAP:
            min_angle = self.angles[min_idx]
            # 这里修正方向用 display_angle，如果不需要可以直接用 min_angle
            display_angle = -min_angle
            # 正确的 set_offsets 输入形状为 (1,2)
            self.closest_point.set_offsets([[display_angle, min_distance]])
        else:
            # 如果没有有效点，传入空的 (0,2) 数组
            self.closest_point.set_offsets(np.empty((0, 2)))

        return self.line, self.robot_outline, self.closest_point
    
    def compute_robot_outline(self):
        # 正确使用尺寸
        half_l = ROBOT_WIDTH / 2      # 0.185
        half_w= ROBOT_LENGTH / 2     # 0.29
    
        # 雷达在中心线往后偏 0.14 米，所以机器人中心在雷达后方
        y_shift = -half_l + LIDAR_OFFSET_FRONT
    
        # 绘制轮廓（相对于雷达）
        corners = np.array([
            [-half_w, -half_l - y_shift],  # 左后
            [ half_w, -half_l - y_shift],  # 右后
            [ half_w,  half_l - y_shift],  # 右前
            [-half_w,  half_l - y_shift],  # 左前
            [-half_w, -half_l - y_shift],  # 回到起点
        ])
    
        xs, ys = corners[:, 0], corners[:, 1]
        rs = np.hypot(xs, ys)
        thetas = np.arctan2(ys, xs)  # 👈 修复关键点，反转角度方向
        return thetas, rs

    def listener_callback(self, msg):
        data = np.array(msg.data)
        if len(data) == REAL_N_SCAN_SAMPLES:
            self.scan_ranges = data
        else:
            self.get_logger().warn(f"Received scan length {len(data)} != expected {REAL_N_SCAN_SAMPLES}")

    def spin_visualizer(self):
        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    visualizer = LidarVisualizer()

    ros_thread = threading.Thread(target=rclpy.spin, args=(visualizer,), daemon=True)
    ros_thread.start()

    visualizer.spin_visualizer()

    visualizer.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
