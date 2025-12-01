#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ProcessedScanNode(Node):
    def __init__(self):
        super().__init__('processed_scan_subscriber')

        # 创建订阅者，订阅 processed_scan 话题
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'processed_scan',
            self.scan_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg: Float32MultiArray):
        # msg.data 是一个归一化后的 [0,1] 数组（长度 = res_x * res_y，例如 900）
        data = msg.data

        # 打印前 10 个值
        self.get_logger().info(f'Got scan data (first 10): {data[:10]}')

        # 如果你想再发布出去，可以这样做：
        # out_pub = Float32MultiArray()
        # out_pub.data = data   
        # self.publisher.publish(out_pub)

def main(args=None):
    rclpy.init(args=args)
    node = ProcessedScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
