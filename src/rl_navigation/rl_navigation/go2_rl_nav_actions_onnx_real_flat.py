#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState
from std_msgs.msg import Float32MultiArray
import numpy as np
import onnxruntime as ort
import os
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32MultiArray
EXPECTED_ROWS = 360
EXPECTED_COLS = 1

class Go2_RL_Nav_Actions(Node):
    def __init__(self):
        super().__init__("nav_action_publisher")
        # Creating publisher to actions topic
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'nav_actions',
            10)
        # Creating subscriptions to obs topics
        self.create_subscription(
            SportModeState,
            'sportmodestate',
            self.state_callback,
            10) 
        self.create_subscription(
            Float32MultiArray,  
            'projected_gravity',    
            self.projected_gravity_callback,    
            10)
        self.create_subscription(
            Float32MultiArray,
            'cmd_pose',
            self.cmd_pose_callback,
            10)
        
        # lidar 
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'processed_scan',
            self.scan_callback,
            10
        )
        #
        self.create_subscription(
            Float32MultiArray,
            'base_ang_vel',
            self.base_ang_vel_callback,
            10)
        

        #   base_angle_vel  from sportmodestate 
        #   projected_gravity  from projected_gravity
        #   Current Cmd pose   from cmd_pose  
        #   lidar_scan  "lidar_resolution": (30, 30),       
        #   height_scan_size: 900  

        # Initializing variables    
        self.nav_actions = None
        self.processed_actions = None
        #add clip 
        
        clips = [(0.0, 0.5), (-0.5, 0.5), (-1.0, 1.0)]
        self.clip_low, self.clip_high = map(lambda x: np.array(x, dtype=np.float32), zip(*clips))

        self.base_vel = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.projected_gravity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.current_cmd_pose = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.current_pose = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
                #add  
        self.base_ang_vel = np.zeros(3) 
        self.lidar_scan = np.zeros((360, 1), dtype=np.float32)
        self.model_in = np.zeros((1, 360), dtype=np.float32)  # Initialize model input
        # 
        self.pose_cmd_defaults = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        # Finding and loading the ONNX model - can be changed to desired model
        share_dir = get_package_share_directory('rl_navigation')
        model_path = os.path.join(share_dir, 'models', 'policynew_5900.onnx')
        self.get_logger().info(f"Model path: {model_path}")
        self.load_onnx_model(model_path)

        # Create a timer to generate actions every 20 milliseconds (50 Hz)  
        self.timer_period = 0.02  # 20 milliseconds
        self.timer = self.create_timer(
            self.timer_period,  
            self.generate_actions)  

    def load_onnx_model(self, model_path):
        self.ort_session = ort.InferenceSession(model_path)

    def scan_callback(self, msg):
        import numpy as np
        data = np.asarray(msg.data, dtype=np.float32)  # 长度=360
        if data.size != 360:
            raise ValueError(f"Lidar length={data.size}, expected 360")
        # 训练端实际是 (N,360)，部署端单帧 -> (1,360)
        self.model_in = data.reshape(1, 360)
            # ====== 安全保护机制 ======
        # 如果 normalize=True，这里 data 是 [0,1] 归一化距离
        # 要换算成米：d_m = data * (range_max - range_min) + range_min
        RANGE_MIN = 0.20
        RANGE_MAX = 5.0
        distances_m = data * (RANGE_MAX - RANGE_MIN) + RANGE_MIN

        min_dist = np.min(distances_m)
        if min_dist < 0.35:
            self.get_logger().warn(f"[SAFETY STOP] Obstacle too close: {min_dist:.2f} m — sending stop command")
            stop_msg = Float32MultiArray()
            stop_msg.data = [0.0, 0.0, 0.0]  # 你机器人停止的动作向量（按你的 action 格式填）
            self.publisher.publish(stop_msg)
            # 这里可以加个标志位防止 generate_actions 再发控制
            self.safety_stop = True
        else:
            self.safety_stop = False

    def base_ang_vel_callback(self, msg):
        self.base_ang_vel = np.array(msg.data, dtype=np.float32)

    def state_callback(self, msg):
        # If first run, set the initial pose to be the current position reading with heading zero
        if not hasattr(self, 'initial_xyz') or not hasattr(self, 'initial_heading'):
            self.initial_xyz = np.array(msg.position, dtype=np.float32)
            self.initial_heading = np.array([msg.imu_state.rpy[2]], dtype=np.float32)
            self.initial_pose = np.concatenate((self.initial_xyz, self.initial_heading))
        
        # Get current pose
        self.current_xyz = np.array(msg.position, dtype=np.float32)
        self.current_heading = np.array([msg.imu_state.rpy[2]], dtype=np.float32)
        self.current_pose = np.concatenate((self.current_xyz, self.current_heading)) - self.initial_pose

        # Base velocity observation         
        self.base_vel = np.array(msg.velocity, dtype=np.float32)

    def projected_gravity_callback(self, msg):
        self.projected_gravity = np.array(msg.data, dtype=np.float32)

    def cmd_pose_callback(self, msg):
        # Calling current pose to update the pose command as approaching the goal
        if not hasattr(self, "initial_cmd_pose"):
            self.initial_cmd_pose = np.array(msg.data, dtype=np.float32) - self.initial_pose

        # Updating command pose based on progress made
        self.current_cmd_pose = self.initial_cmd_pose - self.current_pose

    def generate_actions(self):
        
        if getattr(self, "safety_stop", False):
            return
        # Log the current state of the input components 
        self.get_logger().info(f"Base Velocity: {self.base_vel}")
        self.get_logger().info(f"Projected Gravity: {self.projected_gravity}")
        self.get_logger().info(f"Current Cmd pose: {self.current_cmd_pose}")
        #self.get_logger().info(f"Receive lidar_scan: {self.lidar_scan}")

        # Creating obs vector (without last action)
        obs = np.concatenate((self.base_vel,
                              self.projected_gravity,
                              self.current_cmd_pose,
                              self.model_in), axis=None)
        
        # Log the obs vector and its size
        #self.get_logger().info(f"Obs vector: {np.array2string(obs, precision=3, separator=', ')}")
        #self.get_logger().info(f"Obs vector size: {obs.size}")

        # Make obs into np array & reshape for the batch size
        obs = obs.astype(np.float32)
        obs = obs.reshape(1, -1)

        # Run inference    always output action 
        ort_inputs = {self.ort_session.get_inputs()[0].name: obs}
        ort_outs = self.ort_session.run(None, ort_inputs)
        #self.nav_actions = ort_outs[0].flatten()


        # Clip actions to ensure they are within the expected range
        # Assuming the model outputs actions in the range [-1, 1]
        raw_actions = ort_outs[0].flatten()
        clipped_actions = np.clip(raw_actions, self.clip_low, self.clip_high)
        
        # Publishing action messages
        
        action_msg = Float32MultiArray()
        #action_msg.data = self.nav_actions.tolist()
        action_msg.data = clipped_actions.tolist()
        self.get_logger().info(f"Publishing actions: {action_msg.data}")
        self.publisher.publish(action_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Go2_RL_Nav_Actions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
