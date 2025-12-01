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
        #go2_base_ang_vel
        self.create_subscription(
            Float32MultiArray,
            'base_ang_vel',
            self.base_ang_vel_callback,
            10)
        self.create_subscription(
            Float32MultiArray,
            'joint_pos_vel',
            self.joint_pos_vel_callback,
            10)
        # lidar 
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'processed_scan',
            self.scan_callback,
            10
        )   
        #   base_lin_vel base_vel   no this attri   
        #   base_ang_vel  
        #   projected_gravity
        #   joint_pos
        #   joint_vel 
        #   base_velocity   
        #   actions  #zuihao no add 
        #   distance 
        #   heading         
        #   angle_diff         
        #   lidar_scan  "lidar_resolution": (30, 30),          
           
        # Initializing variables    
        self.nav_actions = None   
        self.processed_actions = None
          
        #self.base_vel = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.projected_gravity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.current_cmd_pose = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.current_pose = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

        # 2
        self.pose_cmd_defaults = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        #add  
        self.base_ang_vel = np.zeros(3) 
        self.joint_pos_vel = np.zeros(24) #  pos and vel 
        self.joint_pos_init = None  
        self.cmd_vel = np.zeros(3)  
        self.lidar_scan = np.array({1,900}, dtype=np.float32)
        self.distance= np.zeros(1) 
        self.heading= np.zeros(1) 
        self.angle_diff= np.zeros(1)

        # Finding and loading the ONNX model - can be changed to desired model
        share_dir = get_package_share_directory('rl_navigation')
        model_path = os.path.join(share_dir, 'models', 'flat_nav_policy.onnx')
        self.get_logger().info(f"Model path: {model_path}")
        self.load_onnx_model(model_path)

        # Create a timer to generate actions every 20 milliseconds (50 Hz)
        self.timer_period = 0.02  # 20 milliseconds
        self.timer = self.create_timer(
            self.timer_period,  
            self.generate_actions)

    def load_onnx_model(self, model_path):
        self.ort_session = ort.InferenceSession(model_path)
    
    def scan_callback(self, msg: Float32MultiArray):
        # msg.data 是一个归一化后的 [0,1] 数组（长度 = res_x * res_y，例如 900）
        self.lidar_scan = msg.data 
        # 打印前 10 个值
        self.get_logger().info(f'Got scan data (first 10): {self.data[:10]}')

        # base_ang_vel  
    def base_ang_vel_callback(self, msg):
        self.base_ang_vel = np.array(msg.data, dtype=np.float32)
        # joint vel & pos 
    def joint_pos_vel_callback(self, msg):
        self.joint_pos_vel = msg.data
        # The first 12 elements are joint positions and left are joint vels
        self.joint_pos_init = msg.data[:12]
        # Getting initial raw action
        self.init_raw_action = (self.joint_pos_init-self.motor_qs_defaults)/self.scale_factor
    #cmd_vel
    def cmd_vel_callback(self, msg):
        # Converting twist message to array for concatenation
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)
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

        
    def projected_gravity_callback(self, msg):
        self.projected_gravity = np.array(msg.data, dtype=np.float32)

    
    def cmd_pose_callback(self, msg):
        # Calling current pose to update the pose command as approaching the goal
        if not hasattr(self, "initial_cmd_pose"):
            self.initial_cmd_pose = np.array(msg.data, dtype=np.float32) - self.initial_pose

        # Updating command pose based on progress made
        self.current_cmd_pose = self.initial_cmd_pose - self.current_pose

    def generate_actions(self):   
        # Log the current state of the input components
        self.get_logger().info(f"Base angle Velocity: {self.base_ang_vel}")
        self.get_logger().info(f"Projected Gravity: {self.projected_gravity}")
        self.get_logger().info(f"Current Cmd pose: {self.current_cmd_pose}")

        # Creating obs vector (without last action)
        obs = np.concatenate((self.base_ang_vel,self.projected_gravity,
                              self.current_cmd_pose,self.joint_pos_vel,  
                              self.distance,self.heading,self.angle_diff,
                              self.lidar_scan), axis=None)

        # Log the obs vector and its size
        self.get_logger().info(f"Obs vector: {np.array2string(obs, precision=3, separator=', ')}")
        self.get_logger().info(f"Obs vector size: {obs.size}")

        # Make obs into np array & reshape for the batch size
        obs = obs.astype(np.float32).reshape(1, -1)

        # Run inference   should always out put action for navgation    
        ort_inputs = {self.ort_session.get_inputs()[0].name: obs}
        ort_outs = self.ort_session.run(None, ort_inputs)
        self.nav_actions = ort_outs[0].flatten()

        # Publishing action messages
        action_msg = Float32MultiArray()
        action_msg.data = self.nav_actions.tolist()
        self.publisher.publish(action_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Go2_RL_Nav_Actions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
