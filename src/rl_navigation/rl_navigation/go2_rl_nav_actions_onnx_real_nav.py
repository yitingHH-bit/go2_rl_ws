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
import torch, math

def _wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

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
        self.dist_xy= np.zeros(1)  #dist_xy
        self.heading= np.zeros(1) 
        self.heading_diff_deg= np.zeros(1) 
        self.actions_null = np.zeros(3) 
        #add clip 
        
        clips = [(-0.5, 0.5), (-0.5, 0.5), (-1.0, 1.0)]
        self.clip_low, self.clip_high = map(lambda x: np.array(x, dtype=np.float32), zip(*clips))

        #self.base_vel = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.projected_gravity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.current_cmd_pose = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.current_pose = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
                #add  
        self.base_ang_vel = np.zeros(3) 
        self.lidar_scan = np.zeros((300, 3), dtype=np.float32)

        # 
        self.pose_cmd_defaults = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        # Finding and loading the ONNX model - can be changed to desired model
        share_dir = get_package_share_directory('rl_navigation')
        model_path = os.path.join(share_dir, 'models', 'policy_906_good.onnx')
        self.get_logger().info(f"Model path: {model_path}")
        self.load_onnx_model(model_path)

        # Create a timer to generate actions every 20 milliseconds (50 Hz)
        self.timer_period = 0.02  # 20 milliseconds
        self.timer = self.create_timer(
            self.timer_period,  
            self.generate_actions)  

    def load_onnx_model(self, model_path):
        self.ort_session = ort.InferenceSession(model_path)
        # lidar 
    def scan_callback(self, msg: Float32MultiArray):
        # msg.data 是一个归一化后的 [0,1] 数组（长度 = res_x * res_y，例如 900）
        self.lidar_scan = msg.data
        #arr = np.array(self.lidar_scan, dtype=np.float32)
        #= arr.reshape(36, 4)  
        # 打印前 10 个值
        #self.get_logger().info(f'Got scan data (first 10): {self.lidar_scan[:10]}')
    
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
        #self.base_vel = np.array(msg.velocity, dtype=np.float32)

    def projected_gravity_callback(self, msg):
        self.projected_gravity = np.array(msg.data, dtype=np.float32)
    


    def cmd_pose_callback(self, msg):
        # msg.data = [x_tgt, y_tgt, z_tgt, yaw_tgt] —— 世界系绝对量（弧度）
        x_tgt, y_tgt, z_tgt, yaw_tgt = map(float, msg.data)

        # 当前位姿（世界系，弧度）—— 你在别处维护：self.current_pose = [x_cur, y_cur, z_cur, yaw_cur]
        try:
            x_cur, y_cur, z_cur, yaw_cur = map(float, self.current_pose)
        except Exception:
            # 还没拿到当前位姿就先置零，避免报错
            self.current_cmd_pose = np.zeros(4, dtype=np.float32)
            self.nav_obs = np.zeros(3, dtype=np.float32)
            self.get_logger().warn("current_pose not ready; set current_cmd_pose/nav_obs = 0")
            return

        # ---------- 世界系误差（target - current）----------
        dx_w = x_tgt - x_cur
        dy_w = y_tgt - y_cur
        dz_w = z_tgt - z_cur
        dyaw = _wrap_to_pi(yaw_tgt - yaw_cur)  # 航向误差（弧度）

        # ---------- 旋转到机体系（base frame）：x前、y左 ----------
        # R(-yaw_cur) · [dx_w, dy_w]
        cy = math.cos(-yaw_cur); sy = math.sin(-yaw_cur)
        dx_b =  dx_w * cy - dy_w * sy
        dy_b =  dx_w * sy + dy_w * cy
        dz_b =  dz_w  # 忽略滚俯时，z 不变；训练用的是2D距离

        # ========== 1) 你必须保留的变量 ==========
        # 让 current_cmd_pose 表示“机体系误差”+ 航向误差（与训练里 command_manager 的语义一致）
        self.current_cmd_pose = np.array([dx_b, dy_b, dz_b, dyaw], dtype=np.float32)

        # ========== 2) 构造与训练一致的三维观测 ==========
        # distance_to_target_euclidean —— 2D（机体系）
        dist_xy = math.hypot(dx_b, dy_b)                # 米
        # angle_to_target_observation —— 机体系方位角（弧度 ∈ [-π, π]）
        heading_body = math.atan2(dy_b, dx_b)
        # angle_diff —— 航向误差（弧度 ∈ [-π, π]）
        yaw_err = dyaw

        # 按训练时的 scale（保持一模一样）
        dist_scaled = 0.1 * dist_xy
        heading_scaled = heading_body / math.pi
        yaw_err_scaled = yaw_err / math.pi

        # 喂给网络的向量（顺序：distance, heading, angle_diff）
        self.nav_obs = np.array([dist_scaled, heading_scaled, yaw_err_scaled], dtype=np.float32)

        # —— 可选：保留未缩放值，方便你别处用（如果不需要可删）
        self.dist_xy = float(dist_xy)
        self.heading_body_rad = float(heading_body)
        self.heading_diff_rad = float(yaw_err)

        # 调试打印
        self.get_logger().info(
            "CmdPose_bf dx={:.3f}, dy={:.3f}, dz={:.3f}, dyaw={:.3f} rad | "
            "obs=[dist*0.1={:.3f}, heading/pi={:.3f}, yaw_err/pi={:.3f}]".format(
                dx_b, dy_b, dz_b, dyaw, dist_scaled, heading_scaled, yaw_err_scaled
            )
        )

 
    # def cmd_pose_callback(self, msg):
        # msg.data = [x_tgt, y_tgt, z_tgt, yaw_tgt]  —— 世界系绝对量（弧度）
        # x_tgt, y_tgt, z_tgt, yaw_tgt = map(float, msg.data)
        # x_cur, y_cur, z_cur, yaw_cur = map(float, self.current_pose)  # 世界系绝对量（弧度）

        # # 1) 误差（世界系）
        # dx = x_tgt - x_cur
        # dy = y_tgt - y_cur
        # dz = z_tgt - z_cur
        # dyaw = _wrap_to_pi(yaw_tgt - yaw_cur)

        # # 必须保留：供你其他地方使用
        # self.current_cmd_pose = np.array([dx, dy, dz, dyaw], dtype=np.float32)

        # # 2) 根据训练观测项构造网络输入（与训练完全对齐）
        # # distance_to_target_euclidean —— 按你训练时是2D还是3D二选一：
        # dist_xy = math.hypot(dx, dy)             # 如果训练是2D距离 → 用这个
        # # dist_xyz = math.sqrt(dx*dx + dy*dy + dz*dz)  # 如果训练是3D距离 → 用这个替换

        # # angle_to_target_observation —— 机体系方位角：
        # # 世界 dx,dy 旋转到机体系：R(-yaw_cur)·[dx,dy]
        # cos_y = math.cos(-yaw_cur); sin_y = math.sin(-yaw_cur)
        # dx_body =  dx * cos_y - dy * sin_y
        # dy_body =  dx * sin_y + dy * cos_y
        # heading_body = math.atan2(dy_body, dx_body)     # 弧度 ∈ [-π, π]

        # # angle_diff —— 航向误差
        # yaw_err = dyaw  # 已 wrap

        # # 3) 按训练时的 scale 做缩放
        # dist_scaled        = 0.1 * dist_xy           # 或用 dist_xyz，看你训练用的那个
        # heading_body_scaled = heading_body / math.pi
        # yaw_err_scaled      = yaw_err / math.pi

        # # 4) 组织成网络需要的顺序（你说要传：距离、heading、heading_diff）
        # self.nav_obs = np.array(
        #     [dist_scaled, heading_body_scaled, yaw_err_scaled],
        #     dtype=np.float32
        # )

        # # 调试输出
        # self.get_logger().info(
        #     f"CmdPose dx,dy,dz,dyaw(rad)={self.current_cmd_pose}, "
        #     f"obs=[{dist_scaled:.3f}, {heading_body_scaled:.3f}, {yaw_err_scaled:.3f}]"
        # )


    def generate_actions(self):
        # Log the current state of the input components 
        self.get_logger().info(f"Base Velocity: {self.base_ang_vel}")
        self.get_logger().info(f"Projected Gravity: {self.projected_gravity}")
        self.get_logger().info(f"Current Cmd pose: {self.current_cmd_pose}")
        #self.get_logger().info(f"Receive lidar_scan: {self.lidar_scan}")

        # Creating obs vector (without last action)
        # obs = np.concatenate((self.base_ang_vel,self.projected_gravity,                             
        #                       self.actions_null,
        #                       self.dist_xy,self.heading,self.heading_diff_deg,
        #                       self.lidar_scan,
        #                       ), axis=None)
        
        obs = np.concatenate((self.actions_null,
                              self.dist_xy,self.heading,self.heading_diff_deg,
                              self.lidar_scan,
                              #self.base_ang_vel,self.projected_gravity,
                              ), axis=None) 
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
