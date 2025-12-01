#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from unitree_api.msg import Request
import numpy as np
import onnxruntime as ort
import os
from ament_index_python.packages import get_package_share_directory

class Go2_FullStack_Node(Node):
    def __init__(self):
        super().__init__("go2_fullstack")

        # --- Publisher & Subscribers ---
        self.cmd_pub = self.create_publisher(Request, '/api/sport/request', 10)
        # 高层 OBS 订阅
        self.create_subscription(SportModeState, 'sportmodestate', self.state_callback, 10)
        self.create_subscription(Float32MultiArray, 'projected_gravity', self.proj_grav_callback, 10)
        self.create_subscription(Float32MultiArray, 'cmd_pose', self.cmd_pose_callback, 10)
        # 地层 OBS 订阅（关节状态）
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        # （如需接触力、扫描等，也在此添加订阅）

        # --- Load High-Level ONNX Model ---
        hl_share = get_package_share_directory('rl_navigation')
        hl_model = os.path.join(hl_share, 'models', 'flat_nav_policy.onnx')
        self.hl_sess = ort.InferenceSession(hl_model)
        self.hl_in_name = self.hl_sess.get_inputs()[0].name

        # --- Load Low-Level ONNX Model ---
        ll_share = get_package_share_directory('rl_navigation')
        ll_model = os.path.join(ll_share, 'models', 'legged_loco_policy.onnx')
        self.ll_sess = ort.InferenceSession(ll_model)
        self.ll_in_name = self.ll_sess.get_inputs()[0].name
        self.ll_out_name = self.ll_sess.get_outputs()[0].name

        # --- Initialize state holders ---
        self.base_vel = np.zeros(3, dtype=np.float32)
        self.proj_grav = np.zeros(3, dtype=np.float32)
        self.cmd_pose = np.zeros(4, dtype=np.float32)    # x,y,heading, maybe velocity?
        self.joint_pos = None
        self.joint_vel = None

        # High-level initial pose
        self.initial_xyz = None
        self.initial_heading = None
        self.initial_cmd_pose = None

        # Run full-stack at 50Hz
        self.create_timer(0.02, self.step_fullstack)

    # --- High-Level Callbacks ---
    def state_callback(self, msg: SportModeState):
        # 初次拿到基准
        if self.initial_xyz is None:
            self.initial_xyz = np.array(msg.position, dtype=np.float32)
            self.initial_heading = msg.imu_state.rpy[2]
        # 当前 pose
        xyz = np.array(msg.position, dtype=np.float32) - self.initial_xyz
        heading = msg.imu_state.rpy[2] - self.initial_heading
        self.current_pose = np.concatenate((xyz, [heading]), axis=0)
        # 线速度
        self.base_vel = np.array(msg.velocity, dtype=np.float32)

    def proj_grav_callback(self, msg: Float32MultiArray):
        self.proj_grav = np.array(msg.data, dtype=np.float32)

    def cmd_pose_callback(self, msg: Float32MultiArray):
        target = np.array(msg.data, dtype=np.float32)
        if self.initial_cmd_pose is None:
            # 以第一次命令为起始
            self.initial_cmd_pose = target.copy()
        # 距离当前 pose 的向量
        self.cmd_pose = self.initial_cmd_pose - self.current_pose

    # --- Low-Level Callback ---
    def joint_state_callback(self, msg: JointState):
        self.joint_pos = np.array(msg.position, dtype=np.float32)
        self.joint_vel = np.array(msg.velocity, dtype=np.float32)

    # --- Full-Stack Inference ---
    def step_fullstack(self):
        # 1) 构造高层 obs 并 run high-level ONNX
        hl_obs = np.concatenate((self.base_vel, self.proj_grav, self.cmd_pose), axis=0).astype(np.float32)
        hl_in = hl_obs.reshape(1, -1)
        hl_action = self.hl_sess.run(None, {self.hl_in_name: hl_in})[0].flatten()

        # 2) 确保低层观测就绪
        if self.joint_pos is None or self.joint_vel is None:
            return

        # 3) 构造低层 obs：把高层输出也当输入一部分
        ll_obs = np.concatenate((
            hl_action,            # 高层命令（如速度/位姿命令）
            self.joint_pos,       # 关节位置
            self.joint_vel,       # 关节速度
            # … 如果还有 contact、scan、last_action 等，也 append 进 ll_obs …
        ), axis=0).astype(np.float32)
        ll_in = ll_obs.reshape(1, -1)
        ll_out = self.ll_sess.run([self.ll_out_name], {self.ll_in_name: ll_in})[0].flatten()

        # 4) 发布给机器人：如果 low-level 输出是关节命令
        req = Request()
        req.jointcmd = ll_out.tolist()
        self.cmd_pub.publish(req)

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Go2_FullStack_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
