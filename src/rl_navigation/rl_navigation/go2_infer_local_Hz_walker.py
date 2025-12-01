#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
本地推理版（无 HTTP）
- 订阅 /processed_scan(580 floats) + /odom + /goal_pose
- 直接在进程内加载 actor.onnx（优先）或 actor.ts（TorchScript 备选）并前向
- 控制频率 CTRL_HZ（默认 5 Hz），所有与频率相关参数按“秒”配置后换算为 tick

ENV 重点：
- MODEL_PATH=/path/to/actor.onnx  # 若留空默认 ./actor.onnx 找不到再尝试 ./actor.ts
- CTRL_HZ=5.0
- MIN_EVAL_SEC=0.40        # 开始检查终止条件的最小时长（秒），默认拟合 15 tick@38.2Hz
- BRAKE_HOLD_SEC=0.13      # 刹车保持时长（秒），默认拟合 5 tick@38.2Hz

- SCAN_TOPIC=/processed_scan
- ODOM_TOPIC=/odom
- GATE_TOPIC=/forward_or_stop
- REQUIRE_GOAL=1|0, AUTO_ARM=0|1
- REAL_LIDAR_DISTANCE_CAP=6.0, REAL_THRESHOLD_COLLISION=0.18, REAL_THRESHOLD_GOAL=0.35
- REAL_SPEED_LINEAR_MAX=0.40, REAL_SPEED_ANGULAR_MAX=2.00
- REAL_ARENA_LENGTH=14.2, REAL_ARENA_WIDTH=14.2
- PRINT_INFER=0/1, PRINT_PREP=0/1
"""

import os, copy, math, time
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy, QoSProfile, QoSDurabilityPolicy

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import LaserScan

# ========= 常量与 ENV =========
NUM_SCAN_SAMPLES = 580
REAL_ARENA_LENGTH = float(os.getenv("REAL_ARENA_LENGTH", "14.2"))
REAL_ARENA_WIDTH  = float(os.getenv("REAL_ARENA_WIDTH",  "14.2"))
REAL_LIDAR_DISTANCE_CAP   = float(os.getenv("REAL_LIDAR_DISTANCE_CAP",  "6.0"))
REAL_THRESHOLD_COLLISION  = float(os.getenv("REAL_THRESHOLD_COLLISION", "0.20"))
REAL_THRESHOLD_GOAL       = float(os.getenv("REAL_THRESHOLD_GOAL",      "0.00"))

REAL_SPEED_LINEAR_MAX   = float(os.getenv("REAL_SPEED_LINEAR_MAX",  "0.40"))
REAL_SPEED_ANGULAR_MAX  = float(os.getenv("REAL_SPEED_ANGULAR_MAX", "1.50"))

LINEAR, ANGULAR = 0, 1
ENABLE_BACKWARD = False

CTRL_HZ = float(os.getenv("CTRL_HZ", "5.0"))

MODEL_PATH = None  # 将在 main() 中通过命令行/环境变量/交互式选择设定

# ========= 小工具 =========

def euler_from_quaternion(quat) -> Tuple[float, float, float]:
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    sinr_cosp = 2 * (w*x + y*z); cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w*y - z*x); sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny_cosp = 2 * (w*z + x*y); cosy_cosp = 1 - 2 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

# ========= 推理后端（ONNX 优先 / TorchScript 备选） =========
class LocalModel:
    def __init__(self, logger, model_path: str):
        self.logger = logger
        self.model_path = model_path
        self.kind = None
        self.session = None  # onnxruntime
        self.ts = None       # torchscript
        self._init_backend()

    def _init_backend(self):
        path = self.model_path
        if os.path.isdir(path):
            # 若给的是目录，优先找 actor.onnx，其次 actor.ts
            p1 = os.path.join(path, "actor.onnx")
            p2 = os.path.join(path, "actor.ts")
        else:
            p1, p2 = None, None

        # 1) 尝试 ONNX
        candidate_onnx = path if (isinstance(path, str) and path.endswith('.onnx')) else (p1 if p1 and os.path.isfile(p1) else None)
        if candidate_onnx and os.path.isfile(candidate_onnx):
            try:
                import onnxruntime as ort
                so = ort.SessionOptions()
                so.intra_op_num_threads = 1
                so.inter_op_num_threads = 1
                so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                providers = ["CPUExecutionProvider"]
                self.session = ort.InferenceSession(candidate_onnx, sess_options=so, providers=providers)
                # 名称与维度
                in0 = self.session.get_inputs()[0]
                out0 = self.session.get_outputs()[0]
                self.input_name = in0.name
                self.output_name = out0.name
                # 尝试推断 action_dim
                dummy = np.zeros((1, NUM_SCAN_SAMPLES + 4), dtype=np.float32)
                _ = self.session.run([self.output_name], {self.input_name: dummy})
                self.kind = "onnx"
                self.logger.info(f"[MODEL] ONNX loaded: {candidate_onnx} | in={self.input_name} out={self.output_name}")
                return
            except Exception as e:
                self.logger.warn(f"[MODEL][ONNX] load failed: {e}")

        # 2) 尝试 TorchScript
        candidate_ts = path if (isinstance(path, str) and path.endswith('.ts')) else (p2 if p2 and os.path.isfile(p2) else None)
        if candidate_ts and os.path.isfile(candidate_ts):
            try:
                import torch
                self.ts = torch.jit.load(candidate_ts, map_location='cpu')
                self.ts.eval()
                self.kind = "ts"
                self.logger.info(f"[MODEL] TorchScript loaded: {candidate_ts}")
                return
            except Exception as e:
                self.logger.warn(f"[MODEL][TS] load failed: {e}")

        raise FileNotFoundError(f"找不到可用模型：{path}（支持 actor.onnx 或 actor.ts）")

    def forward(self, state: np.ndarray) -> np.ndarray:
        """ state: (584,) float32 -> action: (>=2,) float32 """
        if self.kind == "onnx":
            out = self.session.run([self.output_name], {self.input_name: state.reshape(1, -1)})[0]
            return out.reshape(-1).astype(np.float32)
        elif self.kind == "ts":
            import torch
            with torch.inference_mode():
                x = torch.from_numpy(state).view(1, -1)
                y = self.ts(x)
                return y.squeeze(0).detach().numpy().astype(np.float32, copy=False)
        else:
            raise RuntimeError("model backend not initialized")

# ========= ROS2 节点 =========
class LocalInferNode(Node):
    def __init__(self, model_path: str):
        super().__init__('go2_local_infer')
        self.get_logger().info("Initializing Local Inference Node (no HTTP)…")

        # 参数/话题
        self.scan_topic = os.getenv("SCAN_TOPIC", "/processed_scan")
        odom_topic = os.getenv("ODOM_TOPIC", "/utlidar/robot_odom")
        self.gate_topic = os.getenv("GATE_TOPIC", "/forward_or_stop")
        self.require_goal = (os.getenv("REQUIRE_GOAL", "1") == "1")
        self.auto_arm = (os.getenv("AUTO_ARM", "0") == "1")

        # 频率 → dt
        self.ctrl_hz = max(0.1, float(os.getenv("CTRL_HZ", "5.0")))
        self.dt = 1.0 / self.ctrl_hz
        self.min_eval_sec   = float(os.getenv("MIN_EVAL_SEC",   "0.40"))   # 默认匹配 15 tick@38.2Hz ≈0.39s
        self.brake_hold_sec = float(os.getenv("BRAKE_HOLD_SEC", "0.13"))   # 默认匹配  5 tick@38.2Hz ≈0.13s

        # QoS
        qos_pub  = QoSProfile(depth=10)
        qos_scan = QoSProfile(depth=10)
        qos_scan.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_scan.durability  = QoSDurabilityPolicy.VOLATILE

        # 订阅/发布
        self.create_subscription(Bool, self.gate_topic, self.gate_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_pub)
        self.create_subscription(Pose, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_simu_callback, 10)  # 可选：仿真
        self.create_subscription(Float32MultiArray, self.scan_topic, self.scan_callback, qos_profile=qos_scan)
        self.actions_pub = self.create_publisher(Float32MultiArray, 'nav_actions', 10)
        # 定时器（按 Hz）
        self.timer = self.create_timer(self.dt, self.control_loop)

        # 运行态
        self.motion_enabled = True
        self.scan_ranges   = [1.0] * NUM_SCAN_SAMPLES
        self.obstacle_distance = float('inf')
        self.have_scan     = False

        self.goal_pose     = Pose()
        self.goal_x        = 0.0
        self.goal_y        = 0.0
        self.have_goal     = self.auto_arm
        self.new_goal      = self.auto_arm

        self.robot_x       = 0.0
        self.robot_y       = 0.0
        self.robot_heading = 0.0
        self.have_odom     = False

        self.robot_x_prev   = 0.0
        self.robot_y_prev   = 0.0
        self.total_distance = 0.0
        self._first_odom    = True

        self.goal_distance = float('inf')
        self.goal_angle    = 0.0

        self.prev_vx, self.prev_omega = 0.0, 0.0
        self.prev_action = [0.0, 0.0]
        self._printed_result = False

        self.armed         = self.auto_arm
        self.done          = False
        self._brake_ticks       = 0

        # ← 关键：把“按秒”配置换算为 tick，避免不同 CTRL_HZ 下行为漂移
        self._brake_ticks_hold = max(1, int(round(self.brake_hold_sec / self.dt)))
        self._min_eval_ticks   = max(1, int(round(self.min_eval_sec   / self.dt)))

        self.UNKNOWN, self.SUCCESS, self.COLLISION_WALL = 0, 1, 2
        self.succeed = self.UNKNOWN

        self.last_scan_recv_ns = 0
        self.last_odom_recv_ns = 0
        self.local_step = 0

        # 打印开关
        self.print_infer = (os.getenv("PRINT_INFER", "0") == "1")
        self.print_prep  = (os.getenv("PRINT_PREP",  "0") == "1")

        # 加载模型（ONNX 优先）
        try:
            self.model = LocalModel(self.get_logger(), model_path)
        except Exception as e:
            raise SystemExit(f"[FATAL] 模型加载失败: {e}")

        self.get_logger().info(f"[CFG] scan_topic={self.scan_topic} odom_topic={odom_topic} "
                               f"require_goal={self.require_goal} auto_arm={self.auto_arm} "
                               f"CTRL_HZ={self.ctrl_hz} dt={self.dt:.3f}s "
                               f"MIN_EVAL_SEC={self.min_eval_sec}s BRAKE_HOLD_SEC={self.brake_hold_sec}s "
                               f"→ _min_eval_ticks={self._min_eval_ticks}, _brake_ticks_hold={self._brake_ticks_hold}")

    # ---------- 订阅回调 ----------
    def gate_cb(self, msg: Bool):
        self.motion_enabled = bool(msg.data)
        self.get_logger().info(f"[GATE] motion_enabled={self.motion_enabled}")
        if not self.motion_enabled:
            self.cmd_vel_pub.publish(Twist())
            self._publish_zero()

    def scan_simu_callback(self, msg: LaserScan):
        if not msg.ranges:
            return
        n = min(NUM_SCAN_SAMPLES, len(msg.ranges))
        min_norm = 1.0
        for i in range(n):
            v = float(msg.ranges[i]) / REAL_LIDAR_DISTANCE_CAP
            v = 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)
            if i < NUM_SCAN_SAMPLES:
                self.scan_ranges[i] = v
            if v < min_norm:
                min_norm = v
        self.obstacle_distance = min_norm * REAL_LIDAR_DISTANCE_CAP
        self.have_scan = True
        self.last_scan_recv_ns = time.time_ns()

    def scan_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        if not data:
            return
        if len(data) != NUM_SCAN_SAMPLES:
            self.get_logger().warn(f"[SCAN] got {len(data)} but expected {NUM_SCAN_SAMPLES}; will pad/clip")
        self.scan_ranges = data[:NUM_SCAN_SAMPLES]
        if len(self.scan_ranges) < NUM_SCAN_SAMPLES:
            self.scan_ranges += [1.0] * (NUM_SCAN_SAMPLES - len(self.scan_ranges))
        self.obstacle_distance = min(self.scan_ranges) * REAL_LIDAR_DISTANCE_CAP
        self.last_scan_recv_ns = time.time_ns()
        self.have_scan = True

    def goal_callback(self, msg: Pose):
        self.goal_pose = msg
        self.goal_x = msg.position.x
        self.goal_y = msg.position.y
        self.new_goal  = True
        self.have_goal = True
        self.armed     = True
        self.done      = False
        self.succeed   = self.UNKNOWN
        self._brake_ticks = 0

        self.total_distance = 0.0
        self._printed_result = False
        self.local_step = 0
        if self.have_odom:
            self.robot_x_prev, self.robot_y_prev = self.robot_x, self.robot_y
            self._first_odom = False
        else:
            self._first_odom = True

        self.get_logger().info(f"[GOAL] x={self.goal_x:.3f}, y={self.goal_y:.3f}")

    def odom_callback(self, msg: Odometry):
        self.have_odom = True
        self.last_odom_recv_ns = time.time_ns()
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        _, _, self.robot_heading = euler_from_quaternion(msg.pose.pose.orientation)

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        self.goal_distance = math.hypot(dx, dy)
        heading_to_goal = math.atan2(dy, dx)
        goal_angle = heading_to_goal - self.robot_heading
        while goal_angle > math.pi:  goal_angle -= 2*math.pi
        while goal_angle < -math.pi: goal_angle += 2*math.pi
        self.goal_angle = goal_angle

        if self._first_odom:
            self.robot_x_prev, self.robot_y_prev = self.robot_x, self.robot_y
            self._first_odom = False
        else:
            step = math.hypot(self.robot_x - self.robot_x_prev, self.robot_y - self.robot_y_prev)
            if math.isfinite(step) and step < 2.0:
                self.total_distance += step
            self.robot_x_prev, self.robot_y_prev = self.robot_x, self.robot_y

    # ---------- 控制回路 ----------
    def _publish_zero(self, repeat: int = 1):
        """同时给 /cmd_vel 和 nav_actions 发零速"""
        t = Twist()
        a = Float32MultiArray(); a.data = [0.0, 0.0, 0.0]  # [vx, vy, vyaw]
        for _ in range(repeat):
            self.cmd_vel_pub.publish(t)
            if hasattr(self, "actions_pub"):
                self.actions_pub.publish(a)
    # 
    def _publish_stop(self, reason: str):
        self._publish_zero(repeat=3)         # ← 原来循环发 Twist 的地方用这个
        self._brake_ticks = self._brake_ticks_hold
        self.done = True
        if not getattr(self, "_printed_result", False):
            human = "GOAL REACHED" if reason == "goal_reached" else ("COLLISION" if reason == "collision" else f"STOP: {reason}")
            self.get_logger().info(
                f"[STOP] {human} | traveled={self.total_distance:.2f} m | d_goal={self.goal_distance:.2f} m | d_obs_min={self.obstacle_distance:.2f} m"
            )
            self._printed_result = True

    def _check_stop_conditions(self) -> bool:
        if self.done:
            return False
        if self.goal_distance < REAL_THRESHOLD_GOAL:
            self._publish_stop("goal_reached"); return True
        if self.obstacle_distance < REAL_THRESHOLD_COLLISION:
            self._publish_stop("collision"); return True
        return False

    def control_loop(self):
        if self.done:
            self._publish_zero(); return
        if self._brake_ticks > 0:
            self._brake_ticks -= 1
            self._publish_zero(); return
        if not self.have_odom or not self.have_scan or (self.require_goal and not self.new_goal) or not self.armed:
            self._publish_zero(); return

        self.local_step += 1
        if self.local_step >= self._min_eval_ticks and self._check_stop_conditions():
            return
        if not self.motion_enabled:
            self._publish_zero(); return

        # 构造 584 维状态：[580 扫描] + [d_goal_norm, angle/pi, prev_vx_norm, prev_wz_norm]
        t_p0 = time.perf_counter_ns()
        state = copy.deepcopy(self.scan_ranges)
        max_dist = math.hypot(REAL_ARENA_LENGTH, REAL_ARENA_WIDTH)
        state.append(float(np.clip(self.goal_distance / max_dist, 0.0, 1.0)))
        state.append(float(self.goal_angle) / math.pi)
        state.append(float(self.prev_vx / REAL_SPEED_LINEAR_MAX))
        state.append(float(self.prev_omega / REAL_SPEED_ANGULAR_MAX))
        prep_ms = (time.perf_counter_ns() - t_p0) / 1e6
        if self.print_prep:
            self.get_logger().info(f"[PREP] lidar_preprocess_ms={prep_ms:.3f}")

        # 本地推理
        t0 = time.perf_counter_ns()
        action = self.model.forward(np.asarray(state, dtype=np.float32))
        infer_ms = (time.perf_counter_ns() - t0) / 1e6
        if self.print_infer:
            self.get_logger().info(f"[INFER] infer_ms={infer_ms:.2f} action[:2]={action[:2].tolist()}")

        # 动作映射
        if ENABLE_BACKWARD:
            vx = float(action[LINEAR]) * REAL_SPEED_LINEAR_MAX
        else:
            vx = float((action[LINEAR] + 1.0) * 0.5) * REAL_SPEED_LINEAR_MAX
        wz = float(action[ANGULAR]) * REAL_SPEED_ANGULAR_MAX
        self.prev_vx, self.prev_omega = vx, wz

        twist = Twist(); twist.linear.x = vx; twist.angular.z = wz
        self.cmd_vel_pub.publish(twist)

        # 新增（把 vx 和 omega 变成数组发出去）：
        action_msg = Float32MultiArray()
        action_msg.data = [float(vx), 0.0, float(wz)]   # [vx, vy(=0), vyaw]
        self.actions_pub.publish(action_msg)

# 
def _discover_models(search_dir: str):
    exts = ('.onnx')
    names = []
    try:
        for f in os.listdir(search_dir):
            if f.lower().endswith(exts):
                names.append(os.path.join(search_dir, f))
    except FileNotFoundError:
        return []
    # 先按名称排序；也可以改成按 mtime 排序
    names.sort()
    return names


def _interactive_pick(models: list) -> str:
    print("可用模型如下（输入序号选择，直接回车选 [0]）：")
    for i, p in enumerate(models):
        print(f"  [{i}] {p}")
    try:
        s = input("选择模型编号 [0]: ").strip()
    except EOFError:
        s = ''
    idx = 0
    if s.isdigit():
        idx = max(0, min(int(s), len(models)-1))
    print(f"=> 使用模型: {models[idx]}")
    return models[idx]


def main(args=None):
    import argparse
    parser = argparse.ArgumentParser(description="Go2 本地推理（ONNX/TorchScript）")
    parser.add_argument('-m', '--model', default=os.getenv('MODEL_PATH', ''), help='模型路径或包含模型的目录（优先 .onnx，次选 .ts）')
    parser.add_argument('--auto', action='store_true', help='非交互模式：若给的是目录，则自动选择排序后的第一个模型')
    parsed, unknown = parser.parse_known_args()

    # 解析模型路径
    model_path = parsed.model.strip()
    if not model_path:
        model_path = os.path.abspath('./')  # 默认在当前目录里找

    # 如果给的是目录，则枚举其中的 .onnx/.ts
    if os.path.isdir(model_path):
        models = _discover_models(model_path)
        if not models:
            raise SystemExit(f"在目录中未发现 .onnx 或 .ts 模型：{model_path}")
        if parsed.auto or not os.isatty(0):
            chosen = models[0]
            print(f"[AUTO] 选择 {chosen}")
        else:
            chosen = _interactive_pick(models)
        model_path = chosen

    # 限制数值库线程（稳定延迟）
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

    rclpy.init(args=args)
    node = LocalInferNode(model_path)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
