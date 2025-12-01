#!/bin/bash


#	ssh -X unitree@192.168.123.18
#	ssh -X unitree@192.168.50.19   #

#	cd /unitree/module/graph_pid_ws  && ./0_unitree_slam.sh 
# 	cd /unitree/lib/unitree_slam/build && ./demo_xt16 eth0 


source install/setup.bash && ros2 launch pointcloud_to_laserscan  pointcloud_to_scan_launch.py	
source install/setup.bash && ros2 launch pointcloud_to_laserscan  pointcloud_to_scan_launch_580.py

case1:

source install/setup.bash && ros2 run rl_navigation  go2_infer_local

source install/setup.bash && ros2 run rl_navigation  go2_infer_local_Hz

source install/setup.bash && ros2 run rl_deploy_nav go2_rl_nav

or
source install/setup.bash && ros2 run rl_navigation  go2_infer_local_1

source install/setup.bash && ros2 run rl_navigation  go2_infer_local_Hz_1

python3 Go2CmdVelNode.py
python3 cmd_vel_forwarder.py 

send_position.sh 2 0 

case 2:
 source install/setup.bash && ros2 run rl_navigation go2_pose_command --ros-args -p x_cmd:=7.0 -p y_cmd:=0.00 -p heading_cmd:=0.00
 
source install/setup.bash && ros2 run unitree_ros2_python go2_projected_gravity
source install/setup.bash && ros2 run rl_navigation go2_rl_nav_actions_rough_onnx_9000

source install/setup.bash && ros2 run rl_deploy_nav go2_rl_nav
 


























	
# 等待时间（秒）
WAIT_TIME=8			
#gnome-terminal -- bash -c "./build.sh ; exec bash"
gnome-terminal -- bash -c "colcon build --merge-install --symlink-install; exec bash"

sleep $WAIT_TIME
gnome-terminal -- bash -c "colcon build --merge-install --symlink-install; exec bash"

gnome-terminal -- bash -c "source install/setup.bash &&ros2 launch rl_sar gazebo.launch.py rname:=go2; exec bash"
sleep $WAIT_TIME
gnome-terminal -- bash -c "source install/setup.bash && ros2 launch pointcloud_to_laserscan pointcloud_filter.launch.py; exec bash"
#	ssh -X unitree@192.168.123.18
#	ssh -X unitree@192.168.50.19   #

#	cd /unitree/module/graph_pid_ws  && ./0_unitree_slam.sh 
# 	cd /unitree/lib/unitree_slam/build && ./demo_xt16 eth0 

cd student_projects/1/go2_rl_ws


conda deactivate	

cd student_projects/1/go2_rl_ws

iperf3
iperf3
iperf3



source install/setup.bash
ros2 run calibrate_imu calibrate_imu

./system_real_robot_with_route_planner.sh

############################################################################3

你这个“切到 ONNX 就报错”的两段日志其实在说两件事：

（黄）Initializer 警告
Initializer ... appears in graph inputs ⇒ 你的 onnx 里把一些权重放进了“图输入”。功能不影响，只是少了优化（常量折叠）。可选修复：

重新导出时关掉：torch.onnx.export(..., keep_initializers_as_inputs=False)

或用 ORT 脚本清理：onnxruntime/tools/python/remove_initializer_from_input.py

（红）CUDA EP 加载失败
Failed to load libonnxruntime_providers_cuda.so: libcudnn.so.9 not found
并且提示：需要 cuDNN 9 + CUDA 12。你当前环境没有（或不在 LD_LIBRARY_PATH），所以 CUDAExecutionProvider 初始化失败。不过你后面看到了：

重新导出 onnx 时：keep_initializers_as_inputs=False

###########################################
source install/setup.bash &&   ros2 run pointcloud_to_laserscan pointcloud_to_pointnet \
  --ros-args \
  -p input_topic:=/notGround_pointCloud \
  -p output_topic:=/pointnet_cloud \
  -p num_points:=384 \
  -p min_distance:=0.05 -p max_distance:=5.0 \
  -p min_z:=0.47 -p max_z:=0.55 \
  -p z_offset:=0.45
wosdasdasd

cd student_projects/1/go2_rl_ws

 source install/setup.bash &&  ros2 launch pointcloud_to_laserscan pointnet.launch.py
 
#gnome-terminal -- bash -c "source install/setup.bash && ros2 run rl_sar rl_sim; exec bash"		
gnome-terminal -- bash -c "source install/setup.bash && ros2 run rl_sar rl_sim; exec bash"  #_lidar	
 
#wget https://nvidia.box.com/shared/static/w4r7ta0gpb9z8t2mlkcb7kg2cnm28tff.whl -O torch-2.0.0-cp38-cp38-linux_aarch64.whl
 
 


 source install/setup.bash &&  ros2 run rl_navigation go2_rl_nav_actions_onnx_real_rough_720   --ros-args -p y_sign:=1 -p omega_sign:=1 -p swap_xy:=false


 
	
  
  
 
  source install/setup.bash && ros2 launch pointcloud_to_laserscan  pointnet.launch.py	
 

 
 source install/setup.bash && ros2 run unitree_ros2_python go2_projected_gravity
 
 source install/setup.bash && ros2 run rl_navigation go2_rl_nav_actions_rough_onnx     ##### 6000
 
source install/setup.bash && ros2 run rl_navigation  go2_rl_nav_actions_onnx_real_rough_720
 	 
 source install/setup.bash && ros2 run rl_navigation go2_rl_flat_actions_onnx    #
 
  source install/setup.bash && ros2 run rl_navigation go2_rl_nav_actions_onnx_real_rough_new  #
  source install/setup.bash && ros2 run rl_navigation go2_rl_nav_actions_rough_onnx_9000
 
 source install/setup.bash && ros2 launch pointcloud_to_laserscan pointcloud_to_scan_launch.py
 
 source install/setup.bash && ros2 run rl_deploy_nav go2_rl_nav	
 
 xruntime.capi.onnxruntime_pybind11_state.NoSuchFile: [ONNXRuntimeError] : 3 : NO_SUCHFILE : Load model from /home/unitree/student_projects/1/go2_rl_ws/install/rl_navigation/share/rl_navigation/models/policy_6000_tough.onnx failed:Load model /home/unitree/student_projects/1/go2_rl_ws/install/rl_navigation/share/rl_navigation/models/policy_6000_tough.onnx failed. File doesn't exist







  source install/setup.bash &&  ros2 launch pointcloud_to_laserscan pointnet.launch.py

source install/setup.bash
ros2 run pointcloud_to_laserscan pointcloud_to_pointnet_node \
  --ros-args \
  -p target_frame:=base \
  -p num_points:=384 \
  -p max_distance:=5.0 \
  -p ground_z_min:=-0.45 \
  -p channels:=xyzm \
  -p vertical_bins:=4 \
  -p ring_field:=ring \
  -p input_topic:=/hesai/points \
  -p output_topic:=/lidar_pointnet

source install/setup.bash && ros2 run pointcloud_to_laserscan pointcloud_to_pointnet_node \
  --ros-args \
  -p input_topic:=/notGround_pointCloud \
  -p output_topic:=/lidar_pointnet \
  -p num_points:=384 \
  -p max_distance:=5.0 \
  -p channels:=xyzm



python3 - <<'PY'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class Count5m(Node):
    def __init__(self, topic='/notGround_pointCloud'):
        super().__init__('count_5m')
        self.sub = self.create_subscription(PointCloud2, topic, self.cb, 10)

    def cb(self, msg: PointCloud2):
        total = 0
        within = 0
        r2_max = 25.0  # 5m 的平方
        for x, y in point_cloud2.read_points(msg, field_names=('x','y'), skip_nans=True):
            total += 1
            if x*x + y*y <= r2_max:
                within += 1
        self.get_logger().info(f"total={total}, within_5m={within}")
        rclpy.shutdown()

rclpy.init()
rclpy.spin(Count5m('/notGround_pointCloud'))
PY


     sensor: xt16                                # lidar sensor type, either 'velodyne' or 'ouster'   添加 ‘robosense’‘xt16’‘mid360’‘helios32’‘xt32’
      lidarYsn: "AW3HFG53903HFI51"                #"AW3HFG53903HFI51"  AW36FI539D36FG51
      N_SCAN: 16                                  # number of lidar channel (i.e., 16, 32, 64, 128)
      Horizon_SCAN: 2000                          # lidar horizontal resolution (Velodyne:1800, Ouster:512,1024,2048, xt16:2000, mid360:5000, helios32:1800, xt32:2000/10Hz 1000/20Hz)
      timeField: "time"                           # point timestamp field, Velodyne - "time", Ouster - "t"
      downsampleRate: 1                         # default: 1. Downsample your data if too many points. i.e., 16 = 64 / 4, 16 = 16 / 1 
      lidarMinRange: 0.2                           #  default: 1.      add by zzr 2220728
      lidarMaxRange: 20.0
      inversion: false                            # whether lidar upside down
      lidarPose: [0.0, 0.0, 0.0, 0.0, 13.0, 0.0]                   # xyzrpy m m m deg deg deg

      #Point cloud removal around the back of dog's body
      removeWidth: 0.2 # Half the actual width
      removeHeight: 0.6

      # IMU Settings
      imuAccNoise: 3.9939570888238808e-03
      imuGyrNoise: 1.5636343949698187e-03
      imuAccBiasN: 6.4356659353532566e-05
      imuGyrBiasN: 3.5640318696367613e-05
      imuGravity: 9.80511
      imuRPYWeight: 0.01
      
      
      

cd /home/unitree/student_projects/1/go2_rl_ws/src/rl_navigation/rl_navigation
pip install fastapi uvicorn

nano go2_rl_nav_actions_onnx_real_rough.py  

policy_6000_togh3


 source install/setup.bash && ros2 launch  yolov8_ros2 distance_yolo.launch.py


source install/setup.bash && ros2 run rl_navigation  go2_rl_nav_onnx_real_rough_record


 
unitree_command
/up_state_feedback
/uslam/client_command
/uslam/cloud_map
/uslam/frontend/cloud_world_ds
/uslam/frontend/odom
/uslam/localization/cloud_world
/uslam/localization/odom
/uslam/navigation/global_path
/uslam/server_log
/utlidar/client_cmd
/utlidar/cloud
/utlidar/cloud_base
/utlidar/cloud_deskewed
/utlidar/grid_map
/utlidar/height_map
/utlidar/height_map_array
/utlidar/imu
/utlidar/lidar_state
/utlidar/mapping_cmd
/utlidar/range_info
/utlidar/range_map
/utlidar/robot_odom
/utlidar/robot_pose
/utlidar/server_log
/utlidar/switch
/utlidar/voxel_map
/utlidar/voxel_map_compress


 
 
要部署，请确保四足动物处于运动模式或 AI 模式。姿势命令应在世界坐标系中发送。确保 z 命令约为 0.35 [米]。

打开终端，输入unitree_ros和go2_rl_ws，并创建一个姿势命令：
source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh &&
source ~/workspaces/go2_rl_ws/install/setup.sh &&
cd ~/workspaces/go2_rl_ws &&
ros2 run rl_navigation go2_pose_command --ros-args -p x_cmd:=0.1 -p y_cmd:=0.00 -p heading_cmd:=0.00
打开一个新的终端，输入unitree_ros和go2_rl_ws，并运行投影重力发布器：
source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh &&
source ~/workspaces/go2_rl_ws/install/setup.sh &&
cd ~/workspaces/go2_rl_ws &&
ros2 run unitree_ros2_python go2_projected_gravity
打开新的终端，source unitre_ros 和 go2_rl_ws，运行导航动作推理：
source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh &&
source ~/workspaces/go2_rl_ws/install/setup.sh &&
cd ~/workspaces/go2_rl_ws &&
ros2 run rl_navigation go2_rl_nav_actions_onnx
打开一个新的终端，source unitre_ros 和 go2_rl_ws，并运行导航命令：
source ~/workspaces/go2_rl_ws/src/unitree_ros2/setup.sh &&
source ~/workspaces/go2_rl_ws/install/setup.sh &&
cd ~/workspaces/go2_rl_ws &&
ros2 run rl_deploy_nav go2_rl_nav


#  source install/setup.bash && ros2 launch sport_control sport_control_launch.py  
#source install/setup.bash &&ros2 run  go2_cmd_processor  go2_service_client
# source install/setup.bash && ros2 run  go2_cmd_processor  keyboard_teleop 
# source install/setup.bash   &&   ./system_real_robot_with_route_planner.sh
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
#source install/setup.bash ./system_real_robot.sh
#source install/setup.bash && ros2 launch go2_config gazebo_velodyne.launch.py rviz:=false
#python3 -m pip install --user onnxruntime-gpu				
#source install/setup.bash && ros2 launch go2_config gazebo_velodyne.launch.py rviz:=false
#source install/setup.bash && ros2 launch go2_config gazebo_velodyne.launch.py rviz:=false
# source install/setup.bash && ros2 launch slam_toolbox online_async_launch.py 
 
#ros2 launch slam_toolbox online_async_launch.py
#gnome-terminal -- bash -c "source install/setup.bash && ros2 run go2_drl1 dijkstra ; exec bash" 
#	ssh -X unitree@192.168.123.18	
#	cd /unitree/module/graph_pid_ws  && ./0_unitree_slam.sh 
# 	cd /unitree/lib/unitree_slam/build && ./demo_xt16 eth0  
#pp > Device > Data > Robot, locate the motor data, and take a screenshot to share for further diagnosis.  
#ssh -X unitree@192.168.123.18 
#ros2 run nav2_map_server map_saver_cli -f ~/your_map	
#rsync -avz unitree@192.168.123.18:/unitree/module/graph_pid_ws/config_files/lio_sam_config/maps/pcd/default/GlobalMap.pcd  ~/Downloads/maps/
#pip3 install matplotlib pandas pyqtgraph==0.12.4 PyQt5==5.14.1 torch==1.10.0+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
#如何再次安装  2.0.0 +nv23.05
#wget https://nvidia.box.com/shared/static/w4r7ta0gpb9z8t2mlkcb7kg2cnm28tff.whl -O torch-2.0.0-cp38-cp38-linux_aarch64.whl
#sudo pip3 install torch-2.0.0-cp38-cp38-linux_aarch64.whl
#sudo pip3 install torchvision==0.15.2
#python3 -c "import torch; print(torch.__version__)"

https://github.com/samshoni/gesture_recognition_ros2.gitgit c=asdsd
# 把当前用户加进 video 组
sudo usermod -aG video $USER

# 立刻在当前 shell 切换到新组（免重登；也可直接注销后重新登录）
newgrp video

# 确认已经在 video 组里
groups $USER
/dev/video2：'Z16' —— 深度流（Depth）

/dev/video4：GREY / UYVY / Y8I / Y12I —— 红外/双目红外流

/dev/video6：YUYV 且有 1920×1080 / 1280×720 / 640×480 —— 这就是彩色相机（Color）


 source install/setup.bash && ros2 launch  yolov8_ros2 distance_yolo.launch.py
 
 # 先起 RealSense
ros2 launch realsense2_camera rs_launch.py align_depth:=true
# 再跑你的节点
source install/setup.bash &&  ros2 launch yolov8_ros2 distance_yolo.launch.py \
  use_ros_image:=true \
  image_topic:=/camera/camera/color/image_raw \
  use_depth:=true \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  is_depth_aligned:=false \
  auto_resize_depth:=true \
  device:=cuda:0 conf:=0.25 imgsz:=640 enable_viz:=true \
  depth_min:=0.2 depth_max:=6.0 roi_shrink:=0.3 roi_bottom_only:=true


source install/setup.bash && ros2 run yolov8_ros2 distance_estimate \
  --ros-args \
  -p use_ros_image:=true \
  -p image_topic:=/camera/camera/color/image_raw \
  -p use_depth:=true \
  -p depth_topic:=/camera/camera/depth/image_rect_raw \
  -p is_depth_aligned:=true \
  -p enable_viz:=false

  
https://github.com/samshoni/gesture_recognition_ros2.git

1.gesture_control



pip install  mediapipe 
export FOLLOW_MODE=pose        # 每个周期在 odom 上“迈一步”，发 /goal_pose
export FOLLOW_MAX_STEP=0.4     # 每步最大位移

1.ros2 launch realsense2_camera rs_launch.py    #  key world 
2.python3 gesture_control.py  

source install/setup.bash && ros2 run gesture_controller gesture_controller \
  --ros-args -p image_topic:=/camera/camera/color/image_raw -p enable_viz:=true



gunicorn drl_agent_3_service_fastbin:app -b 0.0.0.0:5000 --workers 1 --threads 2 --keep-alive 60
source install/setup.bash && ros2 run go2_drl1 td3receive
2.python3 gesture_control.py    一定要右手操作  


3
source install/setup.bash && ros2 run yolov8_ros2 distance_estimate \
  --ros-args \
  -p use_ros_image:=true \
  -p image_topic:=/camera/camera/color/image_raw \
  -p use_depth:=true \
  -p depth_topic:=/camera/camera/depth/image_rect_raw \
  -p is_depth_aligned:=true \
  -p enable_viz:=false



# 让机器人以“2.0m”距离认为太远，会前进
ros2 topic pub -r 5 /human_distance std_msgs/Float32 "data: 2.0"
# 让距离=1.2m（在死区）→ 停
ros2 topic pub -r 5 /human_distance std_msgs/Float32 "data: 1.2"




export FOLLOW_MODE=pose        # 每个周期在 odom 上“迈一步”，发 /goal_pose
export FOLLOW_MAX_STEP=0.4     # 每步最大位移
















