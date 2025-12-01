#!/bin/bash
	
# 等待时间（秒）
WAIT_TIME=8			
#gnome-terminal -- bash -c "./build.sh ; exec bash"
gnome-terminal -- bash -c "colcon build --merge-install --symlink-install; exec bash"

sleep $WAIT_TIME	
gnome-terminal -- bash -c "colcon build --merge-install --symlink-install; exec bash"

gnome-terminal -- bash -c "source install/setup.bash &&ros2 launch rl_sar gazebo.launch.py rname:=go2; exec bash"
sleep $WAIT_TIME
gnome-terminal -- bash -c "source install/setup.bash && ros2 launch pointcloud_to_laserscan pointcloud_filter.launch.py; exec bash"

#gnome-terminal -- bash -c "source install/setup.bash && ros2 run rl_sar rl_sim; exec bash"		
gnome-terminal -- bash -c "source install/setup.bash && ros2 run rl_sar rl_sim; exec bash"  #_lidar	
 
#wget https://nvidia.box.com/shared/static/w4r7ta0gpb9z8t2mlkcb7kg2cnm28tff.whl -O torch-2.0.0-cp38-cp38-linux_aarch64.whl
 
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



