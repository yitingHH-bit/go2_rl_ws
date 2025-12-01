cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan)

add_compile_options(-std=c++17 -O3)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(tf2_ros        REQUIRED)
find_package(nav_msgs    REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen      REQUIRED)
find_package(std_msgs REQUIRED)


# pointcloud -> PointCloudToScanNode
ament_auto_add_library(pointcloud_to_scan SHARED
  src/pointcloud_to_scan_node.cpp)
rclcpp_components_register_node(pointcloud_to_scan
  PLUGIN "pointcloud_to_laserscan::PointCloudToScanNode"
  EXECUTABLE PointCloudToScanNode)

# pointcloud -> PointCloudToScanNode_580
ament_auto_add_library(pointcloud_to_scan_580 SHARED
  src/pointcloud_to_scan_node_580.cpp)
rclcpp_components_register_node(pointcloud_to_scan_580
  PLUGIN "pointcloud_to_laserscan::PointCloudToScanNode_580"
  EXECUTABLE PointCloudToScanNode_580)
  
# pointcloud -> subscribe_height_map_Node
#ament_auto_add_library(subscribe_height_map_Node SHARED
 # src/subscribe_height_map.cpp)
#rclcpp_components_register_node(subscribe_height_map_Node
  # "pointcloud_to_laserscan::subscribe_height_map"
  #EXECUTABLE subscribe_height_map)

#  for  subscribe_height_map
# 头文件目录（⚠️ 第一个参数是目标名，不是路径）
#target_include_directories(subscribe_height_map_Node PRIVATE
 # /usr/local/include/ddscxx
  #/usr/local/include/iceoryx/v2.0.2
#)

# 链接库（更稳妥：先 find_library，再链接到目标）
#find_library(UNITREE_SDK2_LIB unitree_sdk2 PATHS /usr/local/lib REQUIRED)
#(DDSC_LIB         ddsc         PATHS /usr/local/lib REQUIRED)
#find_library(DDSCXX_LIB       ddscxx       PATHS /usr/local/lib REQUIRED)

#target_link_libraries(subscribe_height_map_Node
 # ${UNITREE_SDK2_LIB}
 # ${DDSC_LIB}
 # ${DDSCXX_LIB}
 # rt
 # pthread
#)

# 不要再 add_executable(subscribe_height_map ...) —— 已由 rclcpp_components_register_node 生成

#

# statistical outlier removal 点云过滤器节点



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
