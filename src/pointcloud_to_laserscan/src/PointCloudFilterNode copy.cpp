// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

// class RobotBodyCropNode : public rclcpp::Node
// {
// public:
//   RobotBodyCropNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
//       : Node("robotbody_crop", options)
//   {
    
//     declare_parameter<std::string>("input_cloud", "/points");
//     declare_parameter<std::string>("output_cloud", "/cropped_cloud");
//     declare_parameter<double>("robot_length", 0.70);
//     declare_parameter<double>("robot_width", 0.37);
//     declare_parameter<double>("lidar_offset_front", 0.14);

//     get_parameter("input_cloud", input_topic_);
//     get_parameter("output_cloud", output_topic_);
//     get_parameter("robot_length", robot_length_);
//     get_parameter("robot_width", robot_width_);
//     get_parameter("lidar_offset_front", lidar_offset_front_);

//     // 雷达相对于机器人中心（+X为前）
//     lidar_x_ = robot_length_ / 2.0 - lidar_offset_front_;
//     half_l_ = robot_length_ / 2.0;
//     half_w_ = robot_width_ / 2.0;

//     sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
//         input_topic_, rclcpp::SensorDataQoS(),
//         std::bind(&RobotBodyCropNode::cloudCallback, this, std::placeholders::_1));

//     pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

//     RCLCPP_INFO(get_logger(), "RobotBodyCropNode: crop box: L=%.2f, W=%.2f, lidar_x=%.2f (frame_id should be LIDAR)",
//                 robot_length_, robot_width_, lidar_x_);
//   }

// private:
//   // 判断点是否在机器人本体矩形内部
//   bool is_point_in_robot_body(double x, double y) const
//   {
//     // 雷达frame下，机器人中心在 (-lidar_x_, 0)
//     double px = x + lidar_x_;
//     double py = y;
//     return (px >= -half_l_ && px <= half_l_ && py >= -half_w_ && py <= half_w_);
//   }

//   void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//   {
//     // 点云frame必须是雷达frame！（velodyne/velodyne_base_link/laser等）
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*msg, *cloud_in);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
//     cloud_out->reserve(cloud_in->size());

//     size_t remove_count = 0;
//     for (const auto &pt : cloud_in->points)
//     {
//       if (!is_point_in_robot_body(pt.x, pt.y))
//       {
//         cloud_out->push_back(pt);
//       }
//       else
//       {
//         remove_count++;
//       }
//     }

//     sensor_msgs::msg::PointCloud2 out_msg;
//     pcl::toROSMsg(*cloud_out, out_msg);
//     out_msg.header = msg->header;
//     pub_->publish(out_msg);

//     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
//                          "[body_crop] in=%zu, out=%zu, removed=%zu",
//                          cloud_in->size(), cloud_out->size(), remove_count);
//   }

//   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
//   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

//   std::string input_topic_, output_topic_;
//   double robot_length_, robot_width_, lidar_offset_front_;
//   double half_l_, half_w_, lidar_x_;
// };

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(RobotBodyCropNode)
