#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

using std::placeholders::_1;

namespace pointcloud_to_laserscan
{

class PointCloudToScanNode : public rclcpp::Node
{
public:
  explicit PointCloudToScanNode(const rclcpp::NodeOptions & options)
  : Node("pointcloud_to_scan", options) { init(); }

  PointCloudToScanNode() : Node("pointcloud_to_scan") { init(); }

private:
  void init()
  {
    // ---- 参数：严格对齐模拟端 ----
    this->declare_parameter<int>("num_scan_samples", 360);     // 目标 H=360
    this->declare_parameter<int>("raw_bins", 1440);            // 原始角分辨率 R_raw
    this->declare_parameter<double>("range_min", 0.20);        // ★ 与模拟端一致
    this->declare_parameter<double>("range_max", 5.0);         // 量程上限
    this->declare_parameter<bool>("normalize", true);          // True→[0,1]
    this->declare_parameter<int>("min_points_per_raw_bin", 0); // 模拟端无此过滤→默认0
    this->declare_parameter<double>("angle_offset_deg", 0.0);  // 可选：角度零位修正
    this->declare_parameter<bool>("invert_yaw", false);        // 可选：方向翻转

    num_scan_samples_        = this->get_parameter("num_scan_samples").as_int();
    raw_bins_                = this->get_parameter("raw_bins").as_int();
    range_min_               = this->get_parameter("range_min").as_double();
    range_max_               = this->get_parameter("range_max").as_double();
    normalize_               = this->get_parameter("normalize").as_bool();
    min_points_per_raw_bin_  = this->get_parameter("min_points_per_raw_bin").as_int();
    angle_offset_deg_        = this->get_parameter("angle_offset_deg").as_double();
    invert_yaw_              = this->get_parameter("invert_yaw").as_bool();

    // 保障 raw_bins_ ≥ 目标 360
    if (raw_bins_ < num_scan_samples_) {
      RCLCPP_WARN(this->get_logger(),
        "raw_bins (%d) < num_scan_samples (%d)，强制将 raw_bins 设为 %d。",
        raw_bins_, num_scan_samples_, num_scan_samples_);
      raw_bins_ = num_scan_samples_;
    }

    d_raw_.assign(raw_bins_, range_max_);  // 原始 R_raw 桶，初值=最远
    cnt_raw_.assign(raw_bins_, 0);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_cloud", rclcpp::SensorDataQoS(),
      std::bind(&PointCloudToScanNode::cloudCallback, this, _1));

    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "processed_scan", 10);

    RCLCPP_INFO(this->get_logger(),
      "Init OK | target=%d, raw_bins=%d, range=[%.2f, %.2f], normalize=%s, angle_offset=%.1fdeg, invert_yaw=%s",
      num_scan_samples_, raw_bins_, range_min_, range_max_,
      normalize_ ? "true" : "false",
      angle_offset_deg_, invert_yaw_ ? "true" : "false");
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 1) 原始角度分桶（R_raw），XY 距离，最小值聚合
    std::fill(d_raw_.begin(), d_raw_.end(), range_max_);
    std::fill(cnt_raw_.begin(), cnt_raw_.end(), 0);

    const double angle_min = -M_PI;
    const double angle_max =  M_PI;
    const double angle_span = angle_max - angle_min;

    for (sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x"),
                                                      it_y(*msg, "y");
         it_x != it_x.end(); ++it_x, ++it_y)
    {
      const float x = *it_x;
      const float y = *it_y;
      double r = std::hypot(x, y);  // 仅 XY

      // 越界/无效 → 跳过（空桶默认=最远，与模拟端等价）
      if (!std::isfinite(r) || r < range_min_ || r > range_max_) continue;

      double angle = std::atan2(y, x);  // [-pi, pi]
      if (invert_yaw_) angle = -angle;
      if (angle_offset_deg_ != 0.0) {
        angle += angle_offset_deg_ * M_PI / 180.0;
        // wrap 到 [-pi, pi]
        if (angle >  M_PI) angle -= 2*M_PI;
        if (angle < -M_PI) angle += 2*M_PI;
      }

      if (angle < angle_min || angle > angle_max) continue;

      int idx = static_cast<int>((angle - angle_min) / angle_span * raw_bins_);
      if (idx < 0) idx = 0;
      if (idx >= raw_bins_) idx = raw_bins_ - 1;

      if (r < d_raw_[idx]) d_raw_[idx] = r;  // 最小值聚合
      cnt_raw_[idx]++;
    }

    // 可选：原始桶计数过低时置最远（为与模拟端一致，默认禁用）
    if (min_points_per_raw_bin_ > 0) {
      for (int i = 0; i < raw_bins_; ++i) {
        if (cnt_raw_[i] < min_points_per_raw_bin_) d_raw_[i] = range_max_;
      }
    }

    // 2) 等价 Python 的 “比例映射” 下采样 → 覆盖完整 360°
    const int target_size = num_scan_samples_; // 360
    const int R = raw_bins_;

    std::vector<double> d_target(target_size, range_max_);

    // 预构建 raw->target 的映射：idx_r[i] = floor(i * target / R)
    static thread_local std::vector<int> idx_r;
    idx_r.resize(R);
    for (int i = 0; i < R; ++i) {
      int b = static_cast<int>((static_cast<int64_t>(i) * target_size) / R);
      if (b >= target_size) b = target_size - 1;
      idx_r[i] = b;
    }
    // 聚合最小值
    for (int i = 0; i < R; ++i) {
      const int b = idx_r[i];
      const double v = d_raw_[i];
      if (v < d_target[b]) d_target[b] = v;
    }

    // 3) 归一化或输出米制；并发布为 layout=(360,1)
    std_msgs::msg::Float32MultiArray out_msg;
    out_msg.data.resize(target_size);

    if (normalize_) {
      const double denom = (range_max_ - range_min_);
      for (int i = 0; i < target_size; ++i) {
        double d = std::clamp(d_target[i], range_min_, range_max_);
        double norm = (denom > 0.0) ? ((d - range_min_) / denom) : 1.0;
        norm = std::clamp(norm, 0.0, 1.0);
        out_msg.data[i] = static_cast<float>(norm);
      }
    } else {
      for (int i = 0; i < target_size; ++i) {
        double d = std::clamp(d_target[i], range_min_, range_max_);
        out_msg.data[i] = static_cast<float>(d);
      }
    }

    // layout 声明为 2D： (360,1)
    out_msg.layout.dim.resize(2);
    out_msg.layout.dim[0].label  = "rows";
    out_msg.layout.dim[0].size   = num_scan_samples_;
    out_msg.layout.dim[0].stride = num_scan_samples_ * 1; // rows*cols(=1)
    out_msg.layout.dim[1].label  = "cols";
    out_msg.layout.dim[1].size   = 1;
    out_msg.layout.dim[1].stride = 1;
    out_msg.layout.data_offset = 0;

    pub_->publish(out_msg);
  }

  // --- 成员 ---
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;

  int num_scan_samples_;         // 目标 360
  int raw_bins_;                 // 原始角分辨率 R
  double range_min_, range_max_;
  bool normalize_;
  int min_points_per_raw_bin_;
  double angle_offset_deg_;
  bool invert_yaw_;

  std::vector<double> d_raw_;    // 原始 R 桶：最小值聚合（空桶=range_max）
  std::vector<int>    cnt_raw_;  // 原始 R 桶计数
};

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToScanNode)
