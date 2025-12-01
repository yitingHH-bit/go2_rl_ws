// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_components/register_node_macro.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <std_msgs/msg/float32_multi_array.hpp>

// #include <unitree/robot/channel/channel_subscriber.hpp>
// #include <unitree/robot/channel/channel_factory.hpp>
// #include <unitree/idl/go2/HeightMap_.hpp>

// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <mutex>
// #include <string>

// using unitree::robot::ChannelSubscriber;
// using unitree::robot::ChannelFactory;

// namespace pointcloud_to_laserscan {

// // === 与仿真一致的网格参数 ===
// static constexpr float VOX = 0.06f;
// static constexpr float RX0 = -0.8f, RX1 = 0.2f;   // x ∈ [-0.8, 0.2]
// static constexpr float RY0 = -0.8f, RY1 = 0.8f;   // y ∈ [-0.8, 0.8]
// static constexpr int   NX  = int(std::floor((RX1 - RX0)/VOX)) + 1; // 17
// static constexpr int   NY  = int(std::floor((RY1 - RY0)/VOX)) + 1; // 27
// static_assert(NX==17 && NY==27, "grid must be 17x27");

// static constexpr float OFFSET = 0.0f;
// static constexpr float EMPTY  = 1.0e9f;
// static constexpr const char* DDS_TOPIC = "rt/utlidar/height_map_array";

// struct Pose2D { double x{0}, y{0}, yaw{0}; };

// class subscribe_height_map : public rclcpp::Node {
// public:
//   explicit subscribe_height_map(const rclcpp::NodeOptions& options)
//   : rclcpp::Node("subscribe_height_map_Node", options)
//   {
//     // 参数：网卡名、里程计话题、输出话题
//     nic_        = this->declare_parameter<std::string>("nic", "enp3s0");
//     odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/utlidar/robot_odom");
//     out_topic_  = this->declare_parameter<std::string>("out_topic", "heightmap/resampled_459");

//     pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(out_topic_, 10);

//     // 订阅里程计
//     sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
//       odom_topic_, 10,
//       [this](const nav_msgs::msg::Odometry::SharedPtr msg){
//         std::lock_guard<std::mutex> lk(pose_mtx_);
//         pose_.x = msg->pose.pose.position.x;
//         pose_.y = msg->pose.pose.position.y;
//         const auto& q = msg->pose.pose.orientation;
//         pose_.yaw = quat2yaw(q.w, q.x, q.y, q.z);
//       });

//     // 初始化 DDS 并订阅官方高度图
//     ChannelFactory::Instance()->Init(0, nic_.c_str());
//     dds_sub_ = std::make_unique<ChannelSubscriber<unitree_go::msg::dds_::HeightMap_>>(DDS_TOPIC);

//     // 使用静态回调转发到实例方法
//     g_this_ = this;
//     dds_sub_->InitChannel(&subscribe_height_map::DDSHandlerStatic);

//     RCLCPP_INFO(get_logger(), "subscribe_height_map ready. NIC=%s, odom_topic=%s, out_topic=%s",
//                 nic_.c_str(), odom_topic_.c_str(), out_topic_.c_str());
//   }

// private:
//   // ====== DDS 静态回调桥 ======
//   static void DDSHandlerStatic(const void* message){
//     if (g_this_) g_this_->handleDDS(message);
//   }

//   void handleDDS(const void* message){
//     const auto* map = reinterpret_cast<const unitree_go::msg::dds_::HeightMap_*>(message);

//     Pose2D p;
//     { std::lock_guard<std::mutex> lk(pose_mtx_); p = pose_; }

//     const float c = std::cos((float)p.yaw), s = std::sin((float)p.yaw);

//     std::vector<float> hm(NX*NY, 0.f);
//     auto idx = [&](int ix,int iy){ return ix*NY + iy; };

//     for(int ix=0; ix<NX; ++ix){
//       const float xr = RX0 + ix*VOX;
//       for(int iy=0; iy<NY; ++iy){
//         const float yr = RY0 + iy*VOX;

//         // 机体系 -> 世界系(odom)
//         const float xw = (float)p.x + c*xr - s*yr;
//         const float yw = (float)p.y + s*xr + c*yr;

//         float z;
//         if (bilinearSample(*map, xw, yw, z)) {
//           float v = z - OFFSET;
//           if (v < 0.05f) v = 0.0f;
//           if (!std::isfinite(v)) v = 0.0f;
//           hm[idx(ix,iy)] = v;
//         } else {
//           hm[idx(ix,iy)] = 0.0f;
//         }
//       }
//     }

//     maxPool3x3(hm);

//     // 发布 Float32MultiArray（长度 459）
//     std_msgs::msg::Float32MultiArray msg;
//     msg.data = hm;
//     pub_->publish(msg);
//   }

//   // ===== 工具函数 =====
//   static inline double quat2yaw(double w,double x,double y,double z){
//     return std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
//   }

//   static inline bool bilinearSample(
//     const unitree_go::msg::dds_::HeightMap_& map, float xw, float yw, float& z_out)
//   {
//     const float res = map.resolution();
//     const float x0 = map.origin()[0], y0 = map.origin()[1];
//     const int   W = (int)map.width(),   H = (int)map.height();
//     const auto& D = map.data();

//     const float gx = (xw - x0)/res, gy = (yw - y0)/res;
//     const int x0i = (int)std::floor(gx), y0i = (int)std::floor(gy);
//     const int x1i = x0i + 1, y1i = y0i + 1;
//     if (x0i<0 || y0i<0 || x1i>=W || y1i>=H) return false;

//     auto At = [&](int ix,int iy)->float { return D[iy*W + ix]; };

//     float z00 = At(x0i,y0i), z10=At(x1i,y0i);
//     float z01 = At(x0i,y1i), z11=At(x1i,y1i);

//     if (z00==EMPTY || z10==EMPTY || z01==EMPTY || z11==EMPTY) {
//       const int xn = std::clamp((int)std::lround(gx), 0, W-1);
//       const int yn = std::clamp((int)std::lround(gy), 0, H-1);
//       float zn = At(xn,yn);
//       if (zn==EMPTY || !std::isfinite(zn)) return false;
//       z_out = zn; return true;
//     }

//     const float tx = gx - x0i, ty = gy - y0i;
//     const float z0 = z00*(1.f - tx) + z10*tx;
//     const float z1 = z01*(1.f - tx) + z11*tx;
//     const float z  = z0*(1.f - ty) + z1*ty;
//     if (!std::isfinite(z)) return false;
//     z_out = z; return true;
//   }

//   static inline void maxPool3x3(std::vector<float>& grid){
//     std::vector<float> out(grid.size(), 0.f);
//     auto idx=[&](int ix,int iy){ return ix*NY + iy; };
//     for(int ix=0; ix<NX; ++ix){
//       for(int iy=0; iy<NY; ++iy){
//         float m = grid[idx(ix,iy)];
//         for(int dx=-1; dx<=1; ++dx){
//           for(int dy=-1; dy<=1; ++dy){
//             int jx = std::clamp(ix+dx, 0, NX-1);
//             int jy = std::clamp(iy+dy, 0, NY-1);
//             m = std::max(m, grid[idx(jx,jy)]);
//           }
//         }
//         out[idx(ix,iy)] = m;
//       }
//     }
//     grid.swap(out);
//   }

// private:
//   // 成员
//   std::string nic_, odom_topic_, out_topic_;
//   std::unique_ptr<ChannelSubscriber<unitree_go::msg::dds_::HeightMap_>> dds_sub_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
//   rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
//   Pose2D pose_;
//   std::mutex pose_mtx_;

//   // 静态 self 指针用于 DDS C 回调桥
//   static inline subscribe_height_map* g_this_ = nullptr;
// };

// } // namespace pointcloud_to_laserscan

// RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::subscribe_height_map)
