/**
 * @file pointcloud_to_livox.cpp
 * @date 10/9/2026
 */

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <cmath>

/**
 * @class PointCloudToLivox
 * @brief Converts the Gazebo gpu_lidar PointCloud2 into the Livox CustomMsg the real MID-360
 * driver publishes, so the simulated lidar feeds the same LIO pipeline. Rays with no return
 * (inf/nan) are dropped.
 */
class PointCloudToLivox : public rclcpp::Node
{
public:
  PointCloudToLivox() : Node("pointcloud_to_livox")
  {
    std::string input_topic = declare_parameter<std::string>("input_topic", "/livox/lidar_points");
    std::string output_topic = declare_parameter<std::string>("output_topic", "/livox/lidar");

    pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(output_topic, 20);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudToLivox::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    livox_ros_driver2::msg::CustomMsg out;
    out.header = msg->header;
    out.timebase = rclcpp::Time(msg->header.stamp).nanoseconds();
    out.points.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
    for (; x != x.end(); ++x, ++y, ++z)
    {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z))
      {
        continue;
      }
      livox_ros_driver2::msg::CustomPoint p;
      p.x = *x;
      p.y = *y;
      p.z = *z;
      // gpu_lidar scans are instantaneous, so every point shares the base time
      p.offset_time = 0;
      out.points.push_back(p);
    }
    out.point_num = out.points.size();
    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToLivox>());
  rclcpp::shutdown();
  return 0;
}
