/**
 * @file pointcloud_to_livox.cpp
 * @date 10/9/2026
 */

#include "pointcloud_to_livox.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @class PointCloudToLivox
 * @brief Converts the Gazebo gpu_lidar PointCloud2 into the Livox CustomMsg the real MID-360
 * driver publishes, so the simulated lidar feeds the same LIO pipeline.
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
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { pub_->publish(to_livox(*msg)); }

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
