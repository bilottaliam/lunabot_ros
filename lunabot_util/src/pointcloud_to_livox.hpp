#pragma once

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <cmath>

/**
 * @brief Converts a PointCloud2 with x/y/z fields into a Livox CustomMsg.
 * Rays with no return (inf/nan) are dropped. gpu_lidar scans are instantaneous, so every
 * point gets offset_time 0.
 */
inline livox_ros_driver2::msg::CustomMsg to_livox(const sensor_msgs::msg::PointCloud2 & msg)
{
  livox_ros_driver2::msg::CustomMsg out;
  out.header = msg.header;
  out.timebase = rclcpp::Time(msg.header.stamp).nanoseconds();
  out.points.reserve(msg.width * msg.height);

  sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
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
    p.offset_time = 0;
    out.points.push_back(p);
  }
  out.point_num = out.points.size();
  return out;
}
