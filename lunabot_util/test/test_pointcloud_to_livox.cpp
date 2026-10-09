#include <gtest/gtest.h>

#include <limits>

#include "../src/pointcloud_to_livox.hpp"

TEST(PointCloudToLivox, DropsNoReturnRaysAndKeepsHeader)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "livox_frame";
  cloud.header.stamp.sec = 2;
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(3);

  const float inf = std::numeric_limits<float>::infinity();
  const float pts[3][3] = {{1, 2, 3}, {inf, inf, inf}, {-4, 5, -6}};
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto & p : pts)
  {
    *x = p[0];
    *y = p[1];
    *z = p[2];
    ++x, ++y, ++z;
  }

  const auto out = to_livox(cloud);
  ASSERT_EQ(out.point_num, 2u);
  ASSERT_EQ(out.points.size(), 2u);
  EXPECT_EQ(out.header.frame_id, "livox_frame");
  EXPECT_EQ(out.timebase, 2000000000u);
  EXPECT_FLOAT_EQ(out.points[1].x, -4.0f);
  EXPECT_FLOAT_EQ(out.points[1].z, -6.0f);
}
