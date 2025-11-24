#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <vector>
#include <string>
#include <cmath>
#include <limits>

double map_resolution;
double max_slope_deg;
double z_min, z_max;
std::string file_directory;
std::string file_name;
std::string map_topic_name;
std::string frame_id;

bool loadPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  std::string full_path = file_directory;
  if (!full_path.empty() && full_path.back() != '/')
    full_path += "/";
  full_path += file_name + ".pcd";

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(full_path, *cloud) == -1) {
    ROS_ERROR("无法加载 PCD 文件: %s", full_path.c_str());
    return false;
  }
  ROS_INFO("成功加载 PCD: %s, 点数: %zu", full_path.c_str(), cloud->points.size());
  return true;
}

void buildTraversabilityMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            nav_msgs::OccupancyGrid &msg) {
  if (cloud->points.empty()) {
    ROS_WARN("输入点云为空，无法生成栅格地图");
    return;
  }

  // 1. 先根据 z 过滤一下点
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  filtered->points.reserve(cloud->points.size());
  for (const auto &pt : cloud->points) {
    if (pt.z >= z_min && pt.z <= z_max) {
      filtered->points.push_back(pt);
    }
  }
  if (filtered->points.empty()) {
    ROS_WARN("z 范围内无点 (%.2f ~ %.2f)，无法生成栅格地图", z_min, z_max);
    return;
  }

  // 2. 计算 XY 边界
  double x_min = filtered->points[0].x;
  double x_max = filtered->points[0].x;
  double y_min = filtered->points[0].y;
  double y_max = filtered->points[0].y;

  for (size_t i = 1; i < filtered->points.size(); ++i) {
    const auto &pt = filtered->points[i];
    if (pt.x < x_min) x_min = pt.x;
    if (pt.x > x_max) x_max = pt.x;
    if (pt.y < y_min) y_min = pt.y;
    if (pt.y > y_max) y_max = pt.y;
  }

  // 3. 设置 OccupancyGrid 基本信息
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;

  msg.info.resolution = map_resolution;
  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width  = static_cast<unsigned int>((x_max - x_min) / map_resolution) + 1;
  msg.info.height = static_cast<unsigned int>((y_max - y_min) / map_resolution) + 1;

  size_t grid_size = static_cast<size_t>(msg.info.width) * msg.info.height;
  msg.data.assign(grid_size, -1);  // 默认未知

  // 4. 统计每个格子的平均高度
  std::vector<double> z_sum(grid_size, 0.0);
  std::vector<int>    z_count(grid_size, 0);

  for (const auto &pt : filtered->points) {
    int ix = static_cast<int>((pt.x - x_min) / map_resolution);
    int iy = static_cast<int>((pt.y - y_min) / map_resolution);

    if (ix < 0 || iy < 0 ||
        ix >= static_cast<int>(msg.info.width) ||
        iy >= static_cast<int>(msg.info.height))
      continue;

    size_t idx = static_cast<size_t>(ix) + static_cast<size_t>(iy) * msg.info.width;
    z_sum[idx]   += pt.z;
    z_count[idx] += 1;
  }

  std::vector<double> z_mean(grid_size, std::numeric_limits<double>::quiet_NaN());
  for (size_t idx = 0; idx < grid_size; ++idx) {
    if (z_count[idx] > 0) {
      z_mean[idx] = z_sum[idx] / static_cast<double>(z_count[idx]);
    }
  }

  // 5. 根据最大爬坡角判断可通行性
  const double PI = 3.14159265358979323846;
  double max_slope_rad = max_slope_deg * PI / 180.0;

  for (unsigned int iy = 0; iy < msg.info.height; ++iy) {
    for (unsigned int ix = 0; ix < msg.info.width; ++ix) {
      size_t idx = static_cast<size_t>(ix) + static_cast<size_t>(iy) * msg.info.width;

      if (z_count[idx] == 0) {
        msg.data[idx] = -1;  // 无点 -> 未知
        continue;
      }

      double max_local_slope = 0.0;
      bool has_neighbor = false;

      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;

          int nx = static_cast<int>(ix) + dx;
          int ny = static_cast<int>(iy) + dy;

          if (nx < 0 || ny < 0 ||
              nx >= static_cast<int>(msg.info.width) ||
              ny >= static_cast<int>(msg.info.height))
            continue;

          size_t nidx = static_cast<size_t>(nx) + static_cast<size_t>(ny) * msg.info.width;
          if (z_count[nidx] == 0) continue;

          has_neighbor = true;

          double dz = std::fabs(z_mean[nidx] - z_mean[idx]);
          double dist_xy = map_resolution * std::sqrt(dx * dx + dy * dy);
          if (dist_xy < 1e-6) continue;

          double slope = std::atan2(dz, dist_xy);
          if (slope > max_local_slope) {
            max_local_slope = slope;
          }
        }
      }

      if (!has_neighbor) {
        msg.data[idx] = -1;         // 周围全是空 -> 未知
      } else if (max_local_slope <= max_slope_rad) {
        msg.data[idx] = 0;          // 可通行
      } else {
        msg.data[idx] = 100;        // 坡度太大 -> 不可通行
      }
    }
  }

  ROS_INFO("生成坡度可通行栅格: width=%u, height=%u, res=%.3f, max_slope_deg=%.1f",
           msg.info.width, msg.info.height, msg.info.resolution, max_slope_deg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcd_traversability");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param("file_directory", file_directory, std::string("/home/robot/catkin_ws/src/fast_lio/PCD"));
  pnh.param("file_name", file_name, std::string("scans"));
  pnh.param("map_resolution", map_resolution, 0.05);
  pnh.param("max_slope_deg", max_slope_deg, 10.0);
  pnh.param("z_min", z_min, -1.0);
  pnh.param("z_max", z_max,  1.0);
  pnh.param("map_topic_name", map_topic_name, std::string("traversability_map"));
  pnh.param("frame_id", frame_id, std::string("map"));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (!loadPointCloud(cloud)) {
    return -1;
  }

  nav_msgs::OccupancyGrid grid;
  buildTraversabilityMap(cloud, grid);

  ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1, true);
  map_pub.publish(grid);

  ROS_INFO("已发布可通行栅格到 topic: %s (latched)", map_topic_name.c_str());

  ros::spin();
  return 0;
}

