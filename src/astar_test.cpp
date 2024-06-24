#include "m3_explorer/astar.h"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <visualization_msgs/Marker.h>

int main() {
// 加载octomap

// 设定起点终点
  Eigen::Vector3f start_pt = {0.0, 0.0, 0.0};
  Eigen::Vector3f end_pt = {10.0, 5.0, 0.0};
// A*寻路，并统计时间
  Astar astar;
  astar.astar_path_distance();
// 可视化轨迹

}
