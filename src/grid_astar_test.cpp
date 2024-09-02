#include "m3_explorer/grid_astar.h"
#include "m3_explorer/time_track.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <random>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

octomap::OcTree *ocmap = nullptr;
void octomap_cb(const octomap_msgs::Octomap::ConstPtr &msg) {
  delete ocmap;
  ocmap = dynamic_cast<octomap::OcTree *>(msgToMap(*msg));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grid_astar_test");
  ros::NodeHandle nh("");

  // get octomap
  ros::Subscriber octomap_sub =
      nh.subscribe<octomap_msgs::Octomap>("/merged_map", 1, octomap_cb);

  // visualize waypoint
  ros::Publisher wp_pub =
      nh.advertise<visualization_msgs::Marker>("/waypoint", 10);

  // visualize map
  ros::Publisher node_pub =
      nh.advertise<visualization_msgs::Marker>("/leafnode", 10);

  ros::Rate rate(0.5);

  visualization_msgs::Marker cube_list;
  cube_list.header.frame_id = "map";
  cube_list.header.stamp = ros::Time::now();
  cube_list.ns = "cube_list";
  cube_list.action = visualization_msgs::Marker::ADD;
  cube_list.pose.orientation.w = 1.0;
  cube_list.id = 0;
  cube_list.type = visualization_msgs::Marker::CUBE_LIST;
  cube_list.scale.x = 0.09;
  cube_list.scale.y = 0.09;
  cube_list.scale.z = 0.09;

  visualization_msgs::Marker waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = ros::Time::now();
  waypoint.ns = "line_strip";
  waypoint.action = visualization_msgs::Marker::ADD;
  waypoint.scale.x = 0.1;
  waypoint.pose.orientation.w = 1.0;
  waypoint.id = 0;
  waypoint.type = visualization_msgs::Marker::LINE_STRIP;
  waypoint.color.r = 1.0;
  waypoint.color.g = 1.0;
  waypoint.color.b = 1.0;
  waypoint.color.a = 1.0;

  const float min_x = -11.0;
  const float max_x = 11.0;
  const float min_y = -11.0;
  const float max_y = 11.0;
  const float min_z = -1.0;
  const float max_z = 4.0;
  const float resolution = 0.1;
  GridAstar grid_astar(min_x, max_x, min_y, max_y, min_z, max_z, resolution);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> xy(min_x, max_x);
  std::uniform_real_distribution<float> z(1.0, 2.0);

  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
    if (ocmap == nullptr) {
      continue;
    }
    octomap::point3d bx_min(min_x, min_y, min_z);
    octomap::point3d bx_max(max_x, max_y, max_z);
    TimeTrack track;
    grid_astar.UpdateFromMap(ocmap, bx_min, bx_max);
    track.OutputPassingTime("Update Map");

    track.SetStartTime();
    const std::vector<std::vector<std::vector<GridAstar::GridState>>>
        &grid_map = grid_astar.grid_map();

    const int x_size = grid_map.size();
    const int y_size = grid_map[0].size();
    const int z_size = grid_map[0][0].size();
    cube_list.points.clear();
    cube_list.colors.clear();
    for (int i = 0; i < x_size; ++i) {
      for (int j = 0; j < y_size; ++j) {
        for (int k = 0; k < z_size; ++k) {
          if (grid_map[i][j][k] == GridAstar::GridState::kOcc) {
            geometry_msgs::Point grid_pos;
            grid_pos.x = min_x + i * resolution + 0.5 * resolution;
            grid_pos.y = min_y + j * resolution + 0.5 * resolution;
            grid_pos.z = min_z + k * resolution + 0.5 * resolution;
            cube_list.points.emplace_back(grid_pos);
            cube_list.colors.emplace_back();
            cube_list.colors.back().a = 1.0;
            cube_list.colors.back().g = 0.8;
          }
        }
      }
    }
    node_pub.publish(cube_list);
    track.OutputPassingTime("Visualize Map");

    // 设定起点终点
    // Eigen::Vector3f start_pt = {xy(gen), xy(gen), z(gen)};
    // Eigen::Vector3f end_pt = {xy(gen), xy(gen), z(gen)};
    Eigen::Vector3f start_pt = {0.0, 0.0, 1.0};
    Eigen::Vector3f end_pt = {2.0, 0.0, 1.0};

    // 检查是否起点、终点是否合法
    int index_start_x =
        static_cast<int>(std::floor((start_pt.x() - min_x) / resolution));
    int index_start_y =
        static_cast<int>(std::floor((start_pt.y() - min_y) / resolution));
    int index_start_z =
        static_cast<int>(std::floor((start_pt.z() - min_z) / resolution));
    int index_end_x =
        static_cast<int>(std::floor((end_pt.x() - min_x) / resolution));
    int index_end_y =
        static_cast<int>(std::floor((end_pt.y() - min_y) / resolution));
    int index_end_z =
        static_cast<int>(std::floor((end_pt.z() - min_z) / resolution));
    if (grid_map[index_start_x][index_start_y][index_start_z] ==
            GridAstar::GridState::kOcc ||
        grid_map[index_end_x][index_end_y][index_end_z] ==
            GridAstar::GridState::kOcc) {
      continue;
    }

    // A*寻路，并统计时间
    track.SetStartTime();
    grid_astar.AstarPathDistance(start_pt, end_pt);
    track.OutputPassingTime("Astar Search");
    // 可视化轨迹
    waypoint.points.clear();
    const std::vector<std::shared_ptr<GridAstarNode>> &path = grid_astar.path();
    const int wp_num = path.size();
    for (int i = 0; i < wp_num; ++i) {
      geometry_msgs::Point wp_pos;
      wp_pos.x = min_x + path[i]->index_x_ * resolution;
      wp_pos.y = min_y + path[i]->index_y_ * resolution;
      wp_pos.z = min_z + path[i]->index_z_ * resolution;
      waypoint.points.emplace_back(wp_pos);
    }
    wp_pub.publish(waypoint);
  }
}

// octomap::point3d bx_min(-10.0, -10.0, 0.2);
// octomap::point3d bx_max(10.0, 10.0, 0.4);
// int count = 0;
// marker_array.markers.clear();
// for(octomap::OcTree::leaf_bbx_iterator it = ocmap->begin_leafs_bbx(bx_min,
// bx_max), end=ocmap->end_leafs_bbx(); it != end; it++){
//   visualization_msgs::Marker box;
//   box.header.frame_id = "map";
//   box.header.stamp = ros::Time::now();

//   box.action = visualization_msgs::Marker::ADD;
//   box.scale.x = it.getSize()*0.8;
//   box.scale.y = it.getSize()*0.8;
//   box.scale.z = it.getSize()*0.8;
//   box.pose.orientation.w = 1.0;
//   box.id = count;
//   box.type = visualization_msgs::Marker::CUBE;
//   box.color.r = 0.0;
//   box.color.g = 0.5;
//   box.color.b = 0.0;
//   box.color.a = 1.0;
//   box.pose.position.x = it.getCoordinate().x();
//   box.pose.position.y = it.getCoordinate().y();
//   box.pose.position.z = it.getCoordinate().z();
//   marker_array.markers.emplace_back(box);
//   ++count;
// }
// node_pub.publish(marker_array);
