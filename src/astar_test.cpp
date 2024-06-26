#include "m3_explorer/astar.h"
#include <Eigen/Dense>
#include <chrono>
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
  ros::init(argc, argv, "astar_test");
  ros::NodeHandle nh("");

  // get octomap
  ros::Subscriber octomap_sub =
      nh.subscribe<octomap_msgs::Octomap>("/merged_map", 1, octomap_cb);

  // visualize waypoint
  ros::Publisher wp_pub =
      nh.advertise<visualization_msgs::Marker>("/waypoint", 10);

  visualization_msgs::Marker waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = ros::Time::now();

  waypoint.type = visualization_msgs::Marker::LINE_STRIP;
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

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> xy(-10.0, 10.0);
  std::uniform_real_distribution<float> z(1.0, 2.0);

  ros::Rate rate(2);

  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
    if(ocmap == nullptr) continue;
    // 设定起点终点
    Eigen::Vector3f start_pt = {xy(gen), xy(gen), z(gen)};
    Eigen::Vector3f end_pt = {xy(gen), xy(gen), z(gen)};
    octomap::OcTreeNode *start_node = ocmap->search(start_pt.x(), start_pt.y(), start_pt.z());
    octomap::OcTreeNode *end_node = ocmap->search(end_pt.x(), end_pt.y(), end_pt.z());
    if(!start_node || !end_node) continue;
    if(ocmap->isNodeOccupied(start_node) || ocmap->isNodeOccupied(end_node)) continue;

    // A*寻路，并统计时间
    Astar astar;
    auto start_time = std::chrono::system_clock::now();
    astar.astar_path_distance(ocmap, start_pt, end_pt);
    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<float, std::milli> elapsed = end_time - start_time;
    cout << "[astar search] :" << elapsed.count() << " ms" << endl;
    // 可视化轨迹
    waypoint.points.clear();
    const int wp_num = astar.path_.size();
    for (int i = 0; i < wp_num; ++i) {
      const Eigen::Vector3f pos = astar.path_[i].position_;
      geometry_msgs::Point wp_pos;
      wp_pos.x = pos.x();
      wp_pos.y = pos.y();
      wp_pos.z = pos.z();
      waypoint.points.emplace_back(wp_pos);
    }
    wp_pub.publish(waypoint);
  }
}
