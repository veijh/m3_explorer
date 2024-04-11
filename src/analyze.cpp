#include <ros/ros.h>
#include <vector>
#include <unordered_map>
#include <set>
#include <queue>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <sys/time.h>
#include <utility>
#include <cmath>
#include <random>
#include "m3_explorer/frontier_detector.h"
#include "m3_explorer/frontier_cluster.h"
#include "m3_explorer/path_planning.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include "lkh_ros/Solve.h"
#include "fstream"

using namespace std;

std::ofstream outputFile("/src/output.txt");

octomap::OcTree* ocmap;
void octomap_cb(const octomap_msgs::Octomap::ConstPtr& msg)
{
  // free memory for old map
  delete ocmap;
  ocmap = dynamic_cast<octomap::OcTree*>(msgToMap(*msg));
}

ros::Publisher marker_pub;
visualization_msgs::Marker line_strip;
nav_msgs::Odometry uav_odom;
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  uav_odom = *msg;
  geometry_msgs::Point p;
  p.x = uav_odom.pose.pose.position.x;
  p.y = uav_odom.pose.pose.position.y;
  p.z = uav_odom.pose.pose.position.z;
  line_strip.points.push_back(p);
  marker_pub.publish(line_strip);
}

void timerCallback(const ros::TimerEvent&)
{
  if(ocmap == nullptr) return;

  int count = 0;
  for(double x = -10.0; x <= 10.0; x += 0.1){
    for(double y = -10.0; y <= 10.0; y += 0.1){
      for(double z = 0.0; z <= 2.5; z += 0.1){
        if(ocmap->search(octomap::point3d(x,y,z)) != nullptr) {
          ++count;
        }
      }
    }
  }
  outputFile << count << endl;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "analyze");
  ros::NodeHandle nh("");
  // get octomap
  ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/uav0/octomap_binary", 1, octomap_cb);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/uav0/mavros/local_position/odom", 1, odom_cb);

  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Timer timer = nh.createTimer(ros::Duration(1.0), timerCallback);

  line_strip.header.frame_id = "map"; // 坐标系
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "line_strip";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1; // 线宽
  line_strip.color.b = 1.0; // 红色
  line_strip.color.a = 1.0; // 不透明度

  if (!outputFile.is_open()) { // 检查文件是否成功打开
    std::cerr << "Failed to open the file!" << std::endl;
    return 1;
  }

  ros::spin();

  return 0;
}
