#ifndef FRONTIER_DETECTOR_H
#define FRONTIER_DETECTOR_H
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <vector>
#include <set>
#include <queue>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <sys/time.h>
#include <utility>
#include <cmath>

using namespace std;

struct QuadMesh
{
  double size;
  octomap::point3d center;
  octomap::point3d normal;
  bool operator < (const QuadMesh& item) const{
    if(size != item.size){
      return size < item.size;
    }
    else if(center.x() != item.center.x()){
      return center.x() < item.center.x();
    }
    else if(center.y() != item.center.y()){
      return center.y() < item.center.y();
    }
    else if(center.z() != item.center.z()){
      return center.z() < item.center.z();
    }
    else if(normal.x() != item.normal.x()){
      return normal.x() < item.normal.x();
    }
    else if(normal.y() != item.normal.y()){
      return normal.y() < item.normal.y();
    }
    else{
      return normal.z() < item.normal.z();
    }
  }
};

// detect frontier: 
// input = octomap; current pose; FOV; max range; region bbx
// output = a set containing all frontier voxels
void frontier_detect(set<QuadMesh>& frontiers, octomap::OcTree* ocmap, const geometry_msgs::PointStamped& cur_pose, const double& sensor_range);
#endif
