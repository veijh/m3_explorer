#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H
#include <ros/ros.h>
#include <string>
#include "lkh_ros/Solve.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
using namespace std;
geometry_msgs::PoseArray atsp_path(const geometry_msgs::PointStamped& current_pose, const geometry_msgs::PoseArray view_points, ros::ServiceClient& lkh_client, const string& problem_path);

#endif
