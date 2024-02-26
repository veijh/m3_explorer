#ifndef FRONTIER_CLUSTER_H
#define FRONTIER_CLUSTER_H
#include <ros/ros.h>
#include "m3_explorer/QuadMesh.h"
#include <set>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <visualization_msgs/MarkerArray.h>

#define MAX_ITERATION (1000)

struct Cluster{
    Eigen::Vector3f center;
    Eigen::Vector3f normal;
};

using namespace std;

//// frontier clustering
//// input = the set containing all frontier voxels
//// output = a vector containing all frontier clusters
vector<Cluster> k_mean_cluster(set<QuadMesh>& frontiers);
void cluster_visualize(vector<Cluster>& cluster_vec, ros::Publisher& cluster_pub);
#endif
