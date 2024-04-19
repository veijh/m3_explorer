#ifndef HASTAR_H
#define HASTAR_H
#include <ros/ros.h>
#include <memory>
#include <queue>
#include <map>
#include <octomap/octomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

using namespace std;
class PathNode{
    private:

    public:
        Eigen::Vector3f position;
        float yaw, vel;
        float vz;

        Eigen::Vector3f father_acc;
        int father_id;
        float f_score, g_score, h_score;

        PathNode(const Eigen::Vector3f& _position, const float& _yaw, const float& _vel, const float& _vz)
            :position(_position), yaw(_yaw), vel(_vel), vz(_vz){};
        PathNode(const float& px, const float& py, const float& pz, const float& _yaw, const float& _vel, const float& _vz)
            :position(Eigen::Vector3f(px, py, pz)), yaw(_yaw), vel(_vel), vz(_vz){};

        // PathNode(const PathNode& node){
        //     position = node.position;
        //     yaw = node.yaw;
        //     vel = node.vel;
        //     vz = node.vz;
        //     father_acc = node.father_acc;
        //     father_id = node.father_id;
        //     f_score = node.f_score;
        //     g_score = node.g_score;
        //     h_score = node.h_score;
        // }
};

struct NodeCmp{
    bool operator()(const PathNode& lhs, const PathNode& rhs){
        return lhs.f_score > rhs.f_score;
    }
};

struct MapCmp{
    bool operator()(const PathNode& lhs, const PathNode& rhs){
        int x0 = (int)lhs.position.x()/0.1;
        int y0 = (int)lhs.position.y()/0.1;
        int z0 = (int)lhs.position.z()/0.1;

        int x1 = (int)rhs.position.x()/0.1;
        int y1 = (int)rhs.position.y()/0.1;
        int z1 = (int)rhs.position.z()/0.1;
        if(x0 != x1) return x0 < x1;
        if(y0 != y1) return y0 < y1;
        return z0 < z1;
    }
};

class Hastar{
    private:
    public:
        vector<Eigen::Matrix3f> traj;
        vector<PathNode> path;
        bool search_path(const octomap::OcTree* ocmap, const Eigen::Vector3f& start_p, const Eigen::Vector3f& end_p, const float& vel, const float& yaw, const float& vz);
        float calc_h_score(const Eigen::Vector3f& start_p, const Eigen::Vector3f& end_p);
        bool is_path_valid(const octomap::OcTree* ocmap, const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& next_pos);
};

#endif
