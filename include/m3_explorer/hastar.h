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
#include <string>
#include <unordered_map>

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

        bool operator==(const PathNode& n) const{
            int x0 = (int)(position.x()/0.1);
            int y0 = (int)(position.y()/0.1);
            int z0 = (int)(position.z()/0.1);

            int x1 = (int)(n.position.x()/0.1);
            int y1 = (int)(n.position.y()/0.1);
            int z1 = (int)(n.position.z()/0.1);

            int vx0 = (int)(vel*cos(yaw)/0.1);
            int vy0 = (int)(vel*sin(yaw)/0.1);
            int vz0 = (int)(vz/0.1);

            int vx1 = (int)(n.vel*cos(n.yaw)/0.1);
            int vy1 = (int)(n.vel*sin(n.yaw)/0.1);
            int vz1 = (int)(n.vz/0.1);

            return x0 == x1 && y0 && y1 && z0 == z1 && vx0 == vx1 && vy0 == vy1 && vz0 == vz1;
        }
};

struct NodeCmp{
    bool operator()(const PathNode& lhs, const PathNode& rhs){
        return lhs.f_score > rhs.f_score;
    }
};

struct MapCmp{
    bool operator()(const PathNode& lhs, const PathNode& rhs){
        int x0 = (int)(lhs.position.x()/0.1);
        int y0 = (int)(lhs.position.y()/0.1);
        int z0 = (int)(lhs.position.z()/0.1);

        int x1 = (int)(rhs.position.x()/0.1);
        int y1 = (int)(rhs.position.y()/0.1);
        int z1 = (int)(rhs.position.z()/0.1);

        int vx0 = (int)(lhs.vel*cos(lhs.yaw)/0.1);
        int vy0 = (int)(lhs.vel*sin(lhs.yaw)/0.1);
        int vz0 = (int)(lhs.vz/0.1);

        int vx1 = (int)(rhs.vel*cos(rhs.yaw)/0.1);
        int vy1 = (int)(rhs.vel*sin(rhs.yaw)/0.1);
        int vz1 = (int)(rhs.vz/0.1);

        if(x0 != x1) return x0 < x1;
        if(y0 != y1) return y0 < y1;
        if(z0 != z1) return z0 < z1;
        if(vx0 != vx1) return vx0 < vx1;
        if(vy0 != vy1)  return vy0 < vy1;
        return vz0 < vz1;
    }
};

struct NodeHash{
    size_t operator()(const PathNode& node) const{
        int x0 = (int)(node.position.x()/0.1) + 300;
        int y0 = (int)(node.position.y()/0.1) + 300;

        int vx0 = (int)(node.vel*cos(node.yaw)/0.1) + 15;
        int vy0 = (int)(node.vel*sin(node.yaw)/0.1) + 15;

        int hash_num = x0 * 400 * 400 * 20 + y0 * 400 * 20 + vx0 * 20 + vy0;
        return static_cast<size_t>(hash_num);
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
