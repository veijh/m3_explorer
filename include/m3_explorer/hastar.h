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
#include <functional>

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

        PathNode(){
            position = Eigen::Vector3f::Zero();
            father_acc = Eigen::Vector3f::Zero();
        }
        PathNode(const Eigen::Vector3f& _position, const float& _yaw, const float& _vel, const float& _vz)
            :position(_position), yaw(_yaw), vel(_vel), vz(_vz){};
        PathNode(const float& px, const float& py, const float& pz, const float& _yaw, const float& _vel, const float& _vz)
            :position(Eigen::Vector3f(px, py, pz)), yaw(_yaw), vel(_vel), vz(_vz){};

        bool operator==(const PathNode& n) const{
            int x0 = (int)(position.x()/0.1);
            int y0 = (int)(position.y()/0.1);

            int x1 = (int)(n.position.x()/0.1);
            int y1 = (int)(n.position.y()/0.1);

            int vx0 = (int)(vel*cos(yaw)/0.1);
            int vy0 = (int)(vel*sin(yaw)/0.1);

            int vx1 = (int)(n.vel*cos(n.yaw)/0.1);
            int vy1 = (int)(n.vel*sin(n.yaw)/0.1);

            return x0 == x1 && y0 && y1 && vx0 == vx1 && vy0 == vy1;
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
        // (x,y) max_range: [-100, 100), signed 11 bit
        // z max_range: [-5, 5), signed 10 bit
        // (vx,vy,vz) max_range: [-1.0, 1.0), signed 8 bit
        // key = x  y   z   vx  vy  vz
        // bit = 55-45 44-34 33-24 23-16 15-8 7-0
        long long x0 = (long long)(node.position.x()/0.1) & 0x7FF;
        x0 <<= 45;
        long long y0 = (long long)(node.position.y()/0.1) & 0x7FF;
        y0 <<= 34;
        long long z0 = (long long)(node.position.z()/0.1) & 0x3FF;
        z0 <<= 24;

        long long vx0 = (long long)(node.vel*cos(node.yaw)/0.1) & 0xFF;
        vx0 <<= 16;
        long long vy0 = (long long)(node.vel*sin(node.yaw)/0.1) & 0xFF;
        vy0 <<= 8;
        long long vz0 = (long long)(node.vz/0.1) & 0xFF;
        vz0 <<= 0;

        long long key = x0 | y0 | z0 | vx0 | vy0 | vz0;
        return hash<long long>()(key);
    }
};

class Traj{
    public:
        Eigen::Vector3f pos;
        Eigen::Vector3f vel;
        Eigen::Vector3f acc;
        float yaw;
        float yaw_rate;
};

class Hastar{
    private:
        bool trajectory_generate();
    public:
        float tau = 1.0;
        float traj_sample = 0.05;
        vector<Traj> traj;
        vector<PathNode> path;
        bool search_path(const octomap::OcTree* ocmap, const Eigen::Vector3f& start_p, const Eigen::Vector3f& end_p, const float& vel, const float& yaw, const float& vz);
        float calc_h_score(const Eigen::Vector3f& start_p, const Eigen::Vector3f& end_p);
        bool is_path_valid(const octomap::OcTree* ocmap, const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& next_pos);
};

#endif
