#ifndef ASTAR_H
#define ASTAR_H
#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <ros/ros.h>

using namespace std;

class AstarNode {
public:
  Eigen::Vector3f position_;
  int father_id;
  float f_score, g_score, h_score;

  AstarNode() { position_ = Eigen::Vector3f::Zero(); }
  AstarNode(const Eigen::Vector3f &position) { position_ = position; }
  // 用于unordered_map
  bool operator==(const AstarNode &n) const {
    int x0 = (int)(position_.x() / 0.1);
    int y0 = (int)(position_.y() / 0.1);
    int z0 = (int)(position_.z() / 0.1);

    int x1 = (int)(n.position_.x() / 0.1);
    int y1 = (int)(n.position_.y() / 0.1);
    int z1 = (int)(n.position_.z() / 0.1);

    return x0 == x1 && y0 == y1 && z0 == z1;
  }
};

// 用于优先队列
struct AstarNodeCmp {
  bool operator()(const AstarNode &lhs, const AstarNode &rhs) {
    return lhs.f_score > rhs.f_score;
  }
};

// 用于map
struct AstarMapCmp {
  bool operator()(const AstarNode &lhs, const AstarNode &rhs) {
    int x0 = (int)(lhs.position_.x() / 0.1);
    int y0 = (int)(lhs.position_.y() / 0.1);
    int z0 = (int)(lhs.position_.z() / 0.1);

    int x1 = (int)(rhs.position_.x() / 0.1);
    int y1 = (int)(rhs.position_.y() / 0.1);
    int z1 = (int)(rhs.position_.z() / 0.1);

    if (x0 != x1)
      return x0 < x1;
    if (y0 != y1)
      return y0 < y1;
    return z0 < z1;
  }
};

// 用于unordered_map
struct AstarNodeHash {
  size_t operator()(const AstarNode &node) const {
    // (x,y) max_range: [-100, 100], signed 10 bit
    // z max_range: [-5, 5], signed 7 bit
    // yaw max_range: [-12, 12], signed 5 bit
    // key = x  y   z
    // bit =  31-22   21-12   11-5
    int x0 = ((int)(node.position_.x() / 0.1)) & 0x3FF;
    x0 <<= 22;
    int y0 = ((int)(node.position_.y() / 0.1)) & 0x3FF;
    y0 <<= 12;
    int z0 = ((int)(node.position_.z() / 0.1)) & 0x7F;
    z0 <<= 5;

    int key = x0 | y0 | z0;
    return hash<int>()(key);
  }
};

class Astar {
public:
  float astar_path_distance(const octomap::OcTree *ocmap,
                            const Eigen::Vector3f &start_p,
                            const Eigen::Vector3f &end_p);
  float calc_h_score(const Eigen::Vector3f &start_p,
                     const Eigen::Vector3f &end_p);
  bool is_path_valid(const octomap::OcTree *ocmap,
                     const Eigen::Vector3f &cur_pos,
                     const Eigen::Vector3f &next_pos);
};

#endif
