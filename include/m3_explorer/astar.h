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
    int x0 = (int)(lhs.position_.x() / 0.2);
    int y0 = (int)(lhs.position_.y() / 0.2);
    int z0 = (int)(lhs.position_.z() / 0.2);

    int x1 = (int)(rhs.position_.x() / 0.2);
    int y1 = (int)(rhs.position_.y() / 0.2);
    int z1 = (int)(rhs.position_.z() / 0.2);

    if (x0 != x1)
      return x0 < x1;
    if (y0 != y1)
      return y0 < y1;
    return z0 < z1;
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
