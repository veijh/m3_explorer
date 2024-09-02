#ifndef GRID_ASTAR_H
#define GRID_ASTAR_H
#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <ros/ros.h>
#include <vector>

class GridAstarNode {
public:
  int index_x_;
  int index_y_;
  int index_z_;
  float f_score_ = 0.0;
  std::shared_ptr<GridAstarNode> father_node_ = nullptr;

public:
  GridAstarNode(const int index_x, const int index_y, const int index_z);
};

// 用于优先队列
struct GridAstarNodeCmp {
  bool operator()(const std::shared_ptr<GridAstarNode> &lhs,
                  const std::shared_ptr<GridAstarNode> &rhs) const {
    return lhs->f_score_ > rhs->f_score_;
  }
};

class GridKey {
public:
  int index_x_;
  int index_y_;
  int index_z_;

public:
  GridKey(const int index_x, const int index_y, const int index_z)
      : index_x_(index_x), index_y_(index_y), index_z_(index_z) {}
  bool operator==(const GridKey &key) const {
    return index_x_ == key.index_x_ && index_y_ == key.index_y_ &&
           index_z_ == key.index_z_;
  }
};

// 用于unordered_map
struct GridHash {
  size_t operator()(const GridKey &key) const {
    // x max_range: [-50, 50], 10 bit
    // y max_range: [-50, 50], 10 bit
    // z max_range: [0, 3], 5 bit
    // key = x  y   z
    // bit = 24-15   14-5    4-0
    const int x0 = key.index_x_ << 15;
    const int y0 = key.index_y_ << 5;
    const int z0 = key.index_z_ << 0;
    const int hash_value = x0 | y0 | z0;
    return std::hash<int>()(hash_value);
  }
};

class GridInfo {
public:
  enum class AstarState { kNull = 0, kOpen, kClose };
  AstarState state_ = AstarState::kNull;
  float g_score_ = 0.0;
  float h_score_ = 0.0;
};

class GridAstar {
public:
  enum class GridState { kFree = 0, kUnknown, kOcc };

private:
  float min_x_ = -50.0;
  float max_x_ = 50.0;
  float min_y_ = -50.0;
  float max_y_ = 50.0;
  float min_z_ = 0.0;
  float max_z_ = 2.5;
  float resolution_ = 0.1;
  std::vector<std::vector<std::vector<GridState>>> grid_map_;
  std::vector<std::shared_ptr<GridAstarNode>> path_;

public:
  const std::vector<std::vector<std::vector<GridState>>> &grid_map() const;
  const std::vector<std::shared_ptr<GridAstarNode>> &path() const;

  GridAstar(const float min_x, const float max_x, const float min_y,
            const float max_y, const float min_z, const float max_z,
            const float resolution = 0.1);
  void UpdateFromMap(const octomap::OcTree *ocmap,
                     const octomap::point3d &bbx_min,
                     const octomap::point3d &bbx_max);
  float AstarPathDistance(const Eigen::Vector3f &start_p,
                          const Eigen::Vector3f &end_p);
  inline float CalHeurScore(const std::shared_ptr<GridAstarNode> &node,
                            const Eigen::Vector3f &end_p);
};

#endif
