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

class RangeVoxel {
public:
  int min_ = 0;
  int max_ = 0;
  RangeVoxel() = default;
  RangeVoxel(const RangeVoxel &rhs) = default;
  RangeVoxel(const int min, const int max) : min_(min), max_(max) {}
};

class RangeVoxel2D {
public:
  int y_min_ = 0;
  int y_max_ = 0;
  int z_min_ = 0;
  int z_max_ = 0;
  RangeVoxel2D() = default;
  RangeVoxel2D(const RangeVoxel2D &rhs) = default;
  RangeVoxel2D(const int y_min, const int y_max, const int z_min,
               const int z_max)
      : y_min_(y_min), y_max_(y_max), z_min_(z_min), z_max_(z_max) {}
};

class RangeVoxel2DWapper {
public:
  RangeVoxel2D range_voxel_2d_;
  RangeVoxel plug_;
  RangeVoxel2DWapper() = default;
  RangeVoxel2DWapper(const RangeVoxel2DWapper &rhs) = default;
  RangeVoxel2DWapper(const RangeVoxel &plug) : plug_(plug) {}
  RangeVoxel2DWapper(const RangeVoxel2D &range_voxel_2d, const RangeVoxel &plug)
      : range_voxel_2d_(range_voxel_2d), plug_(plug) {}
  bool operator<(const RangeVoxel2DWapper &rhs) const {
    return plug_.min_ < rhs.plug_.min_;
  }
};

class RangeVoxel3D {
public:
  int x_min_ = 0;
  int x_max_ = 0;
  int y_min_ = 0;
  int y_max_ = 0;
  int z_min_ = 0;
  int z_max_ = 0;
  RangeVoxel3D() = default;
  RangeVoxel3D(const RangeVoxel3D &rhs) = default;
  RangeVoxel3D(const int x_min, const int x_max, const int y_min,
               const int y_max, const int z_min, const int z_max)
      : x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max),
        z_min_(z_min), z_max_(z_max) {}
};

class RangeVoxel3DWapper {
public:
  RangeVoxel3D range_voxel_3d_;
  RangeVoxel2D plug_;
  RangeVoxel3DWapper() = default;
  RangeVoxel3DWapper(const RangeVoxel3DWapper &rhs) = default;
  RangeVoxel3DWapper(const RangeVoxel2D &plug) : plug_(plug) {}
  RangeVoxel3DWapper(const RangeVoxel3D &range_voxel_2d,
                     const RangeVoxel2D &plug)
      : range_voxel_3d_(range_voxel_2d), plug_(plug) {}
  bool operator<(const RangeVoxel3DWapper &rhs) const {
    return plug_.y_min_ != rhs.plug_.y_min_ ? plug_.y_min_ < rhs.plug_.y_min_
                                            : plug_.z_min_ < rhs.plug_.z_min_;
  }
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
  std::vector<std::vector<std::vector<RangeVoxel>>> merge_map_;
  std::vector<std::vector<RangeVoxel2D>> merge_map_2d_;
  std::vector<RangeVoxel3D> merge_map_3d_;
  std::vector<std::shared_ptr<GridAstarNode>> path_;
  // Return the set of merged_voxels.
  std::vector<RangeVoxel2D>
  Merge2DVoxelAlongY(const std::vector<std::vector<RangeVoxel>> &yz_voxels);
  std::vector<RangeVoxel3D>
  Merge3DVoxelAlongX(const std::vector<std::vector<RangeVoxel2D>> &xyz_voxels);

public:
  const std::vector<std::vector<std::vector<GridState>>> &grid_map() const;
  const std::vector<std::vector<std::vector<RangeVoxel>>> &merge_map() const;
  const std::vector<std::vector<RangeVoxel2D>> &merge_map_2d() const;
  const std::vector<RangeVoxel3D> &merge_map_3d() const;
  const std::vector<std::shared_ptr<GridAstarNode>> &path() const;

  GridAstar(const float min_x, const float max_x, const float min_y,
            const float max_y, const float min_z, const float max_z,
            const float resolution = 0.1);
  void UpdateFromMap(const octomap::OcTree *ocmap,
                     const octomap::point3d &bbx_min,
                     const octomap::point3d &bbx_max);
  void MergeMap();
  void Merge2DVoxelAlongYUnitTest();
  void MergeMap2D();
  void Merge3DVoxelAlongXUnitTest();
  void MergeMap3D();
  float AstarPathDistance(const Eigen::Vector3f &start_p,
                          const Eigen::Vector3f &end_p);
  inline float CalHeurScore(const std::shared_ptr<GridAstarNode> &node,
                            const Eigen::Vector3f &end_p);
};

#endif
