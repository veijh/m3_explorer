#include "m3_explorer/grid_astar.h"
#include "m3_explorer/time_track.hpp"
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace {
constexpr float kFreeThreshold = 0.3;
constexpr float kOccThreshold = 0.7;
constexpr int kMapZSize = 8;
constexpr int kMapYZSize = 128;
constexpr int kMapXYZSize = 512;
constexpr int kMergeBuffer = 5;
constexpr int kFilterMinX = 4;
constexpr int kFilterMinY = 4;
constexpr int kFilterMinZ = 4;
constexpr int kConnectivityMinY = 4;
constexpr int kConnectivityMinZ = 4;
} // namespace

const std::vector<Eigen::Vector3i> expand_offset = {
    {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};

void GraphTable::AddNewEdge(const KeyBlock &src_key_block,
                            const KeyBlock &dest_key_block,
                            const float weight) {
  int src_id = nodes_.size();
  int dest_id = nodes_.size() + 1;
  nodes_.emplace_back(src_key_block);
  nodes_.emplace_back(dest_key_block);
  // Add edge between src and dest.
  nodes_[src_id].edges_.emplace_back(dest_id, weight);
  nodes_[dest_id].edges_.emplace_back(src_id, weight);
}

void GraphTable::AddEdgeBetweenExistingNode(const int src_id, const int dest_id,
                                            const float weight) {
  nodes_[src_id].edges_.emplace_back(dest_id, weight);
}

void GraphTable::UpdateEdgesInSameBlock() {
  std::unordered_map<int, std::shared_ptr<std::vector<int>>> block_group;
  const int num_nodes = nodes_.size();
  // Group nodes by block_id.
  for (int i = 0; i < num_nodes; ++i) {
    if (block_group.find(nodes_[i].key_block_.block_id_) == block_group.end()) {
      block_group[nodes_[i].key_block_.block_id_] =
          std::make_shared<std::vector<int>>();
      block_group[nodes_[i].key_block_.block_id_]->emplace_back(i);
    } else {
      block_group[nodes_[i].key_block_.block_id_]->emplace_back(i);
    }
  }
  // Update edges between nodes in the same block.
  for (auto it = block_group.begin(); it != block_group.end(); ++it) {
    const int group_size = it->second->size();
    for (int i = 0; i < group_size; ++i) {
      for (int j = 0; j < group_size; ++j) {
        if (i != j) {
          const int src_index = it->second->at(i);
          const int dest_index = it->second->at(j);
          const KeyBlock &src_key_block = nodes_[src_index].key_block_;
          const KeyBlock &dest_key_block = nodes_[dest_index].key_block_;
          const float delta_x = src_key_block.x_ - dest_key_block.x_;
          const float weight = src_key_block.block_.GetRoughDistance(
              dest_key_block.block_, delta_x);
          AddEdgeBetweenExistingNode(src_index, dest_index, weight);
        }
      }
    }
  }
}

GridAstarNode::GridAstarNode(const int index_x, const int index_y,
                             const int index_z)
    : index_x_(index_x), index_y_(index_y), index_z_(index_z) {}

GridAstar::GridAstar(const float min_x, const float max_x, const float min_y,
                     const float max_y, const float min_z, const float max_z,
                     const float resolution)
    : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y), min_z_(min_z),
      max_z_(max_z), resolution_(resolution) {
  const int num_x_grid =
      static_cast<int>(std::ceil((max_x_ - min_x_) / resolution_));
  const int num_y_grid =
      static_cast<int>(std::ceil((max_y_ - min_y_) / resolution_));
  const int num_z_grid =
      static_cast<int>(std::ceil((max_z_ - min_z_) / resolution_));
  // grid_map_[i][j][k] indicates the occupancy of node in
  // min_x_ + i * resolution -> min_x_ + (i + 1) * resolution
  // min_y_ + j * resolution -> min_y_ + (j + 1) * resolution
  // min_z_ + k * resolution -> min_z_ + (k + 1) * resolution
  grid_map_.resize(
      num_x_grid,
      std::vector<std::vector<GridState>>(
          num_y_grid, std::vector<GridState>(num_z_grid, GridState::kUnknown)));
}

const std::vector<std::vector<std::vector<GridAstar::GridState>>> &
GridAstar::grid_map() const {
  return grid_map_;
}

const std::vector<std::vector<std::vector<RangeVoxel>>> &
GridAstar::merge_map() const {
  return merge_map_;
}

const std::vector<std::vector<Block2D>> &GridAstar::merge_map_2d() const {
  return merge_map_2d_;
}

const std::vector<Block3D> &GridAstar::merge_map_3d() const {
  return merge_map_3d_;
}

const std::vector<std::shared_ptr<GridAstarNode>> &GridAstar::path() const {
  return path_;
}

const std::vector<int> &GridAstar::block_path() const { return block_path_; }

const GraphTable &GridAstar::graph_table() const { return graph_table_; }

void GridAstar::UpdateFromMap(const octomap::OcTree *ocmap,
                              const octomap::point3d &bbx_min,
                              const octomap::point3d &bbx_max) {
  if (ocmap == nullptr)
    return;

  const int num_x_grid = grid_map_.size();
  const int num_y_grid = grid_map_[0].size();
  const int num_z_grid = grid_map_[0][0].size();
  for (octomap::OcTree::leaf_bbx_iterator
           it = ocmap->begin_leafs_bbx(bbx_min, bbx_max),
           end = ocmap->end_leafs_bbx();
       it != end; ++it) {
    float occ_probility = it->getOccupancy();
    // Do not update the unknown node.
    if (occ_probility >= kFreeThreshold && occ_probility <= kOccThreshold) {
      continue;
    }

    GridState grid_state =
        occ_probility < (kFreeThreshold + kOccThreshold) * 0.5
            ? GridState::kFree
            : GridState::kOcc;
    const float size = static_cast<float>(it.getSize());
    const octomap::point3d center = it.getCoordinate();
    const float node_min_x = center.x() - size * 0.5;
    const float node_max_x = center.x() + size * 0.5;
    const float node_min_y = center.y() - size * 0.5;
    const float node_max_y = center.y() + size * 0.5;
    const float node_min_z = center.z() - size * 0.5;
    const float node_max_z = center.z() + size * 0.5;

    int min_x_index =
        static_cast<int>(std::floor((node_min_x - min_x_) / resolution_));
    int max_x_index =
        static_cast<int>(std::ceil((node_max_x - min_x_) / resolution_));
    int min_y_index =
        static_cast<int>(std::floor((node_min_y - min_y_) / resolution_));
    int max_y_index =
        static_cast<int>(std::ceil((node_max_y - min_y_) / resolution_));
    int min_z_index =
        static_cast<int>(std::floor((node_min_z - min_z_) / resolution_));
    int max_z_index =
        static_cast<int>(std::ceil((node_max_z - min_z_) / resolution_));

    min_x_index = std::clamp(min_x_index, 0, num_x_grid - 1);
    max_x_index = std::clamp(max_x_index, 0, num_x_grid - 1);
    min_y_index = std::clamp(min_y_index, 0, num_y_grid - 1);
    max_y_index = std::clamp(max_y_index, 0, num_y_grid - 1);
    min_z_index = std::clamp(min_z_index, 0, num_z_grid - 1);
    max_z_index = std::clamp(max_z_index, 0, num_z_grid - 1);

    for (int i = min_x_index; i <= max_x_index; ++i) {
      for (int j = min_y_index; j <= max_y_index; ++j) {
        for (int k = min_z_index; k <= max_z_index; ++k) {
          grid_map_[i][j][k] = grid_state;
        }
      }
    }
  }
}

void GridAstar::MergeMap() {
  const int num_x_grid = grid_map_.size();
  const int num_y_grid = grid_map_[0].size();
  const int num_z_grid = grid_map_[0][0].size();
  merge_map_.resize(num_x_grid,
                    std::vector<std::vector<RangeVoxel>>(num_y_grid));
  int total_num = 0;
  for (int i = 0; i < num_x_grid; ++i) {
    for (int j = 0; j < num_y_grid; ++j) {
      int min = 0;
      int max = num_z_grid - 1;
      int state = 0;
      merge_map_[i][j].clear();
      merge_map_[i][j].reserve(kMapZSize);
      for (int k = 0; k < num_z_grid; ++k) {
        switch (state) {
        case 0:
          if (grid_map_[i][j][k] == GridState::kFree) {
            min = k;
            state = 1;
          }
          break;
        case 1:
          if (k == num_z_grid - 1 || grid_map_[i][j][k] != GridState::kFree) {
            max = grid_map_[i][j][k] != GridState::kFree ? k - 1 : k;
            // Filter narrow range.
            if (max - min + 1 > kFilterMinZ) {
              merge_map_[i][j].emplace_back(min, max);
              ++total_num;
            }
            state = 0;
          }
          break;
        }
      }
    }
  }
  std::cout << "Voxel1D total voxel num: " << total_num << std::endl;
}

std::vector<Block2D> GridAstar::Merge2DVoxelAlongY(
    const std::vector<std::vector<RangeVoxel>> &yz_voxels) {
  const int num_y_voxels = yz_voxels.size();
  std::set<Block2DWrapper> unmerged_voxels;
  std::vector<Block2D> merge_results;
  merge_results.reserve(kMapYZSize);
  for (int i = 0; i < num_y_voxels; ++i) {
    std::set<Block2DWrapper> merged_voxels;
    const int num_z_voxels = yz_voxels[i].size();
    for (int j = 0; j < num_z_voxels; ++j) {
      const RangeVoxel new_z_voxel = yz_voxels[i][j];
      // Find the range to merge in unmerged_voxels.
      const int voxel_min_lb = new_z_voxel.min_ - kMergeBuffer + 1;
      const int voxel_max_lb = new_z_voxel.min_ + kMergeBuffer - 1;
      // Binary search.
      auto range_lower_it = std::lower_bound(
          unmerged_voxels.begin(), unmerged_voxels.end(),
          Block2DWrapper(RangeVoxel(voxel_min_lb, new_z_voxel.max_)));
      auto range_upper_it = std::upper_bound(
          unmerged_voxels.begin(), unmerged_voxels.end(),
          Block2DWrapper(RangeVoxel(voxel_max_lb, new_z_voxel.max_)));
      if (std::distance(range_lower_it, range_upper_it) >= 1) {
        const int voxel_min_ub = new_z_voxel.max_ - kMergeBuffer + 1;
        const int voxel_max_ub = new_z_voxel.max_ + kMergeBuffer - 1;
        const int plug_max = range_lower_it->plug_.max_;
        if (voxel_min_ub <= plug_max && plug_max <= voxel_max_ub) {
          // Merge and add to the result.
          Block2DWrapper new_voxel_wrapper = *range_lower_it;
          new_voxel_wrapper.plug_ = new_z_voxel;
          new_voxel_wrapper.block_2d_.EmplaceRangeBack(new_z_voxel);
          merged_voxels.insert(new_voxel_wrapper);
          // Erase the range.
          unmerged_voxels.erase(range_lower_it);
        } else {
          const Block2D new_voxel(i, new_z_voxel);
          const Block2DWrapper new_voxel_wrapper(new_voxel, new_z_voxel);
          merged_voxels.insert(new_voxel_wrapper);
        }
      } else {
        const Block2D new_voxel(i, new_z_voxel);
        const Block2DWrapper new_voxel_wrapper(new_voxel, new_z_voxel);
        merged_voxels.insert(new_voxel_wrapper);
      }
    }
    // Add all item of unmerged_voxels to result.
    for (auto &item : unmerged_voxels) {
      // Filter small voxels.
      if (item.block_2d_.y_max_ - item.block_2d_.y_min_ + 1 <= kFilterMinY) {
        continue;
      }
      merge_results.emplace_back(item.block_2d_);
    }
    unmerged_voxels = merged_voxels;
  }
  // Add all item of unmerged_voxels to result.
  for (auto &item : unmerged_voxels) {
    // Filter small voxels.
    if (item.block_2d_.y_max_ - item.block_2d_.y_min_ + 1 <= kFilterMinY) {
      continue;
    }
    merge_results.emplace_back(item.block_2d_);
  }
  return merge_results;
}

void GridAstar::Merge2DVoxelAlongYUnitTest() {
  std::vector<std::vector<RangeVoxel>> yz_voxels;
  yz_voxels.resize(7);
  yz_voxels[0].emplace_back(RangeVoxel(0, 2));
  yz_voxels[0].emplace_back(RangeVoxel(5, 6));
  yz_voxels[1].emplace_back(RangeVoxel(0, 1));
  yz_voxels[1].emplace_back(RangeVoxel(4, 6));
  yz_voxels[2].emplace_back(RangeVoxel(0, 1));
  yz_voxels[2].emplace_back(RangeVoxel(3, 6));
  yz_voxels[4].emplace_back(RangeVoxel(0, 1));
  yz_voxels[4].emplace_back(RangeVoxel(3, 3));
  yz_voxels[4].emplace_back(RangeVoxel(5, 6));
  yz_voxels[5].emplace_back(RangeVoxel(0, 1));
  yz_voxels[5].emplace_back(RangeVoxel(3, 6));
  yz_voxels[6].emplace_back(RangeVoxel(0, 6));
  std::vector<Block2D> result = Merge2DVoxelAlongY(yz_voxels);
  std::cout << "[Result Size]: " << result.size() << std::endl;
  for (auto item : result) {
    std::cout << "[Item]: (" << item.y_min_ << ", " << item.y_max_ << "), ("
              << item.z_min_ << ", " << item.z_max_ << ")" << std::endl;
  }
}

void GridAstar::MergeMap2D() {
  auto cmp = [](const Block2D &lhs, const Block2D &rhs) {
    return lhs.y_min_ != rhs.y_min_ ? lhs.y_min_ < rhs.y_min_
                                    : lhs.z_min_ < rhs.z_min_;
  };
  const int num_x_voxel = merge_map_.size();
  merge_map_2d_.resize(num_x_voxel);
  int total_num = 0;
  for (int i = 0; i < num_x_voxel; ++i) {
    merge_map_2d_[i] = Merge2DVoxelAlongY(merge_map_[i]);
    std::sort(merge_map_2d_[i].begin(), merge_map_2d_[i].end(), cmp);
    total_num += merge_map_2d_[i].size();
  }
  std::cout << "total_num: " << total_num << std::endl;
}

std::vector<Block3D> GridAstar::Merge3DVoxelAlongX(
    const std::vector<std::vector<Block2D>> &xyz_voxels) {
  const int num_x_voxels = xyz_voxels.size();
  int block_id = 0;
  GraphTable graph_table;
  std::vector<Block3D> merge_results;
  merge_results.reserve(kMapXYZSize);
  std::set<Block3DWrapper> unmerged_voxels;
  for (int i = 0; i < num_x_voxels; ++i) {
    std::set<Block3DWrapper> merged_voxels;
    const int num_yz_voxels = xyz_voxels[i].size();
    for (int j = 0; j < num_yz_voxels; ++j) {
      const Block2D new_yz_voxel = xyz_voxels[i][j];
      // Find the range to merge in unmerged_voxels.
      const int voxel_y_min_lb = new_yz_voxel.y_min_ - kMergeBuffer + 1;
      const int voxel_y_min_ub = new_yz_voxel.y_min_ + kMergeBuffer - 1;
      const int voxel_z_min_lb = new_yz_voxel.z_min_ - kMergeBuffer + 1;
      const int voxel_z_min_ub = new_yz_voxel.z_min_ + kMergeBuffer - 1;
      const int voxel_y_max_lb = new_yz_voxel.y_max_ - kMergeBuffer + 1;
      const int voxel_y_max_ub = new_yz_voxel.y_max_ + kMergeBuffer - 1;
      const int voxel_z_max_lb = new_yz_voxel.z_max_ - kMergeBuffer + 1;
      const int voxel_z_max_ub = new_yz_voxel.z_max_ + kMergeBuffer - 1;

      bool is_merged = false;
      bool is_connected = false;
      for (auto it = unmerged_voxels.begin(); it != unmerged_voxels.end();) {
        const int plug_y_min = it->plug_.y_min_;
        const int plug_z_min = it->plug_.z_min_;
        const int plug_y_max = it->plug_.y_max_;
        const int plug_z_max = it->plug_.z_max_;
        if (voxel_y_min_lb <= plug_y_min && plug_y_min <= voxel_y_min_ub &&
            voxel_z_min_lb <= plug_z_min && plug_z_min <= voxel_z_min_ub &&
            voxel_y_max_lb <= plug_y_max && plug_y_max <= voxel_y_max_ub &&
            voxel_z_max_lb <= plug_z_max && plug_z_max <= voxel_z_max_ub) {
          // Merge and add to the result.
          Block3DWrapper new_voxel_wrapper = *it;
          new_voxel_wrapper.plug_ = new_yz_voxel;
          new_voxel_wrapper.block_3d_.EmplaceBlockBack(new_yz_voxel);
          merged_voxels.insert(new_voxel_wrapper);
          // Erase the range.
          unmerged_voxels.erase(it++);
          is_merged = true;
          break;
        }
        const std::vector<Block2D> overlap_blocks =
            new_yz_voxel.GetOverlap(it->plug_);
        for (const Block2D &overlap_block : overlap_blocks) {
          const int overlap_y_min = overlap_block.y_min_;
          const int overlap_z_min = overlap_block.z_min_;
          const int overlap_y_max = overlap_block.y_max_;
          const int overlap_z_max = overlap_block.z_max_;
          if (overlap_y_max - overlap_y_min + 1 >= kConnectivityMinY &&
              overlap_z_max - overlap_z_min + 1 >= kConnectivityMinZ) {
            // If two voxels are partially overlapped, add a new edge to the
            // graph.
            const int key_block_x = i;
            const KeyBlock last_key_block(
                key_block_x - 1, it->block_3d_.block_id_, overlap_block);
            const KeyBlock key_block(key_block_x, block_id, overlap_block);
            graph_table.AddNewEdge(last_key_block, key_block);
            break;
          }
        }
        it++;
      }
      if (!is_merged) {
        const Block3D new_voxel(i, new_yz_voxel);
        const Block3DWrapper new_voxel_wrapper(new_voxel, new_yz_voxel,
                                               block_id);
        // Update block id of next voxel.
        ++block_id;
        merged_voxels.insert(new_voxel_wrapper);
      }
    }
    // Add all item of unmerged_voxels to result.
    for (auto &item : unmerged_voxels) {
      merge_results.emplace_back(item.block_3d_);
    }
    unmerged_voxels = merged_voxels;
  }
  // Add all item of unmerged_voxels to result.
  for (auto &item : unmerged_voxels) {
    merge_results.emplace_back(item.block_3d_);
  }
  // Update topology of the graph.
  graph_table.UpdateEdgesInSameBlock();
  graph_table_ = graph_table;
  return merge_results;
}

void GridAstar::Merge3DVoxelAlongXUnitTest() {
  std::vector<std::vector<Block2D>> xyz_voxels;
  xyz_voxels.resize(7);
  xyz_voxels[0].emplace_back(Block2D(0, 4, 0, 4));
  xyz_voxels[0].emplace_back(Block2D(5, 10, 0, 5));
  xyz_voxels[1].emplace_back(Block2D(0, 4, 0, 4));
  xyz_voxels[1].emplace_back(Block2D(5, 10, 0, 4));
  xyz_voxels[2].emplace_back(Block2D(0, 4, 0, 4));
  xyz_voxels[2].emplace_back(Block2D(5, 10, 0, 3));
  xyz_voxels[4].emplace_back(Block2D(0, 4, 0, 4));
  xyz_voxels[5].emplace_back(Block2D(0, 4, 0, 4));
  xyz_voxels[6].emplace_back(Block2D(0, 4, 0, 4));
  std::vector<Block3D> result = Merge3DVoxelAlongX(xyz_voxels);
  std::cout << "[Result Size]: " << result.size() << std::endl;
  for (auto item : result) {
    std::cout << "[Item]: (" << item.x_min_ << ", " << item.x_max_ << "), ("
              << item.y_min_ << ", " << item.y_max_ << "), (" << item.z_min_
              << ", " << item.z_max_ << ")" << std::endl;
  }
}

void GridAstar::MergeMap3D() {
  merge_map_3d_ = Merge3DVoxelAlongX(merge_map_2d_);
  std::cout << "total_num: " << merge_map_3d_.size() << std::endl;
}

float GridAstar::AstarPathDistance(const Eigen::Vector3f &start_p,
                                   const Eigen::Vector3f &end_p) {
  TimeTrack tracker;
  const int num_x_grid = grid_map_.size();
  const int num_y_grid = grid_map_[0].size();
  const int num_z_grid = grid_map_[0][0].size();

  std::priority_queue<std::shared_ptr<GridAstarNode>,
                      std::vector<std::shared_ptr<GridAstarNode>>,
                      GridAstarNodeCmp>
      astar_q;

  std::unordered_map<GridKey, GridInfo, GridHash> grid_info;

  bool is_path_found = false;
  int count = 0;

  int index_start_x =
      static_cast<int>(std::floor((start_p.x() - min_x_) / resolution_));
  int index_start_y =
      static_cast<int>(std::floor((start_p.y() - min_y_) / resolution_));
  int index_start_z =
      static_cast<int>(std::floor((start_p.z() - min_z_) / resolution_));
  int index_end_x =
      static_cast<int>(std::floor((end_p.x() - min_x_) / resolution_));
  int index_end_y =
      static_cast<int>(std::floor((end_p.y() - min_y_) / resolution_));
  int index_end_z =
      static_cast<int>(std::floor((end_p.z() - min_z_) / resolution_));

  std::shared_ptr<GridAstarNode> astar_start = std::make_shared<GridAstarNode>(
      index_start_x, index_start_y, index_start_z);
  astar_start->father_node_ = nullptr;
  const GridKey start_key(index_start_x, index_start_y, index_start_z);
  grid_info[start_key].g_score_ = 0.0;
  grid_info[start_key].h_score_ = CalHeurScore(astar_start, end_p);
  astar_start->f_score_ =
      grid_info[start_key].g_score_ + grid_info[start_key].h_score_;
  grid_info[start_key].state_ = GridInfo::AstarState::kOpen;
  astar_q.push(astar_start);

  std::shared_ptr<GridAstarNode> astar_end = nullptr;
  tracker.OutputPassingTime("Astar Init");
  tracker.SetStartTime();

  while (!astar_q.empty()) {
    ++count;
    std::shared_ptr<GridAstarNode> cur_node = astar_q.top();
    astar_q.pop();

    const int cur_index_x = cur_node->index_x_;
    const int cur_index_y = cur_node->index_y_;
    const int cur_index_z = cur_node->index_z_;
    const GridKey cur_key(cur_index_x, cur_index_y, cur_index_z);

    // Skip the same node due to the update of g_score.
    if (grid_info[cur_key].state_ == GridInfo::AstarState::kClose) {
      continue;
    }
    // set node to closed
    grid_info[cur_key].state_ = GridInfo::AstarState::kClose;
    if (cur_node->index_x_ == index_end_x &&
        cur_node->index_y_ == index_end_y &&
        cur_node->index_z_ == index_end_z) {
      is_path_found = true;
      astar_end = cur_node;
      break;
    }

    // expand neighbor nodes
    for (auto &offset : expand_offset) {
      int next_index_x = offset.x() + cur_node->index_x_;
      int next_index_y = offset.y() + cur_node->index_y_;
      int next_index_z = offset.z() + cur_node->index_z_;
      const GridKey next_key(next_index_x, next_index_y, next_index_z);
      // skip nodes that is out of range
      if (next_index_x < 0 || next_index_y < 0 || next_index_z < 0) {
        continue;
      }
      if (next_index_x >= num_x_grid || next_index_y >= num_y_grid ||
          next_index_z >= num_z_grid) {
        continue;
      }
      // only expand free nodes
      if (grid_map_[next_index_x][next_index_y][next_index_z] ==
              GridAstar::GridState::kFree &&
          grid_info[next_key].state_ != GridInfo::AstarState::kClose) {
        // Calculate GScore.
        const float new_g_score = grid_info[cur_key].g_score_ + resolution_;
        // Next node has not being visited.
        if (grid_info[next_key].state_ == GridInfo::AstarState::kNull) {
          std::shared_ptr<GridAstarNode> next_node =
              std::make_shared<GridAstarNode>(next_index_x, next_index_y,
                                              next_index_z);
          next_node->father_node_ = cur_node;
          grid_info[next_key].g_score_ = new_g_score;
          grid_info[next_key].h_score_ = CalHeurScore(next_node, end_p);
          // Calculate Fscore.
          next_node->f_score_ = new_g_score + grid_info[next_key].h_score_;
          grid_info[next_key].state_ = GridInfo::AstarState::kOpen;
          astar_q.push(next_node);
        } else {
          // Next node has being visited.
          std::shared_ptr<GridAstarNode> next_node =
              std::make_shared<GridAstarNode>(next_index_x, next_index_y,
                                              next_index_z);
          if (new_g_score < grid_info[next_key].g_score_) {
            next_node->father_node_ = cur_node;
            // Update Gscore.
            grid_info[next_key].g_score_ = new_g_score;
            const float h_score = grid_info[next_key].h_score_;
            // Update Fscore.
            next_node->f_score_ = new_g_score + h_score;
            astar_q.push(next_node);
          }
        }
      }
    }
  }
  tracker.OutputPassingTime("Astar Done");
  tracker.SetStartTime();

  if (is_path_found) {
    float distance = 0.0;
    path_.clear();
    // Insert last node.
    path_.emplace_back(astar_end);
    std::shared_ptr<GridAstarNode> last_path_node = astar_end;
    std::shared_ptr<GridAstarNode> path_node = astar_end->father_node_;
    while (path_node != nullptr) {
      const int delta_x = last_path_node->index_x_ - path_node->index_x_;
      const int delta_y = last_path_node->index_y_ - path_node->index_y_;
      const int delta_z = last_path_node->index_z_ - path_node->index_z_;
      distance += resolution_ *
                  (std::abs(delta_x) + std::abs(delta_y) + std::abs(delta_z));
      path_.emplace_back(path_node);
      last_path_node = path_node;
      path_node = path_node->father_node_;
    }
    reverse(path_.begin(), path_.end());
    std::cout << "[Astar] waypoint generated!! waypoint num: " << path_.size()
              << ", select node num: " << count << std::endl;
    tracker.OutputPassingTime("Astar Output");
    return distance;
  } else {
    std::cout << "[WARNING] no path !! from " << std::endl
              << start_p << std::endl
              << "to " << std::endl
              << end_p << std::endl;
    return (end_p - start_p).norm();
  }
}

float GridAstar::BlockPathDistance(const Eigen::Vector3f &start_p,
                                   const Eigen::Vector3f &end_p) {
  int index_start_x =
      static_cast<int>(std::floor((start_p.x() - min_x_) / resolution_));
  int index_start_y =
      static_cast<int>(std::floor((start_p.y() - min_y_) / resolution_));
  int index_start_z =
      static_cast<int>(std::floor((start_p.z() - min_z_) / resolution_));
  int index_end_x =
      static_cast<int>(std::floor((end_p.x() - min_x_) / resolution_));
  int index_end_y =
      static_cast<int>(std::floor((end_p.y() - min_y_) / resolution_));
  int index_end_z =
      static_cast<int>(std::floor((end_p.z() - min_z_) / resolution_));
  // Find the block that contains the start point.
  int num_blocks = merge_map_3d_.size();
  bool is_start_locked = false;
  bool is_end_locked = false;
  int start_block_index = -1;
  int end_block_index = -1;
  for (int i = 0; i < num_blocks; ++i) {
    if (is_start_locked && is_end_locked) {
      break;
    }
    if (!is_start_locked && merge_map_3d_[i].IsInBlock(
                                index_start_x, index_start_y, index_start_z)) {
      start_block_index = merge_map_3d_[i].block_id_;
      is_start_locked = true;
    }
    if (!is_end_locked &&
        merge_map_3d_[i].IsInBlock(index_end_x, index_end_y, index_end_z)) {
      end_block_index = merge_map_3d_[i].block_id_;
      is_end_locked = true;
    }
  }
  if (start_block_index == end_block_index) {
    // TODO: Calculate the distance between two points in the same block.
    return (end_p - start_p).norm();
  } else {
    // Dijkstra algorithm to find the path between two blocks.
    const int nums_node = graph_table_.nodes_.size();
    std::vector<int> father_nodes(nums_node, -1);
    int end_father = -1;
    std::vector<int> is_visited(nums_node, 0);
    int is_end_visited = 0;
    std::vector<float> distance(nums_node, 0.0);
    float end_distance = 0.0;
    std::priority_queue<std::pair<int, float>,
                        std::vector<std::pair<int, float>>, DijkstraNodeCmp>
        dijkstra_q;
    // Add the key block of start block to the queue.
    for (int i = 0; i < nums_node; ++i) {
      const KeyBlock key_block = graph_table_.nodes_[i].key_block_;
      if (key_block.block_id_ == start_block_index) {
        // Compute the rough distance between the start point and the key block.
        const float delta_x = index_start_x - key_block.x_;
        const float delta_y = index_start_y - 0.5 * (key_block.block_.y_min_ +
                                                     key_block.block_.y_max_);
        const float delta_z = index_start_z - 0.5 * (key_block.block_.z_min_ +
                                                     key_block.block_.z_max_);
        const float edge_weight = std::hypot(delta_x, delta_y, delta_z);
        dijkstra_q.emplace(i, edge_weight);
        is_visited[i] = 1;
        father_nodes[i] = -1;
        distance[i] = edge_weight;
      }
    }
    bool is_path_found = false;
    while (!dijkstra_q.empty()) {
      // Select the node with the smallest distance.
      std::pair<int, float> cur_node = dijkstra_q.top();
      dijkstra_q.pop();
      // Check if the node is the end block.
      if (cur_node.first == -1) {
        is_path_found = true;
        break;
      }
      // Skip nodes that is in Closed state.
      if (is_visited[cur_node.first] == 2) {
        continue;
      }
      // Set the node to Closed state.
      is_visited[cur_node.first] = 2;
      const GraphNode graph_node = graph_table_.nodes_[cur_node.first];
      // Check if the node is in the end block.
      if (graph_node.key_block_.block_id_ == end_block_index) {
        const float delta_x = index_end_x - graph_node.key_block_.x_;
        const float delta_y =
            index_end_y - 0.5 * (graph_node.key_block_.block_.y_min_ +
                                 graph_node.key_block_.block_.y_max_);
        const float delta_z =
            index_end_z - 0.5 * (graph_node.key_block_.block_.z_min_ +
                                 graph_node.key_block_.block_.z_max_);
        const float edge_weight =
            std::hypot(delta_x, delta_y, delta_z) + cur_node.second;
        if (is_end_visited == 0 ||
            (is_end_visited == 1 && edge_weight < end_distance)) {
          end_father = cur_node.first;
          is_end_visited = 1;
          end_distance = edge_weight;
          dijkstra_q.emplace(-1, edge_weight);
        }
      }
      // Expand the node.
      const int num_edges = graph_node.edges_.size();
      for (int i = 0; i < num_edges; ++i) {
        const int neighbor_index = graph_node.edges_[i].dest_id_;
        const float edge_weight =
            graph_node.edges_[i].weight_ + cur_node.second;
        if (is_visited[neighbor_index] == 0 ||
            (is_visited[neighbor_index] == 1 &&
             edge_weight < distance[neighbor_index])) {
          father_nodes[neighbor_index] = cur_node.first;
          is_visited[neighbor_index] = 1;
          distance[neighbor_index] = edge_weight;
          dijkstra_q.emplace(neighbor_index, edge_weight);
        }
      }
    }
    if (is_path_found) {
      std::vector<int> path_blocks;
      int cur_node_index = end_father;
      while (cur_node_index != -1) {
        path_blocks.emplace_back(cur_node_index);
        cur_node_index = father_nodes[cur_node_index];
      }
      std::cout << "[Block Astar] waypoint generated!! waypoint num: "
                << path_blocks.size() << std::endl;
      block_path_ = path_blocks;
      return end_distance;
    } else {
      std::cout << "[WARNING] no path !! from " << std::endl
                << start_p << std::endl
                << "to " << std::endl
                << end_p << std::endl;
      return (end_p - start_p).norm();
    }
  }
}

inline float GridAstar::CalHeurScore(const std::shared_ptr<GridAstarNode> &node,
                                     const Eigen::Vector3f &end_p) {
  float delta_x = end_p.x() - (node->index_x_ * resolution_ + min_x_);
  float delta_y = end_p.y() - (node->index_y_ * resolution_ + min_y_);
  float delta_z = end_p.z() - (node->index_z_ * resolution_ + min_z_);
  return std::abs(delta_x) + std::abs(delta_y) + 10.0 * std::abs(delta_z);
}
