#include "m3_explorer/grid_astar.h"
#include "m3_explorer/time_track.hpp"
#include <algorithm>
#include <queue>
#include <unordered_map>

namespace {
constexpr float kFreeThreshold = 0.2;
constexpr float kOccThreshold = 0.8;
} // namespace

const std::vector<Eigen::Vector3i> expand_offset = {
    {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};

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

const std::vector<std::shared_ptr<GridAstarNode>> &GridAstar::path() const {
  return path_;
}

void GridAstar::UpdateFromMap(const octomap::OcTree *ocmap,
                              const octomap::point3d &bbx_min,
                              const octomap::point3d &bbx_max) {
  if (ocmap == nullptr)
    return;
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

    const int num_x_grid = grid_map_.size();
    const int num_y_grid = grid_map_[0].size();
    const int num_z_grid = grid_map_[0][0].size();

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

float GridAstar::AstarPathDistance(const Eigen::Vector3f &start_p,
                                   const Eigen::Vector3f &end_p) {
  TimeTrack tracker;
  const int num_grid_x = grid_map_.size();
  const int num_grid_y = grid_map_[0].size();
  const int num_grid_z = grid_map_[0][0].size();

  std::priority_queue<std::shared_ptr<GridAstarNode>,
                      std::vector<std::shared_ptr<GridAstarNode>>,
                      GridAstarNodeCmp>
      astar_q;

  std::vector<std::vector<std::vector<GridInfo>>> grid_info_(
      num_grid_x, std::vector<std::vector<GridInfo>>(
                      num_grid_y, std::vector<GridInfo>(num_grid_z)));
  tracker.OutputPassingTime("Grid Info Init");
  tracker.SetStartTime();

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
  grid_info_[index_start_x][index_start_y][index_start_z].g_score_ = 0.0;
  grid_info_[index_start_x][index_start_y][index_start_z].h_score_ =
      CalHeurScore(astar_start, end_p);
  astar_start->f_score_ =
      grid_info_[index_start_x][index_start_y][index_start_z].g_score_ +
      grid_info_[index_start_x][index_start_y][index_start_z].h_score_;
  grid_info_[index_start_x][index_start_y][index_start_z].state_ =
      GridInfo::AstarState::kOpen;
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

    // Skip the same node due to the update of g_score.
    if (grid_info_[cur_index_x][cur_index_y][cur_index_z].state_ ==
        GridInfo::AstarState::kClose) {
      continue;
    }
    // set node to closed
    grid_info_[cur_index_x][cur_index_y][cur_index_z].state_ =
        GridInfo::AstarState::kClose;
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
      // skip nodes that is out of range
      if (next_index_x < 0 || next_index_y < 0 || next_index_z < 0) {
        continue;
      }
      if (next_index_x >= num_grid_x || next_index_y >= num_grid_y ||
          next_index_z >= num_grid_z) {
        continue;
      }
      // only expand free nodes
      if (grid_map_[next_index_x][next_index_y][next_index_z] ==
              GridAstar::GridState::kFree &&
          grid_info_[next_index_x][next_index_y][next_index_z].state_ !=
              GridInfo::AstarState::kClose) {
        // Calculate GScore.
        const float new_g_score =
            grid_info_[cur_index_x][cur_index_y][cur_index_z].g_score_ +
            resolution_;
        // Next node has not being visited.
        if (grid_info_[next_index_x][next_index_y][next_index_z].state_ ==
            GridInfo::AstarState::kNull) {
          std::shared_ptr<GridAstarNode> next_node =
              std::make_shared<GridAstarNode>(next_index_x, next_index_y,
                                              next_index_z);
          next_node->father_node_ = cur_node;
          grid_info_[next_index_x][next_index_y][next_index_z].g_score_ =
              new_g_score;
          grid_info_[next_index_x][next_index_y][next_index_z].h_score_ =
              CalHeurScore(next_node, end_p);
          // Calculate Fscore.
          next_node->f_score_ =
              new_g_score +
              grid_info_[next_index_x][next_index_y][next_index_z].h_score_;
          grid_info_[next_index_x][next_index_y][next_index_z].state_ =
              GridInfo::AstarState::kOpen;
          astar_q.push(next_node);
        } else {
          // Next node has being visited.
          std::shared_ptr<GridAstarNode> next_node =
              std::make_shared<GridAstarNode>(next_index_x, next_index_y,
                                              next_index_z);
          if (new_g_score <
              grid_info_[next_index_x][next_index_y][next_index_z].g_score_) {
            next_node->father_node_ = cur_node;
            // Update Gscore.
            grid_info_[next_index_x][next_index_y][next_index_z].g_score_ =
                new_g_score;
            const float h_score =
                grid_info_[next_index_x][next_index_y][next_index_z].h_score_;
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

inline float GridAstar::CalHeurScore(const std::shared_ptr<GridAstarNode> &node,
                                     const Eigen::Vector3f &end_p) {
  float delta_x = end_p.x() - (node->index_x_ * resolution_ + min_x_);
  float delta_y = end_p.y() - (node->index_y_ * resolution_ + min_y_);
  float delta_z = end_p.z() - (node->index_z_ * resolution_ + min_z_);
  return std::abs(delta_x) + std::abs(delta_y) + 10.0 * std::abs(delta_z);
}
