#include "m3_explorer/octo_astar.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <memory>
#include <queue>
#include <stack>
#include <string>
#include <unordered_map>

const std::vector<Eigen::Vector3f> center_offset = {
    {-1.0, -1.0, -1.0}, {1.0, -1.0, -1.0}, {-1.0, 1.0, -1.0}, {1.0, 1.0, -1.0},
    {-1.0, -1.0, 1.0},  {1.0, -1.0, 1.0},  {-1.0, 1.0, 1.0},  {1.0, 1.0, 1.0}};

std::stack<OctoNode> OctoAstar::search_octonode(const octomap::OcTree *ocmap,
                                                const Eigen::Vector3f &p) {
  octomap::OcTreeNode *node = ocmap->getRoot();
  std::stack<OctoNode> node_stk;
  node_stk.emplace(node, Eigen::Vector3f(0.0, 0.0, 0.0), 0.1 * 65536);
  while (ocmap->nodeHasChildren(node_stk.top().node_)) {
    OctoNode node = node_stk.top();
    Eigen::Vector3f node_center = node.center_;
    int child_id = 1 * (p.x() > node_center.x()) +
                   2 * (p.y() > node_center.y()) +
                   4 * (p.z() > node_center.z());
    node_stk.emplace(ocmap->getNodeChild(node.node_, child_id),
                     node_center +
                         0.5 * node.half_size_ * center_offset[child_id],
                     node.half_size_);
  }
  return node_stk;
}

float OctoAstar::astar_path_distance(const octomap::OcTree *ocmap,
                                 const Eigen::Vector3f &start_p,
                                 const Eigen::Vector3f &end_p) {
  const std::vector<Eigen::Vector3f> expand_offset = {
      {-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0},  {0.0, -1.0, 0.0},
      {0.0, 1.0, 0.0},  {0.0, 0.0, -1.0}, {0.0, 0.0, 1.0}};

  const int expand_size = expand_offset.size();

  // search the node at start_p from top to bottom
  std::stack<OctoNode> node_stk = search_octonode(ocmap, start_p);

  std::priority_queue<AstarNode, std::vector<AstarNode>, AstarNodeCmp> astar_q;
  std::vector<AstarNode> closed_list;
  closed_list.reserve(99999);
  // size of closed_list, maybe faster than closed_list.size()
  int count = 0;

  // state: 0 -> open; 1 -> closed
  std::unordered_map<AstarNode, int, AstarNodeHash> node_state;
  std::unordered_map<AstarNode, float, AstarNodeHash> node_g_score;

  // map<AstarNode, int, AstarMapCmp> node_state;
  // map<AstarNode, float, AstarMapCmp> node_g_score;

  bool is_path_found = false;

  AstarNode root(node_stk.top().center_);
  root.father_id_ = -1;
  root.g_score_ = 0.0;
  root.h_score_ = calc_h_score(root.position_, end_p);
  root.f_score_ = root.g_score_ + root.h_score_;
  astar_q.push(root);
  node_state[root] = 0;
  node_g_score[root] = 0.0;

  while (!astar_q.empty()) {
    // cin.get();
    // std::cout << "top node: " << std::endl;
    // selection
    AstarNode node = astar_q.top();
    astar_q.pop();
    // add node to closed list
    // g值更新导致节点重复
    if (node_state[node] == 1) {
      continue;
    }
    // closed_list[count] = node;
    closed_list.emplace_back(node);
    node_state[node] = 1;

    // std::cout << (node.position_) << ", " << node.f_score_ << std::endl << std::endl;

    if ((node.position_ - end_p).norm() < 0.2) {
      is_path_found = true;
      break;
    }

    std::stack<OctoNode> node_stk = search_octonode(ocmap, start_p);

    // expand in six directions
    Eigen::Vector3f next_pos;
    for (int i = 0; i < expand_size; ++i) {
      // cin.get();
      next_pos = node.position_ + node_stk.top().size_ * expand_offset[i];

      std::stack<OctoNode> node_stk_copy(node_stk);

      // check next node is valid
      if (next_pos.z() > max_z_ || next_pos.z() < min_z_) {
        continue;
      }

      // shift to the center of free leaf node
      next_pos = ;

      bool is_next_node_valid = is_path_valid(ocmap, node.position_, next_pos);
      if (!is_next_node_valid) {
        continue;
      }

      AstarNode next_node(next_pos);
      // check if node is in open/closed list
      if (node_state.find(next_node) == node_state.end()) {
        node_state[next_node] = 0;
      } else {
        if (node_state[next_node] == 0 &&
            node.g_score_ + 0.2 > node_g_score[next_node]) {
          continue;
        }
      }
      next_node.father_id_ = count;
      next_node.h_score_ = calc_h_score(next_node.position_, end_p);
      next_node.g_score_ = node.g_score_ + 0.2;
      node_g_score[next_node] = next_node.g_score_;
      next_node.f_score_ = next_node.g_score_ + next_node.h_score_;
      astar_q.push(next_node);
    }

    count++;
  }

  if (is_path_found) {
    float distance = 0.0;
    path_.clear();
    // add accurate end point
    path_.emplace_back(end_p);
    path_.emplace_back(closed_list[count]);

    Eigen::Vector3f last = closed_list[count].position_;
    distance += (last - end_p).norm();

    int id = closed_list[count].father_id_;
    while (id != -1) {
      distance += (closed_list[id].position_ - last).norm();
      last = closed_list[id].position_;

      path_.emplace_back(closed_list[id]);
      id = closed_list[id].father_id_;
    }
    reverse(path_.begin(), path_.end());
    std::cout << "[Astar] waypoint generated!! waypoint num: " << path_.size()
         << ", select node num: " << count << std::endl;
    return distance;
  } else {
    std::cout << "[WARNING] no path !! from " << std::endl
         << start_p << std::endl
         << "to " << std::endl
         << end_p << std::endl;
    return (end_p - start_p).norm();
  }
}

float OctoAstar::calc_h_score(const Eigen::Vector3f &start_p,
                          const Eigen::Vector3f &end_p) {
  // return (end_p - start_p).norm();
  // return (end_p - start_p).lpNorm<1>();
  Eigen::Vector3f delta = end_p - start_p;
  return abs(delta.x()) + abs(delta.y()) + 10.0 * abs(delta.z());
}

bool OctoAstar::is_path_valid(const octomap::OcTree *ocmap,
                          const Eigen::Vector3f &cur_pos,
                          const Eigen::Vector3f &next_pos) {
  octomap::point3d bbx_min;
  bbx_min.x() = std::min(cur_pos.x(), next_pos.x()) - 0.3;
  bbx_min.y() = std::min(cur_pos.y(), next_pos.y()) - 0.3;
  bbx_min.z() = std::min(cur_pos.z(), next_pos.z()) - 0.2;
  octomap::point3d bbx_max;
  bbx_max.x() = std::max(cur_pos.x(), next_pos.x()) + 0.3;
  bbx_max.y() = std::max(cur_pos.y(), next_pos.y()) + 0.3;
  bbx_max.z() = std::max(cur_pos.z(), next_pos.z()) + 0.2;

  for (octomap::OcTree::leaf_bbx_iterator
           it = ocmap->begin_leafs_bbx(bbx_min, bbx_max),
           end = ocmap->end_leafs_bbx();
       it != end; ++it) {
    if (ocmap->isNodeOccupied(*it)) {
      return false;
    }
  }
  return true;
}
