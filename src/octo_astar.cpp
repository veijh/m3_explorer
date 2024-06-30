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

const std::vector<Eigen::Vector3f> expand_offset = {
    {-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0},  {0.0, -1.0, 0.0},
    {0.0, 1.0, 0.0},  {0.0, 0.0, -1.0}, {0.0, 0.0, 1.0}};

const std::vector<std::vector<int>> adj_child = {{1, 3, 5, 7}, {0, 2, 4, 6},
                                                 {2, 3, 6, 7}, {0, 1, 4, 5},
                                                 {4, 5, 6, 7}, {0, 1, 2, 3}};

bool OctoAstar::search_octonode(const octomap::OcTree *ocmap,
                                const Eigen::Vector3f &p,
                                std::stack<OctoNode> &node_stk) {
  octomap::OcTreeNode *node = ocmap->getRoot();
  node_stk.emplace(node, Eigen::Vector3f(0.0, 0.0, 0.0), 0.1 * 65536);
  while (ocmap->nodeHasChildren(node_stk.top().node_)) {
    OctoNode node = node_stk.top();
    Eigen::Vector3f node_center = node.center_;
    int child_id = 1 * (p.x() > node_center.x()) +
                   2 * (p.y() > node_center.y()) +
                   4 * (p.z() > node_center.z());
    if (ocmap->nodeChildExists(node.node_, child_id)) {
      node_stk.emplace(ocmap->getNodeChild(node.node_, child_id),
                       node_center +
                           0.5 * node.half_size_ * center_offset[child_id],
                       node.half_size_);
    } else {
      return false;
    }
  }
  return true;
}

float OctoAstar::astar_path_distance(const octomap::OcTree *ocmap,
                                     const Eigen::Vector3f &start_p,
                                     const Eigen::Vector3f &end_p) {

  const int expand_size = expand_offset.size();

  // search the node at start_p from top to bottom
  std::stack<OctoNode> node_stk_init;
  if (!search_octonode(ocmap, start_p, node_stk_init)) {
    return 999.0;
  }

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

  AstarNode root(node_stk_init.top().center_);
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

    // std::cout << (node.position_) << ", " << node.f_score_ << std::endl <<
    // std::endl;

    if ((node.position_ - end_p).norm() < 1.6) {
      is_path_found = true;
      break;
    }

    std::stack<OctoNode> node_stk;
    search_octonode(ocmap, node.position_, node_stk);
    // std::cout << "stk size: " << node_stk.size() << std::endl
    //           << "node size: " << node_stk.top().size_ << ", " << std::endl
    //           << node_stk.top().center_ << std::endl;

    // expand in six directions
    Eigen::Vector3f next_pos;
    for (int i = 0; i < expand_size; ++i) {
      // cin.get();
      std::stack<OctoNode> node_stk_copy(node_stk);
      next_pos = node.position_ + node_stk.top().size_ * expand_offset[i];

      const int node_depth = node_stk_copy.size();
      // search for free adjacent leaf node
      // go from bottom to top until bbx contains next_pos
      node_stk_copy.pop();
      while (!node_stk_copy.top().is_in_bbx(next_pos)) {
        node_stk_copy.pop();
      }

      bool is_adj_node_null = false;
      // small node to big node, only add one node
      while (ocmap->nodeHasChildren(node_stk_copy.top().node_) &&
             node_stk_copy.size() < node_depth) {
        OctoNode node = node_stk_copy.top();
        Eigen::Vector3f node_center = node.center_;
        int child_id = 1 * (next_pos.x() > node_center.x()) +
                       2 * (next_pos.y() > node_center.y()) +
                       4 * (next_pos.z() > node_center.z());
        if (ocmap->nodeChildExists(node.node_, child_id)) {
          node_stk_copy.emplace(ocmap->getNodeChild(node.node_, child_id),
                                node_center + 0.5 * node.half_size_ *
                                                  center_offset[child_id],
                                node.half_size_);
        } else {
          is_adj_node_null = true;
          break;
        }
      }

      if (is_adj_node_null) {
        continue;
      }

      // big node to small node, may add multi nodes
      if (ocmap->nodeHasChildren(node_stk_copy.top().node_)) {
        // bfs
        std::queue<OctoNode> bfs_q;
        bfs_q.push(node_stk_copy.top());
        while (!bfs_q.empty()) {
          OctoNode bfs_node = bfs_q.front();
          bfs_q.pop();
          if (!ocmap->nodeHasChildren(bfs_node.node_)) {
            next_pos = bfs_node.center_;
            if(!ocmap->isNodeOccupied(bfs_node.node_)){
              add_node_to_q(ocmap, next_pos, end_p, node, astar_q, count,
                            node_state, node_g_score);
            }
            continue;
          }
          for (int index = 0; index < 4; ++index) {
            if (ocmap->nodeChildExists(bfs_node.node_, adj_child[i][index])) {
              octomap::OcTreeNode *childe_node =
                  ocmap->getNodeChild(bfs_node.node_, adj_child[i][index]);
              if (ocmap->isNodeOccupied(childe_node)) {
                continue;
              }
              bfs_q.emplace(childe_node,
                            bfs_node.center_ +
                                0.5 * bfs_node.half_size_ *
                                    center_offset[adj_child[i][index]],
                            bfs_node.half_size_);
            }
          }
        }
      } else {
        // shift to the center of free leaf node
        next_pos = node_stk_copy.top().center_;
        if(!ocmap->isNodeOccupied(node_stk_copy.top().node_)){
          add_node_to_q(ocmap, next_pos, end_p, node, astar_q, count, node_state,
                        node_g_score);
        }
      }
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
  // octomap::point3d bbx_min(std::min(cur_pos.x(), next_pos.x()) - 0.2,
  //                          std::min(cur_pos.y(), next_pos.y()) - 0.2,
  //                          std::min(cur_pos.z(), next_pos.z()) - 0.1);
  // octomap::point3d bbx_max(std::max(cur_pos.x(), next_pos.x()) + 0.2,
  //                          std::max(cur_pos.y(), next_pos.y()) + 0.2,
  //                          std::max(cur_pos.z(), next_pos.z()) + 0.1);

  // for (octomap::OcTree::leaf_bbx_iterator
  //          it = ocmap->begin_leafs_bbx(bbx_min, bbx_max),
  //          end = ocmap->end_leafs_bbx();
  //      it != end; ++it) {
  //   if (ocmap->isNodeOccupied(*it)) {
  //     return false;
  //   }
  // }
  // return true;
  octomap::OcTreeNode *oc_node = ocmap->search(next_pos.x(), next_pos.y(), next_pos.z());
  if (oc_node == nullptr) {
    // cout << "unknown" << endl;
    return false;
  }

  if (oc_node != nullptr && ocmap->isNodeOccupied(oc_node)) {
    // cout << "occ" << endl;
    return false;
  }

  return true;
}

bool OctoAstar::add_node_to_q(
    const octomap::OcTree *ocmap, const Eigen::Vector3f &next_pos,
    const Eigen::Vector3f &end_p, AstarNode &node,
    std::priority_queue<AstarNode, std::vector<AstarNode>, AstarNodeCmp>
        &astar_q,
    int &count, std::unordered_map<AstarNode, int, AstarNodeHash> &node_state,
    std::unordered_map<AstarNode, float, AstarNodeHash> &node_g_score) {
  // check next node is valid
  if (next_pos.z() > max_z_ || next_pos.z() < min_z_) {
    return false;
  }

  // bool is_next_node_valid = is_path_valid(ocmap, node.position_, next_pos);
  // if (!is_next_node_valid) {
  //   return false;
  // }

  const float delta_g = (next_pos - node.position_).norm();
  AstarNode next_node(next_pos);
  // check if node is in open/closed list
  if (node_state.find(next_node) == node_state.end()) {
    node_state[next_node] = 0;
  } else {
    if (node_state[next_node] == 0 &&
        node.g_score_ + delta_g > node_g_score[next_node]) {
      return false;
    }
  }
  next_node.father_id_ = count;
  next_node.h_score_ = calc_h_score(next_node.position_, end_p);
  next_node.g_score_ = node.g_score_ + delta_g;
  node_g_score[next_node] = next_node.g_score_;
  next_node.f_score_ = next_node.g_score_ + next_node.h_score_;
  astar_q.push(next_node);
  return true;
}
