#include "m3_explorer/hastar.h"
const float MAX_VEL = 1.0;

bool Hastar::search_path(const octomap::OcTree* ocmap, const Eigen::Vector3f& start_p, const Eigen::Vector3f& end_p, const float& vel, const float& yaw, const float& vz)
{
    float tau = 0.5;
    vector<float> acc_t = {-1.0, -0.5, 0, 0.5, 1.0};
    vector<float> acc_n = {-1.0, -0.5, 0, 0.5, 1.0};
    vector<float> acc_z = {0};

    priority_queue<PathNode, vector<PathNode>, NodeCmp> hastar_q;
    vector<PathNode> closed_list;
    // size of closed_list, maybe faster than closed_list.size()
    int count = 0;
    // state: 0 -> open; 1 -> closed
    unordered_map<PathNode, int, NodeHash> node_state;
    unordered_map<PathNode, float, NodeHash> node_g_score;

    PathNode root(start_p, yaw, vel, vz);
    root.father_id = -1;
    root.g_score = 0.0;
    root.h_score = calc_h_score(root.position, end_p);
    root.f_score = root.g_score + root.h_score;
    hastar_q.push(root);
    node_state[root] = 0;
    node_g_score[root] = 0.0;
    bool is_path_found = false;

    while(!hastar_q.empty()){
        // selection
        PathNode node = hastar_q.top();
        hastar_q.pop();
        // add node to closed list
        // 可以考虑将priority_queue替换为set，因为g值更新导致节点重复
        if(node_state[node] == 1){
            continue;
        }
        closed_list.push_back(node);
        node_state[node] = 1;

        // cout << (node.position - end_p).norm() << endl << endl;
        // cout << (node.position) << node.vel << endl << endl;

        if((node.position - end_p).norm() < 0.5){
            is_path_found = true;
            cout << "find_path" << endl;
            break;
        }

        // expansion
        Eigen::Vector3f next_pos;
        float next_yaw, next_vel, next_vz;
        for(int i = 0; i < acc_t.size(); i++){ for(int j = 0; j < acc_n.size(); j++){ for(int k = 0; k < acc_z.size(); k++){
            Eigen::Vector3f acc = {
            acc_t[i] * (float)cos(node.yaw) + acc_n[j] * (float)cos(node.yaw + M_PI/2.0),
            acc_t[i] * (float)sin(node.yaw) + acc_n[j] * (float)sin(node.yaw + M_PI/2.0),
            acc_z[k]};
            Eigen::Vector3f vel_start = {node.vel*cos(node.yaw), node.vel*sin(node.yaw), node.vz};
            Eigen::Vector3f vel_end = vel_start + acc * tau;
            next_pos = node.position + vel_start * tau + acc * pow(tau, 2) / 2.0;
            next_vel = sqrt(vel_end(0) * vel_end(0) + vel_end(1) * vel_end(1));
            next_vz = vel_end(2);
            if(next_vel > 1e-2){
                next_yaw = atan2(vel_end(1), vel_end(0));
            }
            else{
                next_yaw = node.yaw;
            }

            // check max vel constraint
            if(vel_end.norm() > MAX_VEL){
                continue;
            }

            // check next node is valid
            bool is_next_node_valid = is_path_valid(ocmap, node.position, next_pos);

            if(!is_next_node_valid){
                continue;
            }

            PathNode next_node(next_pos, next_yaw, next_vel, next_vz);
            // check if node is in open/closed list
            if(node_state.find(next_node) == node_state.end()){
                node_state[next_node] = 0;
            }
            else{
                if(node_state[next_node] == 0 && node.g_score + tau > node_g_score[next_node]){
                    continue;
                }
            }
            next_node.father_id = count;
            next_node.father_acc = acc;
            next_node.h_score = calc_h_score(next_node.position, end_p);
            next_node.g_score = node.g_score + tau;
            node_g_score[next_node] = next_node.g_score;
            next_node.f_score = next_node.g_score + next_node.h_score;
            hastar_q.push(next_node);

        }}}
        count++;
    }

    if(is_path_found){
        // path.clear();
        // int id = closed_list[closed_list.size()-1].father_id;
        // path.push_back(closed_list[closed_list.size()-1]);
        // while(id != -1){
        //     path.push_back(closed_list[id]);
        //     id = closed_list[id].father_id;
        // }
        // reverse(path.begin(), path.end());
        cout << "ok" << endl;
        return true;
    }
    else{
        cout << "no path" << endl;
        return false;
    }
}

float Hastar::calc_h_score(const Eigen::Vector3f& start_p, const Eigen::Vector3f& end_p)
{
    return (end_p-start_p).norm() / MAX_VEL;
}

// bool Hastar::is_path_valid(const octomap::OcTree* ocmap, const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& next_pos)
// {
//     float bbx_x0 = min(cur_pos.x(), next_pos.x()) - 0.25;
//     float bbx_x1 = max(cur_pos.x(), next_pos.x()) + 0.25;
//     float bbx_y0 = min(cur_pos.y(), next_pos.y()) - 0.25;
//     float bbx_y1 = max(cur_pos.y(), next_pos.y()) + 0.25;
//     float bbx_z0 = min(cur_pos.z(), next_pos.z()) - 0.1;
//     float bbx_z1 = max(cur_pos.z(), next_pos.z()) + 0.1;

//     for(float check_x = bbx_x0; check_x <= bbx_x1; check_x += 0.1){
//         for(float check_y = bbx_y0; check_y <= bbx_y1; check_y += 0.1){
//             for(float check_z = bbx_z0; check_z <= bbx_z1; check_z += 0.1){
//                 octomap::point3d check(check_x, check_y, check_z);
//                 octomap::OcTreeNode* oc_node = ocmap->search(check);
//                 if(oc_node != nullptr && ocmap->isNodeOccupied(oc_node)){
//                     return false;
//                 }
//             }
//         }
//     }
//     return true;
// }

bool Hastar::is_path_valid(const octomap::OcTree* ocmap, const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& next_pos)
{
    return true;
}
