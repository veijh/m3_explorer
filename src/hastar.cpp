#include "m3_explorer/hastar.h"
const float MAX_VEL = 1.0;

bool Hastar::search_path(const octomap::OcTree* ocmap, const Eigen::Vector3f& start_p, const Eigen::Vector3f& end_p, const float& vel, const float& yaw, const float& vz)
{
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

    // map<PathNode, int, MapCmp> node_state;
    // map<PathNode, float, MapCmp> node_g_score;

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
        // closed_list[count] = node;
        closed_list.emplace_back(node);
        node_state[node] = 1;

        // cout << (node.position - end_p).norm() << endl << endl;
        // cout << (node.position) << node.vel << endl << endl;

        // 终点处应当约束速度为0,此处可以用庞特里亚金求解
        if((node.position - end_p).norm() < 0.2 && node.vel < 0.2){
        // if((node.position - end_p).norm() < 0.2){
            is_path_found = true;
            cout << "[Hastar] find_path !!!" << endl;
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
            next_vel = vel_end.topRows(2).norm();
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
        path.clear();

        float end_yaw = atan2(end_p.y() - closed_list[count].position.y(),
        end_p.x() - closed_list[count].position.x());
        PathNode end(end_p, end_yaw, 0.0, 0.0);
        path.push_back(end);

        int id = closed_list[count].father_id;
        path.push_back(closed_list[count]);
        while(id != -1){
            path.push_back(closed_list[id]);
            id = closed_list[id].father_id;
        }
        reverse(path.begin(), path.end());
        cout << "[Hastar] waypoint generated!! waypoint num: " << path.size() << endl;
        trajectory_generate();
        return true;
    }
    else{
        cout << "[Hastar] no path" << endl;
        return false;
    }
}

float Hastar::calc_h_score(const Eigen::Vector3f& start_p, const Eigen::Vector3f& end_p)
{
    return (end_p-start_p).norm() / MAX_VEL;
}

bool Hastar::is_path_valid(const octomap::OcTree* ocmap, const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& next_pos)
{
    octomap::point3d next_pos_check(next_pos.x(), next_pos.y(), next_pos.z());
    octomap::OcTreeNode* oc_node = ocmap->search(next_pos_check);
    if(oc_node == nullptr) return false;

    float bbx_x0 = min(cur_pos.x(), next_pos.x()) - 0.3;
    float bbx_x1 = max(cur_pos.x(), next_pos.x()) + 0.3;
    float bbx_y0 = min(cur_pos.y(), next_pos.y()) - 0.3;
    float bbx_y1 = max(cur_pos.y(), next_pos.y()) + 0.3;
    float bbx_z0 = min(cur_pos.z(), next_pos.z()) - 0.1;
    float bbx_z1 = max(cur_pos.z(), next_pos.z()) + 0.1;

    for(float check_x = bbx_x0; check_x <= bbx_x1; check_x += 0.1){
        for(float check_y = bbx_y0; check_y <= bbx_y1; check_y += 0.1){
            for(float check_z = bbx_z0; check_z <= bbx_z1; check_z += 0.1){
                octomap::point3d check(check_x, check_y, check_z);
                octomap::OcTreeNode* oc_node = ocmap->search(check);
                if(oc_node != nullptr && ocmap->isNodeOccupied(oc_node)){
                    return false;
                }
            }
        }
    }
    return true;
}

// bool Hastar::is_path_valid(const octomap::OcTree* ocmap, const Eigen::Vector3f& cur_pos, const Eigen::Vector3f& next_pos)
// {
//     return true;
// }

bool Hastar::trajectory_generate(){
    traj.clear();
    for(int i = 0; i < path.size() - 1; i++){
        Eigen::Vector3f vel_start = {path[i].vel*cos(path[i].yaw), path[i].vel*sin(path[i].yaw), path[i].vz};
        for(float time = 0.0; time < tau; time += traj_sample){
            Traj traj_point;
            traj_point.acc = path[i+1].father_acc;
            traj_point.vel = vel_start + path[i+1].father_acc * time;
            traj_point.pos = path[i].position + vel_start * time + path[i+1].father_acc * pow(time, 2) / 2.0;
            if(traj_point.vel.topRows(2).norm() > 1e-2){
                traj_point.yaw = atan2(traj_point.vel(1), traj_point.vel(0));
            }
            else{
                traj_point.yaw = path[i].yaw;
            }
            traj.push_back(traj_point);
        }
    }
    Traj traj_point;
    traj_point.acc = Eigen::Vector3f::Zero();
    traj_point.vel = Eigen::Vector3f::Zero();
    traj_point.pos = path.back().position;
    traj.push_back(traj_point);
    cout << "[Hastar] Traj generate OK!! traj point num: " << traj.size() << endl;
    return true;
}
