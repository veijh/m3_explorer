#include "m3_explorer/path_planning.h"
geometry_msgs::PoseArray atsp_path(const geometry_msgs::PointStamped& current_pose, const geometry_msgs::PoseArray view_points, ros::ServiceClient& lkh_client, const string& problem_path){
    geometry_msgs::PoseArray result;
    string par = problem_path + "/single.par";
    string atsp = problem_path + "/single.atsp";
    string tour = problem_path + "/single.tour";
    // create par file for lkh
    std::ofstream par_file(par);
    if(!par_file.is_open()){
        std::cerr << "Failed to open par file." << std::endl;
        return result;
    }
    par_file << "PROBLEM_FILE = " << atsp << endl;
    par_file << "TOUR_FILE = " << tour << endl;
    par_file.close();

    // create atsp file for lkh
    std::ofstream atsp_file(atsp);
    if(!atsp_file.is_open()){
        std::cerr << "Failed to open atsp file." << std::endl;
        return result;
    }
    atsp_file << "NAME : EXPLORATION" << endl;
    atsp_file << "TYPE : ATSP" << endl;
    atsp_file << "DIMENSION : " << view_points.poses.size() + 1 << endl;
    atsp_file << "EDGE_WEIGHT_TYPE: EXPLICIT" << endl;
    atsp_file << "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << endl;
    atsp_file << "EDGE_WEIGHT_SECTION" << endl;
    // weight of edge from i to j
    for(int i = -1; i < view_points.poses.size(); i++){
        for(int j = -1; j < view_points.poses.size(); j++){
            if(j == -1 || i == j){
                atsp_file << to_string(0) << " ";
            }
            else{
                if(i == -1){
                    Eigen::Vector2f edge_w(current_pose.point.x - view_points.poses[j].position.x, current_pose.point.y - view_points.poses[j].position.y);
                    atsp_file << to_string((int)(edge_w.norm()*1000.0)) << " ";
                }
                else{
                    Eigen::Vector2f edge_w(view_points.poses[i].position.x - view_points.poses[j].position.x, view_points.poses[i].position.y - view_points.poses[j].position.y);
                    atsp_file << to_string((int)(edge_w.norm()*1000.0)) << " ";
                }
            }
        }
        atsp_file << endl;
    }
    atsp_file.close();

    // call service
    lkh_ros::Solve srv;
    srv.request.problem_file = par;
    if (lkh_client.call(srv)) {
        cout << "[INFO] atsp solved" << endl;
    } else {
        cout << "[ERROR] atsp solving error!!" << endl;
    }
    // read result && return

    std::ifstream tour_file(tour);
    if(!tour_file.is_open()){
        std::cerr << "Failed to open tour file." << std::endl;
        return result;
    }

    std::string line;
    while (std::getline(tour_file, line)) {
        if(line == "TOUR_SECTION"){
            break;
        }
    }

    vector<int> node_seq;
    while (std::getline(tour_file, line)) {
        int num = stoi(line);
        if(num == -1){
            break;
        }
        else{
            node_seq.push_back(num);
        }
    }
    tour_file.close();

    // check whether the result is reasonable
    if(node_seq.size() != view_points.poses.size() + 1){
        std::cerr << "the result is not reasonable" << std::endl;
        return result;
    }

    result.header.frame_id = "map";
    vector<int>::iterator it = find(node_seq.begin(), node_seq.end(), 1);
    geometry_msgs::Pose pose_now;
    pose_now.position.x = current_pose.point.x;
    pose_now.position.y = current_pose.point.y;
    pose_now.position.z = current_pose.point.z;
    result.poses.push_back(pose_now);

    for(int i = 0; i < node_seq.size(); i++){
        if(it+1 == node_seq.end()){
            it = node_seq.begin();
        }
        else{
            it++;
        }
        int id = *it-2;
        result.poses.push_back(view_points.poses[id]);
    }
    return result;
}