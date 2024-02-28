#include "m3_explorer/path_planning.h"
geometry_msgs::PoseArray atsp_path(const geometry_msgs::PointStamped& current_pose, const geometry_msgs::PoseArray& view_points, ros::ServiceClient& lkh_client, const string& problem_path){
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
    geometry_msgs::PoseArray all_node(view_points);
    geometry_msgs::Pose start_node;
    start_node.orientation.w = 1; start_node.orientation.x = 0; start_node.orientation.y = 0; start_node.orientation.z = 0;
    start_node.position.x = current_pose.point.x;
    start_node.position.y = current_pose.point.y;
    start_node.position.z = current_pose.point.z;
    all_node.poses.push_back(start_node);

    for(int i = 0; i < all_node.poses.size(); i++){
        cout << "writing file !!" << endl;
        for(int j = 0; j < all_node.poses.size(); j++){
            if(j == all_node.poses.size() - 1 || i == j){
                atsp_file << 0 << " ";
            }
            else{
                Eigen::Vector2f edge_w(all_node.poses[i].position.x - all_node.poses[j].position.x, all_node.poses[i].position.y - all_node.poses[j].position.y);
                atsp_file << (int)(edge_w.norm()*1000.0) << " ";
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
    if(node_seq.size() != all_node.poses.size()){
        std::cerr << "the result is not reasonable" << std::endl;
        return result;
    }

    result.header.frame_id = "map";
    vector<int>::iterator it = find(node_seq.begin(), node_seq.end(), all_node.poses.size());

    for(int i = 0; i < node_seq.size(); i++){
        int id = *it-1;
        result.poses.push_back(all_node.poses[id]);
        if(it+1 == node_seq.end()){
            it = node_seq.begin();
        }
        else{
            it++;
        }
    }
    return result;
}
