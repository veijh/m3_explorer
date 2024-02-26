#include "m3_explorer/frontier_cluster.h"
vector<Cluster> k_mean_cluster(set<QuadMesh>& frontiers){
    // dynamic cluster num
    int cluster_num = frontiers.size() / 500;

    // convert set to matrix
    Eigen::Matrix3Xf frontier_center_mat(3, frontiers.size());
    Eigen::Matrix3Xf frontier_normal_mat(3, frontiers.size());
    Eigen::RowVectorXf frontier_size_mat(frontiers.size());
    set<QuadMesh>::iterator it = frontiers.begin();
    for (int i = 0; i < frontiers.size(); i++)
    {
        frontier_center_mat.col(i) << it->center.x(), it->center.y(), it->center.z();
        frontier_normal_mat.col(i) << it->normal.x(), it->normal.y(), it->normal.z();
        frontier_size_mat.col(i) << it->size;
        it++;
    }

    int min = 0, max = frontiers.size() - 1;
    random_device seed;                              // 硬件生成随机数种子
    ranlux48 engine(seed());                         // 利用种子生成随机数引擎
    uniform_int_distribution<int> distrib(min, max); // 设置随机数范围，并为均匀分布

    // initial clusters' center
    vector<int> initial_idx;
    while (initial_idx.size() < cluster_num)
    {
        int random = distrib(engine);
        if (find(initial_idx.begin(), initial_idx.end(), random) == initial_idx.end())
        {
            initial_idx.push_back(random);
        }
    }
    Eigen::Matrix3Xf cluster_center(3, cluster_num);
    
    for (int i = 0; i < initial_idx.size(); i++)
    {
        cluster_center.col(i) = frontier_center_mat.col(initial_idx[i]);
    }

    Eigen::MatrixXf distance_to_center(cluster_num, frontiers.size());
    for (int iter_count = 0; iter_count < MAX_ITERATION; iter_count++)
    {
        // calculate the distance from each point to all centers
        for (int i = 0; i < cluster_num; i++)
        {
            distance_to_center.row(i) = (cluster_center.col(i).replicate(1, frontier_center_mat.cols()) - frontier_center_mat).colwise().norm();
        }

        vector<double> cluster_weight(cluster_num, 0);
        Eigen::Matrix3Xf new_center(3, cluster_num);
        new_center.setZero();
        for (int i = 0; i < distance_to_center.cols(); i++)
        {
            int idx = 0;
            // cluster that frontiers belong to  
            distance_to_center.col(i).minCoeff(&idx);
            double weight = frontier_size_mat(i) * frontier_size_mat(i);
            cluster_weight[idx] += weight;
            new_center.col(idx) += weight * frontier_center_mat.col(i);
        }

        for (int i = 0; i < new_center.cols(); i++)
        {
            new_center.col(i) /= cluster_weight[i];
        }

        float delta = (new_center - cluster_center).colwise().norm().maxCoeff();
        if (delta < 1e-3)
        {
            cout << "break after iteration " << iter_count << endl;
            break;
        }
        else
        {
            cluster_center = new_center;
        }
    }

    // calculate cluster normal
    Eigen::Matrix3Xf cluster_normal(3, cluster_num);
    cluster_normal.setZero();
    vector<double> cluster_weight(cluster_num, 0);
    for (int i = 0; i < distance_to_center.cols(); i++)
    {
        int idx = 0;
        distance_to_center.col(i).minCoeff(&idx);
        double weight = frontier_size_mat(i) * frontier_size_mat(i);
        cluster_weight[idx] += weight;
        cluster_normal.col(idx) += weight * frontier_normal_mat.col(i);
    }

    cluster_normal = cluster_normal.colwise().normalized();

    vector<Cluster> cluster_vec;
    Cluster cluster_result;
    for(int i = 0; i < cluster_num; i++){
        cluster_result.center = cluster_center.col(i);
        cluster_result.normal = cluster_normal.col(i);
        cluster_vec.push_back(cluster_result);
    }
    return cluster_vec;
}

void cluster_visualize(vector<Cluster>& cluster_vec, ros::Publisher& cluster_pub){
    visualization_msgs::MarkerArray marker_cluster;

    // visualize cluster center
    visualization_msgs::Marker marker_cluster_center;
    marker_cluster_center.id = 0;
    marker_cluster_center.header.frame_id = "map";
    marker_cluster_center.pose.orientation.w = 1.0;
    marker_cluster_center.pose.orientation.x = 0.0;
    marker_cluster_center.pose.orientation.y = 0.0;
    marker_cluster_center.pose.orientation.z = 0.0;
    marker_cluster_center.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_cluster_center.color.a = 1; marker_cluster_center.color.r = 0; marker_cluster_center.color.g = 1; marker_cluster_center.color.b = 0;
    marker_cluster_center.scale.x = 0.1; marker_cluster_center.scale.y = 0.1; marker_cluster_center.scale.z = 0.1;

    for(int i = 0; i < cluster_vec.size(); i++){
        geometry_msgs::Point point;
        point.x = cluster_vec[i].center.x();
        point.y = cluster_vec[i].center.y();
        point.z = cluster_vec[i].center.z();
        marker_cluster_center.points.push_back(point);
    }
    marker_cluster.markers.push_back(marker_cluster_center);

    // visualize normal vector of cluster center
    visualization_msgs::Marker marker_cluster_normal(marker_cluster_center);
    marker_cluster_normal.type = visualization_msgs::Marker::ARROW;
    marker_cluster_normal.color.a = 1; marker_cluster_normal.color.r = 0; marker_cluster_normal.color.g = 1; marker_cluster_normal.color.b = 0;
    marker_cluster_normal.scale.x = 0.08; marker_cluster_normal.scale.y = 0.15; marker_cluster_normal.scale.z = 0.15;

    double arrow_len = 0.5;
    for(int i = 0; i < cluster_vec.size(); i++){
        marker_cluster_normal.points.clear();
        marker_cluster_normal.id = 200+i;
        geometry_msgs::Point point;
        point.x = cluster_vec[i].center.x();
        point.y = cluster_vec[i].center.y();
        point.z = cluster_vec[i].center.z();
        marker_cluster_normal.points.push_back(point);
        point.x = cluster_vec[i].center.x() + cluster_vec[i].normal.x() * arrow_len;
        point.y = cluster_vec[i].center.y() + cluster_vec[i].normal.y() * arrow_len;
        point.z = cluster_vec[i].center.z() + cluster_vec[i].normal.z() * arrow_len;
        marker_cluster_normal.points.push_back(point);
        marker_cluster.markers.push_back(marker_cluster_normal);
    }
    cluster_pub.publish(marker_cluster);
}

geometry_msgs::PoseArray view_point_generate(vector<Cluster>& cluster_vec, octomap::OcTree* ocmap){
    geometry_msgs::PoseArray view_point_array;
    view_point_array.header.frame_id = "map";
    
    for(int i = 0; i < cluster_vec.size(); i++){
        octomap::point3d center(cluster_vec[i].center.x(), cluster_vec[i].center.y(), cluster_vec[i].center.z());
        octomap::point3d normal(cluster_vec[i].normal.x(), cluster_vec[i].normal.y(), 0);
        for(double offset = 0.3; offset < 2.5; offset += 0.1){
            octomap::point3d view_point = center - normal * offset;
            // view point has to be reachable
            octomap::OcTreeNode* view_node = ocmap->search(view_point);
            if(view_node == nullptr || view_node->getOccupancy() > 0.6){
                break;
            }
            // there is no obstacle near the view point
            if(is_next_to_obstacle(ocmap, view_point, 0.8)){
                continue;
            }
            else{
                geometry_msgs::Pose view_point_pose;
                view_point_pose.position.x = view_point.x();
                view_point_pose.position.y = view_point.y();
                view_point_pose.position.z = view_point.z();
                tf2::Quaternion quaternion;
                quaternion.setRPY(0.0, 0.0, atan2(normal.y(), normal.x()));
                view_point_pose.orientation.w = quaternion.w();
                view_point_pose.orientation.x = quaternion.x();
                view_point_pose.orientation.y = quaternion.y();
                view_point_pose.orientation.z = quaternion.z();
                view_point_array.poses.push_back(view_point_pose);
                break;
            }
        }
    }
    return view_point_array;
}
