#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <vector>
#include <unordered_map>
#include <set>
#include <queue>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <sys/time.h>
#include <utility>
#include <cmath>
#include <random>
#include "m3_explorer/frontier_detector.h"

using namespace std;
octomap::OcTree* ocmap;

void octomap_cb(const octomap_msgs::Octomap::ConstPtr& msg)
{
  // free memory for old map
  delete ocmap;
  ocmap = dynamic_cast<octomap::OcTree*>(msgToMap(*msg));

  double i = 0.0, j = 0.0, k = 0.0;
  // ocmap->getMetricMax(i, j, k);
  // cout << "max: " << i << ", " << j << ", " << k << endl;
  // ocmap->getMetricMin(i, j, k);
  // cout << "min: " << i << ", " << j << ", " << k << endl;
  ocmap->getMetricSize(i, j, k);
  cout << "size: " << i << ", " << j << ", " << k << endl << endl;

  // for(octomap::OcTree::tree_iterator it = ocmap->begin_tree(), end = ocmap->end_tree(); it!= end; ++it)
  // {
  //   if(it.getSize() > 1.0){
  //   //manipulate node, e.g.:
  //   std::cout << "Node center: " << it.getCoordinate() << std::endl;
  //   std::cout << "Node size: " << it.getSize() << std::endl;
  //   std::cout << "Node value: " << it->getValue() << std::endl;
  //   std::cout << "Node depth: " << it.getDepth() << std::endl;
  //   }
  // }

  // for(octomap::OcTree::leaf_iterator it = ocmap->begin_leafs(),
  //      end=ocmap->end_leafs(); it!= end; ++it)
  // {
  //   if(it.getSize() > 0.4){
  //   //manipulate node, e.g.:
  //   std::cout << "Node center: " << it.getCoordinate() << std::endl;
  //   std::cout << "Node size: " << it.getSize() << std::endl;
  //   std::cout << "Node value: " << it->getValue() << std::endl;
  //   }
  // }

  // octomap::point3d min(0,0,0);
  // octomap::point3d max(5,5,5);

  // for(octomap::OcTree::leaf_bbx_iterator it = ocmap->begin_leafs_bbx(min,max),
  //      end=ocmap->end_leafs_bbx(); it!= end; ++it)
  // {
  //   if(it.getSize() > 0.4){
  //     //manipulate node, e.g.:
  //     std::cout << "Node center: " << it.getCoordinate() << std::endl;
  //     std::cout << "Node size: " << it.getSize() << std::endl;
  //     std::cout << "Node value: " << it->getValue() << std::endl;
  //     std::cout << "Node depth: " << it.getDepth() << std::endl;
  //   }
  // }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "m3_explorer");
  ros::NodeHandle nh("");
  ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/uav0/octomap_binary", 1, octomap_cb);
  double resolution = 0.1, sensor_range = 5.0;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  ros::Duration(1.0).sleep();  // 等待tf2变换树准备好
  ros::Rate rate(5);

  // frontier voxels display
  ros::Publisher markerArrayPub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  // frontier cluster display
  ros::Publisher cluster_pub = nh.advertise<visualization_msgs::Marker>("cluster_pub", 10);
  // ego planner input: target pose
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  bool goal_exec = false;
  int id_exec = 0;
  vector<geometry_msgs::PoseStamped> goal_list;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  std_msgs::ColorRGBA color;
  color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
  marker.color = color;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  set<QuadMesh> frontiers;

  while(ros::ok()){

    // lookup transform of 3 axises
    geometry_msgs::PointStamped cam_o_in_cam;
    cam_o_in_cam.header.frame_id = "uav0_camera_depth_frame";
    cam_o_in_cam.point.x = 0.0;
    cam_o_in_cam.point.y = 0.0;
    cam_o_in_cam.point.z = 0.0;

    geometry_msgs::PointStamped cam_o_in_map;

    try {
        // 使用lookupTransform函数查询坐标变换
        tf_buffer.transform(cam_o_in_cam, cam_o_in_map, "map");
    }
    catch (tf2::TransformException& ex) {
        ROS_ERROR("Failed to transform point: %s", ex.what());
    }

    //// detect frontier: 
    //// input = octomap; current pose; FOV; max range; region bbx
    //// output = a set containing all frontier voxels

    visualization_msgs::MarkerArray markerArray;

    ros::Time current_time = ros::Time::now();

    frontier_detect(frontiers, ocmap, cam_o_in_map, sensor_range);

    vector<vector<visualization_msgs::Marker>> all_marker(3);
    if(!frontiers.empty()){
      int total_type = 0;
      vector<unordered_map<double, int>> marker_type(3);
      auto it = frontiers.begin();
      for(int i = 0; i < frontiers.size(); i++){
        geometry_msgs::Vector3 scale;
        int flag = -1;
        if(it->normal.x() > 0.5 || it->normal.x() < -0.5){
          flag = 0;
          scale.x = resolution;
          scale.y = it->size;
          scale.z = it->size;
        }
        if(it->normal.y() > 0.5 || it->normal.y() < -0.5){
          flag = 1;
          scale.x = it->size;
          scale.y = resolution;
          scale.z = it->size;
        }
        if(it->normal.z() > 0.5 || it->normal.z() < -0.5){
          flag = 2;
          scale.x = it->size;
          scale.y = it->size;
          scale.z = resolution;
        }

        if(flag > -1){
          if(marker_type[flag].find(it->size) == marker_type[flag].end()){
            marker_type[flag][it->size] = marker_type[flag].size();
            all_marker[flag].resize(marker_type[flag].size(), marker);
            all_marker[flag][marker_type[flag][it->size]].id = marker_type[flag][it->size] + flag * 100;
            all_marker[flag][marker_type[flag][it->size]].scale = scale;
          }
          geometry_msgs::Point point_pose;
          point_pose.x = it->center.x();
          point_pose.y = it->center.y();
          point_pose.z = it->center.z();
          all_marker[flag][marker_type[flag][it->size]].points.push_back(point_pose);
        }
        it++;
      }
    }

    for(auto& i:all_marker){
      for(auto& j:i){
        markerArray.markers.push_back(j);
      }
    }
    
    ros::Duration elapsed_time = ros::Time::now() - current_time;
    cout << "Elapsed Time: " << elapsed_time.toSec()*1000.0 << " ms" << endl;

    cout << "frontier voxel num is : " << frontiers.size() << endl;

    markerArrayPub.publish(markerArray);

    //// frontier clustering
    //// input = the set containing all frontier voxels
    //// output = a set containing all frontier clusters

    if(!frontiers.empty() && goal_exec == false)
    {
      current_time = ros::Time::now();
      int cluster_num = 10;


      struct timeval t1{},t2{};
      double timeuse;
      // convert set to matrix
      Eigen::Matrix3Xf frontier_mat(3, frontiers.size());
      auto it = frontiers.begin();
      for(int i = 0; i < frontiers.size(); i++){
        frontier_mat.col(i) << it->center.x(), it->center.y(), it->center.z();
        it++;
      }
      
      int min = 0, max = frontiers.size()-1;
      random_device seed;//硬件生成随机数种子
      ranlux48 engine(seed());//利用种子生成随机数引擎
      uniform_int_distribution<int> distrib(min, max);//设置随机数范围，并为均匀分布
      
      // initial clusters' center
      vector<int> initial_idx;
      while(initial_idx.size() < cluster_num){
        int random = distrib(engine);
        if(find(initial_idx.begin(), initial_idx.end(), random) == initial_idx.end()){
          initial_idx.push_back(random);
        }
      }
      Eigen::Matrix3Xf cluster_center(3, cluster_num);
      for(int i = 0; i < initial_idx.size(); i++){
        cluster_center.col(i) = frontier_mat.col(initial_idx[i]);
      }

      Eigen::MatrixXf distance_to_center(cluster_num, frontiers.size());
      for(int iter_count = 0; iter_count < 1000; iter_count++){
        // cout << "[LOG] cal dis.";
        // gettimeofday(&t1,nullptr);

        // calculate the distance from each point to all centers
        for(int i = 0; i < cluster_num; i++){
            distance_to_center.row(i) = (cluster_center.col(i).replicate(1, frontier_mat.cols()) - frontier_mat).colwise().norm();
        }
        // gettimeofday(&t2,nullptr);
        // timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
        // cout<<" using time = " << timeuse << " s" << endl;  //(in sec)

        // cout << "[LOG] cal new center.";
        // gettimeofday(&t1,nullptr);
        vector<int> cluster_count(cluster_num, 0);
        Eigen::Matrix3Xf new_center(3, cluster_num);
        new_center.setZero();
        for(int i = 0; i < distance_to_center.cols(); i++){
          int idx = 0;
          distance_to_center.col(i).minCoeff(&idx);
          cluster_count[idx]++;
          new_center.col(idx) += frontier_mat.col(i);
        }

        for(int i = 0; i < new_center.cols(); i++){
          new_center.col(i) /= cluster_count[i];
        }
        // gettimeofday(&t2,nullptr);
        // timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
        // cout<<" using time = " << timeuse << " s" << endl;  //(in sec)

        // cout << "[LOG] whether or not to break.";
        // gettimeofday(&t1,nullptr);
        float delta = (new_center - cluster_center).colwise().norm().maxCoeff();
        if(delta < 1e-3){
          cout << "break after iteration " << iter_count << endl;
          break;
        }
        else{
          cluster_center = new_center;
        }
        // gettimeofday(&t2,nullptr);
        // timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
        // cout<<" using time = " << timeuse << " s" << endl;  //(in sec)
      }
      
      if(!goal_list.empty()){
        goal_list.clear();
      }

      visualization_msgs::Marker cluster(marker);
      cluster.type = visualization_msgs::Marker::SPHERE_LIST;
      cluster.color.a = 1; cluster.color.r = 0; cluster.color.g = 1; cluster.color.b = 0;
      cluster.scale.x = 0.5; cluster.scale.y = 0.5; cluster.scale.z = 0.5;
      for(int i = 0; i < cluster_center.cols(); i++){
        geometry_msgs::Point point;
        point.x = cluster_center.col(i).x();
        point.y = cluster_center.col(i).y();
        point.z = cluster_center.col(i).z();
        cluster.points.push_back(point);
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "map";
        target_pose.header.stamp = ros::Time::now();
        target_pose.pose.position.x = cluster_center.col(i).x();
        target_pose.pose.position.y = cluster_center.col(i).y();
        target_pose.pose.position.z = cluster_center.col(i).z();
        target_pose.pose.orientation.w = 1;
        goal_list.push_back(target_pose);
      }
      cluster_pub.publish(cluster);
      id_exec = 0;
      goal_exec = true;

      ros::Duration elapsed_time = ros::Time::now() - current_time;
      cout << "Elapsed Time: " << elapsed_time.toSec()*1000.0 << " ms" << endl;
    }

    if(goal_exec){
      if(id_exec >= goal_list.size()){
        id_exec = 0;
        goal_exec = false;
      }
      else{
        Eigen::Vector3f delta_pose;
        // WARNING: 此处减去的应该为base_link的pose
        delta_pose << goal_list[id_exec].pose.position.x - cam_o_in_map.point.x,
        goal_list[id_exec].pose.position.y - cam_o_in_map.point.y,
        goal_list[id_exec].pose.position.z - cam_o_in_map.point.z;
        if(delta_pose.norm() < 1.0){
          id_exec++;
        }
        else{
          goal_pub.publish(goal_list[id_exec]);
        }
      }
    }
    
    // generate view point
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
