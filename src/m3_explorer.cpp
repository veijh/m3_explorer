#include <ros/ros.h>
#include <vector>
#include <unordered_map>
#include <set>
#include <queue>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <sys/time.h>
#include <utility>
#include <cmath>
#include <random>
#include "m3_explorer/frontier_detector.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

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
  // cout << "size: " << i << ", " << j << ", " << k << endl << endl;

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

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

geometry_msgs::PoseStamped cur_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  cur_pos = *msg;
}

void offboard_takeoff(ros::NodeHandle& nh, const double& height){
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
          ("/uav0/mavros/state", 10, state_cb);
  ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
          ("/uav0/mavros/local_position/pose", 10, pos_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
          ("/uav0/mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
          ("/uav0/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
          ("/uav0/mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // wait for FCU connection
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("mavros connected");

  // get takeoff position
  ros::Time start_time = ros::Time::now();
  int count = 0;
  geometry_msgs::PoseStamped takeoff_pos;
  ROS_INFO("reading takeoff position");
  while(ros::Time::now() - start_time < ros::Duration(2.0)){
    ros::spinOnce();
    count++;
    takeoff_pos.pose.position.x += cur_pos.pose.position.x;
    takeoff_pos.pose.position.y += cur_pos.pose.position.y;
    takeoff_pos.pose.position.z += cur_pos.pose.position.z;
    rate.sleep();
  }
  takeoff_pos.pose.position.x /= count;
  takeoff_pos.pose.position.y /= count;
  takeoff_pos.pose.position.z /= count;
  ROS_INFO("takeoff position: %.2lf, %.2lf, %.2lf", takeoff_pos.pose.position.x, takeoff_pos.pose.position.y, takeoff_pos.pose.position.z);

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = takeoff_pos.pose.position.x;
  pose.pose.position.y = takeoff_pos.pose.position.y;
  pose.pose.position.z = takeoff_pos.pose.position.z + height;

  //send a few setpoints before starting
  for(int i = 5; ros::ok() && i > 0; --i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  while(abs(cur_pos.pose.position.z - pose.pose.position.z) > 0.1){
    if(current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(1.0))){
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
    } else {
        if(!current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
    }

    local_pos_pub.publish(pose);

    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("takeoff done");
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
  ros::Publisher frontier_maker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  // frontier cluster display
  ros::Publisher cluster_pub = nh.advertise<visualization_msgs::Marker>("cluster_pub", 10);
  // ego planner input: target pose
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  bool goal_exec = false;
  int id_exec = 0;
  vector<geometry_msgs::PoseStamped> goal_list;

  // marker template
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

  // frontiers
  set<QuadMesh> frontiers;

  // switch to offboard mode && takeoff to desired height
  offboard_takeoff(nh, 1.5);

  while(ros::ok()){
    ros::spinOnce();

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

    ros::Time current_time = ros::Time::now();

    frontier_detect(frontiers, ocmap, cam_o_in_map, sensor_range);
    // frontier_visualize(frontiers, 0.1, frontier_maker_array_pub);
    
    ros::Duration elapsed_time = ros::Time::now() - current_time;
    cout << "frontier detect using time: " << elapsed_time.toSec()*1000.0 << " ms" << endl;
    cout << "frontier voxel num is : " << frontiers.size() << endl;

    //// frontier clustering
    //// input = the set containing all frontier voxels
    //// output = a set containing all frontier clusters

    if(!frontiers.empty() && goal_exec == false)
    {
      current_time = ros::Time::now();
      int cluster_num = frontiers.size() / 500;

      // convert set to matrix
      Eigen::Matrix3Xf frontier_center_mat(3, frontiers.size());
      Eigen::Matrix3Xf frontier_normal_mat(3, frontiers.size());
      Eigen::RowVectorXf frontier_size_mat(frontiers.size());
      auto it = frontiers.begin();
      for(int i = 0; i < frontiers.size(); i++){
        frontier_center_mat.col(i) << it->center.x(), it->center.y(), it->center.z();
        frontier_normal_mat.col(i) << it->normal.x(), it->normal.y(), it->normal.z();
        frontier_size_mat.col(i) << it->size; 
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
      Eigen::Matrix3Xf cluster_normal(3, cluster_num);
      for(int i = 0; i < initial_idx.size(); i++){
        cluster_center.col(i) = frontier_center_mat.col(initial_idx[i]);
      }

      Eigen::MatrixXf distance_to_center(cluster_num, frontiers.size());
      for(int iter_count = 0; iter_count < 1000; iter_count++){
        // calculate the distance from each point to all centers
        for(int i = 0; i < cluster_num; i++){
            distance_to_center.row(i) = (cluster_center.col(i).replicate(1, frontier_center_mat.cols()) - frontier_center_mat).colwise().norm();
        }

        vector<double> cluster_weight(cluster_num, 0);
        Eigen::Matrix3Xf new_center(3, cluster_num);
        new_center.setZero();
        cluster_normal.setZero();
        for(int i = 0; i < distance_to_center.cols(); i++){
          int idx = 0;
          distance_to_center.col(i).minCoeff(&idx);
          double weight = frontier_size_mat(i) * frontier_size_mat(i);
          cluster_weight[idx] += weight;
          new_center.col(idx) += weight * frontier_center_mat.col(i);
          cluster_normal.col(idx) += weight * frontier_normal_mat.col(i);
        }

        for(int i = 0; i < new_center.cols(); i++){
          new_center.col(i) /= cluster_weight[i];
          cluster_normal.col(i) /= cluster_weight[i];
        }

        float delta = (new_center - cluster_center).colwise().norm().maxCoeff();
        if(delta < 1e-3){
          cout << "break after iteration " << iter_count << endl;
          break;
        }
        else{
          cluster_center = new_center;
        }
      }
      
      if(!goal_list.empty()){
        goal_list.clear();
      }

      visualization_msgs::Marker marker_cluster_center(marker);
      marker_cluster_center.type = visualization_msgs::Marker::SPHERE_LIST;
      marker_cluster_center.color.a = 1; marker_cluster_center.color.r = 0; marker_cluster_center.color.g = 1; marker_cluster_center.color.b = 0;
      marker_cluster_center.scale.x = 0.2; marker_cluster_center.scale.y = 0.2; marker_cluster_center.scale.z = 0.2;
      for(int i = 0; i < cluster_center.cols(); i++){
        geometry_msgs::Point point;
        point.x = cluster_center.col(i).x();
        point.y = cluster_center.col(i).y();
        point.z = cluster_center.col(i).z();
        marker_cluster_center.points.push_back(point);
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "map";
        target_pose.header.stamp = ros::Time::now();
        target_pose.pose.position.x = cluster_center.col(i).x();
        target_pose.pose.position.y = cluster_center.col(i).y();
        target_pose.pose.position.z = cluster_center.col(i).z();
        target_pose.pose.orientation.x = cluster_normal.col(i).normalized().x();
        target_pose.pose.orientation.y = cluster_normal.col(i).normalized().y();
        target_pose.pose.orientation.z = cluster_normal.col(i).normalized().z();
        target_pose.pose.orientation.w = 1;
        goal_list.push_back(target_pose);
      }
      cluster_pub.publish(marker_cluster_center);

      // visualize normal vector
      visualization_msgs::Marker marker_cluster_normal(marker);
      marker_cluster_normal.type = visualization_msgs::Marker::ARROW;
      marker_cluster_normal.color.a = 1; marker_cluster_normal.color.r = 0; marker_cluster_normal.color.g = 1; marker_cluster_normal.color.b = 0;
      marker_cluster_normal.scale.x = 0.1; marker_cluster_normal.scale.y = 0.2; marker_cluster_normal.scale.z = 0.2;
      for(int i = 0; i < cluster_normal.cols(); i++){
        marker_cluster_normal.points.clear();
        marker_cluster_normal.id = 200+i;
        geometry_msgs::Point point;
        point.x = cluster_center.col(i).x();
        point.y = cluster_center.col(i).y();
        point.z = cluster_center.col(i).z();
        marker_cluster_normal.points.push_back(point);
        point.x = cluster_center.col(i).x() + cluster_normal.col(i).normalized().x()*1.0;
        point.y = cluster_center.col(i).y() + cluster_normal.col(i).normalized().y()*1.0;
        point.z = cluster_center.col(i).z() + cluster_normal.col(i).normalized().z()*1.0;
        marker_cluster_normal.points.push_back(point);
        cluster_pub.publish(marker_cluster_normal);
      }

      id_exec = 0;
      goal_exec = true;

      ros::Duration elapsed_time = ros::Time::now() - current_time;
      cout << "Elapsed Time: " << elapsed_time.toSec()*1000.0 << " ms" << endl;
    }

    //// goal generation
    if(goal_exec){
      if(id_exec >= goal_list.size()){
        id_exec = 0;
        goal_exec = false;
      }
      else{
        cout << "[INFO] exe id " << id_exec << endl;
        // generate view point
        octomap::point3d center(goal_list[id_exec].pose.position.x, goal_list[id_exec].pose.position.y, goal_list[id_exec].pose.position.z);
        octomap::point3d offset(goal_list[id_exec].pose.orientation.x, goal_list[id_exec].pose.orientation.y, goal_list[id_exec].pose.orientation.z);
        vector<octomap::point3d> view_point_candidate;
        // raycast
        for(double len = 0.3; len < 2.0; len += 0.3){
          octomap::point3d view_point = center - offset * len;
          octomap::OcTreeNode* view_node = ocmap->search(view_point);
          if(view_node == nullptr){
            continue;
          }
          else{
            if(view_node->getOccupancy() > 0.75){
              continue;
            }
          }
          bool no_collision = true;
          // 是否可达
          for(double x_offset = -0.5; x_offset <= 0.5 && no_collision; x_offset += 0.1){
            for(double y_offset = -0.5; y_offset <= 0.5 && no_collision; y_offset += 0.1){
              octomap::point3d check_point(center);
              check_point.x() = view_point.x() + x_offset;
              check_point.y() = view_point.y() + y_offset;
              octomap::OcTreeNode* check_node = ocmap->search(check_point);
              // 不可达
              if(check_node != nullptr && check_node->getOccupancy() > 0.75){
                no_collision = false;
              }
            }
          }
          if(no_collision){
            view_point_candidate.push_back(view_point);
          }
          else{
            break;
          }
        }

        if(view_point_candidate.empty()){
          id_exec++;
        }
        else{
          geometry_msgs::PoseStamped goal(goal_list[id_exec]);
          goal.pose.position.x = (view_point_candidate.begin() + view_point_candidate.size()/2)->x();
          goal.pose.position.y = (view_point_candidate.begin() + view_point_candidate.size()/2)->y();
          goal.pose.position.z = (view_point_candidate.begin() + view_point_candidate.size()/2)->z();

          Eigen::Vector3f delta_pose;
          // WARNING: 此处减去的应该为base_link的pose
          delta_pose << goal.pose.position.x - cam_o_in_map.point.x,
          goal.pose.position.y - cam_o_in_map.point.y,
          goal.pose.position.z - cam_o_in_map.point.z;
          
          if(delta_pose.norm() < 1.0){
            ros::Duration(2.0).sleep();
            id_exec++;
          }
          else{
            goal_pub.publish(goal);
          }
        }

      }
    }
    
    // generate view point

    // path
    rate.sleep();
  }
  return 0;
}
