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
#include "m3_explorer/frontier_cluster.h"
#include "m3_explorer/path_planning.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "lkh_ros/Solve.h"

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
  ros::Publisher frontier_maker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("frontier", 10);
  // frontier cluster display
  ros::Publisher cluster_pub = nh.advertise<visualization_msgs::MarkerArray>("cluster", 10);
  // view point display
  ros::Publisher view_point_pub = nh.advertise<geometry_msgs::PoseArray>("view_point", 10);
  // lkh client
  ros::ServiceClient lkh_client = nh.serviceClient<lkh_ros::Solve>("lkh_solve");
  string problem_path;
  if(!nh.getParam("Problem_Path", problem_path)){
    rate.sleep();
  }
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
  geometry_msgs::PoseArray explore_path;
  int path_id = 0;

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
    frontier_visualize(frontiers, 0.1, frontier_maker_array_pub);
    
    ros::Duration elapsed_time = ros::Time::now() - current_time;
    cout << "frontier detect using time: " << elapsed_time.toSec()*1000.0 << " ms" << endl;
    cout << "frontier voxel num is : " << frontiers.size() << endl;

    vector<Cluster> cluster_vec;
    geometry_msgs::PoseArray vp_array;

    if(!frontiers.empty() && goal_exec == false)
    {
      current_time = ros::Time::now();
      //// frontier clustering
      //// input = the set containing all frontier voxels
      //// output = a vector containing all frontier clusters
      cluster_vec = k_mean_cluster(frontiers);
      cluster_visualize(cluster_vec, cluster_pub);

      cout << "frontier clusters generation using time: " << (ros::Time::now() - current_time).toSec()*1000.0 << " ms" << endl;

      current_time = ros::Time::now();
      //// generate view point
      //// input = a vector containing all frontier clusters
      //// output = a vector containing poses of all view points
      vp_array = view_point_generate(cluster_vec, ocmap);
      view_point_pub.publish(vp_array);

      cout << "view points generation using time: " << (ros::Time::now() - current_time).toSec()*1000.0 << " ms" << endl;

      current_time = ros::Time::now();
      //// path planning
      //// input = current pose of uav && a vector containing poses of all view points
      //// output = a vector containing the sequence of waypoints
      explore_path = atsp_path(cam_o_in_map, vp_array, lkh_client, problem_path);

      cout << "path planning using time: " << (ros::Time::now() - current_time).toSec()*1000.0 << " ms" << endl;

      goal_exec = true;
    }

    if(goal_exec){
      if(path_id >= explore_path.poses.size()){
        path_id = 0;
        goal_exec = false;
      }
      else{
        cout << "path_id : " << path_id << endl;
        Eigen::Vector2f delta(cam_o_in_map.point.x - explore_path.poses[path_id].position.x, cam_o_in_map.point.y - explore_path.poses[path_id].position.y);
        if(delta.norm() < 0.5){
          path_id++;
        }
        else{
          geometry_msgs::PoseStamped target_pose;
          target_pose.header.frame_id = "map";
          target_pose.pose = explore_path.poses[path_id];
          goal_pub.publish(target_pose);
        }
      }
    }
    
    rate.sleep();
  }
  return 0;
}
