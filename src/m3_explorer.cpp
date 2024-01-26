#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeIterator.hxx>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <vector>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <sys/time.h>

using namespace std;
octomap::OcTree* my_map;

void octomap_cb(const octomap_msgs::Octomap::ConstPtr& msg)
{
  // free memory for old map
  delete my_map;
  my_map = dynamic_cast<octomap::OcTree*>(msgToMap(*msg));

  double i = 0.0, j = 0.0, k = 0.0;
  my_map->getMetricMax(i, j, k);
  cout << "max: " << i << ", " << j << ", " << k << endl;
  my_map->getMetricMin(i, j, k);
  cout << "min: " << i << ", " << j << ", " << k << endl;
  my_map->getMetricSize(i, j, k);
  cout << "size: " << i << ", " << j << ", " << k << endl << endl;

  // for(octomap::OcTree::tree_iterator it = my_map->begin_tree(), end = my_map->end_tree(); it!= end; ++it)
  // {
  //   if(it.getSize() > 1.0){
  //   //manipulate node, e.g.:
  //   std::cout << "Node center: " << it.getCoordinate() << std::endl;
  //   std::cout << "Node size: " << it.getSize() << std::endl;
  //   std::cout << "Node value: " << it->getValue() << std::endl;
  //   std::cout << "Node depth: " << it.getDepth() << std::endl;
  //   }
  // }

  // for(octomap::OcTree::leaf_iterator it = my_map->begin_leafs(),
  //      end=my_map->end_leafs(); it!= end; ++it)
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

  // for(octomap::OcTree::leaf_bbx_iterator it = my_map->begin_leafs_bbx(min,max),
  //      end=my_map->end_leafs_bbx(); it!= end; ++it)
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
  ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/uav0/octomap_full", 1, octomap_cb);
  double resolution = 0.1, sensor_range = 5.0;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  ros::Duration(1.0).sleep();  // 等待tf2变换树准备好
  ros::Rate rate(5);

  ros::Publisher markerArrayPub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  geometry_msgs::Vector3 scale;
  scale.x = 0.02;
  scale.y = 0.02;
  scale.z = 0.02;
  marker.scale = scale;
  std_msgs::ColorRGBA color;
  color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
  marker.color = color;
  marker.type = visualization_msgs::Marker::CUBE;

  while(ros::ok()){
    //// detect frontier: 
    //// input = octomap; current pose; FOV; max range; region bbx
    //// output = a vector containing all frontier voxels

    // check old frontier

    // add new frontier
    // lookup transform of 3 axises
    vector<Eigen::Vector3d> frontiers;

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

    vector<geometry_msgs::PointStamped> source_point_vec;
    vector<Eigen::Vector3d> target_point_vec;
    geometry_msgs::PointStamped x_axis(cam_o_in_cam);
    x_axis.point.x = 1.0;
    x_axis.point.y = 0.0;
    x_axis.point.z = 0.0;
    geometry_msgs::PointStamped y_axis(cam_o_in_cam);
    y_axis.point.x = 0.0;
    y_axis.point.y = 1.0;
    y_axis.point.z = 0.0;
    geometry_msgs::PointStamped z_axis(cam_o_in_cam);
    z_axis.point.x = 0.0;
    z_axis.point.y = 0.0;
    z_axis.point.z = 1.0;
    source_point_vec.push_back(x_axis);
    source_point_vec.push_back(y_axis);
    source_point_vec.push_back(z_axis);

    for(int i = 0; i < source_point_vec.size(); i++){
      try {
        geometry_msgs::PointStamped target_point;
        tf_buffer.transform(source_point_vec[i], target_point, "map");
        target_point.point.x -= cam_o_in_map.point.x;
        target_point.point.y -= cam_o_in_map.point.y;
        target_point.point.z -= cam_o_in_map.point.z;
        Eigen::Vector3d target;
        target << target_point.point.x, target_point.point.y, target_point.point.z;
        target_point_vec.push_back(target);
      }
      catch (tf2::TransformException& ex) {
          ROS_ERROR("Failed to transform point: %s", ex.what());
      }
    }

    visualization_msgs::MarkerArray markerArray;
    int voxel_num = sensor_range/resolution;
    int id = 0;

    ros::Time current_time = ros::Time::now();
    // check whether the point is frontier
    for(int dx = -voxel_num; dx <= voxel_num; dx++){

      if(target_point_vec.size() != 3){
        cout << "[EROOR] tf vector wrong size: " << target_point_vec.size() << endl;
        break;
      }
      if(my_map == nullptr){
        cout << "[ERROR] the ptr of octomap is null" << endl;
        break;
      }

      for(int dy = -voxel_num; dy <= voxel_num; dy++){
        for(int dz = 0; dz <= voxel_num; dz++){
          Eigen::Vector3d point;
          point << cam_o_in_map.point.x, cam_o_in_map.point.y, cam_o_in_map.point.z;
          point += dx * resolution * target_point_vec[0] + dy * resolution * target_point_vec[1] + dz * resolution * target_point_vec[2];

          octomap::OcTreeNode* node = my_map->search(point(0), point(1), point(2));

          if(node != nullptr){
            continue;
          }
          else{
            vector<vector<int>> nbr_vec{{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
            for(vector<int> &item:nbr_vec){
              Eigen::Vector3d nbr_point = point + item[0] * resolution * target_point_vec[0] + item[1] * resolution * target_point_vec[1] + item[2] * resolution * target_point_vec[2];
              octomap::OcTreeNode* nbr_node = my_map->search(nbr_point(0), nbr_point(1), nbr_point(2));
              if(nbr_node != nullptr){
                    id++;
                    frontiers.push_back(point);
                    marker.id = id;
                    marker.pose.position.x = point(0);
                    marker.pose.position.y = point(1);
                    marker.pose.position.z = point(2);
                    markerArray.markers.push_back(marker);
              }
            }
          }
        }
      }
    }
    ros::Duration elapsed_time = ros::Time::now() - current_time;
    cout << "Elapsed Time: " << elapsed_time.toSec()*1000.0 << " ms" << endl;


    cout << "frontier voxel num is : " << frontiers.size() << endl;

    markerArrayPub.publish(markerArray);

    //// frontier clustering
    //// input = the vector containing all frontier voxels
    //// output = a vector containing all frontier clusters

    // generate view point
    ros::spinOnce();
    // rate.sleep();
  }
  return 0;
}
