#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <vector>
#include <queue>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <sys/time.h>
#include <utility>
#include <cmath>

using namespace std;
octomap::OcTree* ocmap;
typedef struct quad_mesh
{
  octomap::point3d center;
  double size;
  octomap::point3d normal;
}quad_mesh;

void octomap_cb(const octomap_msgs::Octomap::ConstPtr& msg)
{
  // free memory for old map
  delete ocmap;
  ocmap = dynamic_cast<octomap::OcTree*>(msgToMap(*msg));

  double i = 0.0, j = 0.0, k = 0.0;
  ocmap->getMetricMax(i, j, k);
  cout << "max: " << i << ", " << j << ", " << k << endl;
  ocmap->getMetricMin(i, j, k);
  cout << "min: " << i << ", " << j << ", " << k << endl;
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
  ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/uav0/octomap_full", 1, octomap_cb);
  double resolution = 0.05, sensor_range = 5.0;
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
  std_msgs::ColorRGBA color;
  color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 0.5;
  marker.color = color;
  marker.type = visualization_msgs::Marker::CUBE;

  while(ros::ok()){
    //// detect frontier: 
    //// input = octomap; current pose; FOV; max range; region bbx
    //// output = a vector containing all frontier voxels

    // check old frontier

    // add new frontier
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

    ros::Time current_time = ros::Time::now();
    // check whether the point is frontier
    // if(ocmap == nullptr){
    //   cout << "[ERROR] the ptr of octomap is null" << endl;
    // }
    // else{
    //   octomap::point3d pmin(0,0,0);
    //   octomap::point3d pmax(1,1,1);
    //   ocmap->getUnknownLeafCenters(unk_points, pmin, pmax);
    //   int id = 0;
    //   for(auto it = unk_points.begin(); it != unk_points.end(); it++){
    //     marker.id = id++;
    //     marker.pose.position.x = it->x();
    //     marker.pose.position.y = it->y();
    //     marker.pose.position.z = it->z();
    //     markerArray.markers.push_back(marker);
    //   }
    // }

    vector<quad_mesh> frontiers;
    octomap::point3d_collection nbr_dir = {{1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0},
                                    {0.0, 1.0, 0.0}, {0.0, -1.0, 0.0},
                                    {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}};
    octomap::point3d_collection offset = {{0.0, 1.0, 1.0}, {0.0, -1.0, 1.0}, {0.0, -1.0, -1.0}, {0.0, 1.0, -1.0},
                                      {1.0, 0.0, 1.0}, {-1.0, 0.0, 1.0}, {-1.0, 0.0, -1.0}, {1.0, 0.0, -1.0},
                                      {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}, {-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}};
    octomap::point3d min(-5, -5, 0.1);
    octomap::point3d max(5, 5, 3);
    if(ocmap == nullptr){
      cout << "[ERROR] the ptr of octomap is null" << endl;
    }
    else{
      for(octomap::OcTree::leaf_bbx_iterator it = ocmap->begin_leafs_bbx(min, max),
       end=ocmap->end_leafs_bbx(); it!= end; ++it)
      {
        double size = it.getSize();
        int depth = it.getDepth();
        octomap::point3d center = it.getCoordinate();
        
        // no need to check obstacles' neighbors
        if(it->getValue() > log(0.8/0.2)){
          continue;
        }

        // check 6 faces
        for(int i = 0; i < nbr_dir.size(); i++){
          octomap::point3d nbr_point = center + nbr_dir[i] * size;
          octomap::OcTreeNode* nbr_node = ocmap->search(nbr_point, depth);
          quad_mesh mesh;
          if(nbr_node == nullptr){
            mesh.center = center + nbr_dir[i] * (size / 2.0);
            mesh.normal = nbr_dir[i];
            mesh.size = size;
            frontiers.push_back(mesh);
          }
          else{
            if(nbr_node->hasChildren()){
              // bfs search unknown voxel
              queue<pair<octomap::point3d, int>> bfs_queue;
              octomap::point3d surface = center + nbr_dir[i] * ((size / 2.0) + ocmap->getResolution());
              bfs_queue.push(make_pair(surface, depth));

              while(!bfs_queue.empty()){
                octomap::point3d point = bfs_queue.front().first;
                int point_depth = bfs_queue.front().second;
                double point_size = size * pow(0.5, point_depth - depth);
                bfs_queue.pop();

                octomap::OcTreeNode* point_node = ocmap->search(point, point_depth);
                if(point_node != nullptr){
                    if(point_node->hasChildren()){
                      // add 4 points into queue
                      for(int offset_i = 0; offset_i < 4; offset_i++){
                        double child_size = point_size / 2.0;
                        octomap::point3d child = point + offset[offset_i + 4 * (i/2)] * (child_size / 2.0);
                        bfs_queue.push(make_pair(child, point_depth+1));
                      }
                    }
                }
                else{
                  mesh.center = point;
                  mesh.normal = nbr_dir[i];
                  mesh.size = point_size;
                  frontiers.push_back(mesh);
                }
              }
            }
          }
        }
      }
    }

    for(int i = 0; i < frontiers.size(); i++){
      marker.id = i;
      marker.pose.position.x = frontiers[i].center.x();
      marker.pose.position.y = frontiers[i].center.y();
      marker.pose.position.z = frontiers[i].center.z();
      geometry_msgs::Vector3 scale;
      if(frontiers[i].normal.x() > 0.5 || frontiers[i].normal.x() < -0.5){
        scale.x = resolution;
        scale.y = frontiers[i].size;
        scale.z = frontiers[i].size;
      }
      if(frontiers[i].normal.y() > 0.5 || frontiers[i].normal.y() < -0.5){
        scale.x = frontiers[i].size;
        scale.y = resolution;
        scale.z = frontiers[i].size;
      }
      if(frontiers[i].normal.z() > 0.5 || frontiers[i].normal.z() < -0.5){
        scale.x = frontiers[i].size;
        scale.y = frontiers[i].size;
        scale.z = resolution;
      }
      marker.scale = scale;
      markerArray.markers.push_back(marker);
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
    rate.sleep();
  }
  return 0;
}
