#include "m3_explorer/frontier_detector.h"
void frontier_detect(set<QuadMesh>& frontiers, octomap::OcTree* ocmap, const geometry_msgs::PointStamped& cur_pose, const double& sensor_range)
{
    // check old frontier
    if(!frontiers.empty() && ocmap != nullptr){
      auto it = frontiers.begin();
      while(it != frontiers.end()){
        if(abs(it->center.x() - cur_pose.point.x) > sensor_range){
          it++;
          continue;
        }
        if(abs(it->center.y() - cur_pose.point.y) > sensor_range){
          it++;
          continue;
        }

        if(ocmap->search(it->center) != nullptr){
          frontiers.erase(it++);
        }
        else{
          it++;
        }
      }
    }

    // add new frontier
    // check whether the point is frontier
    octomap::point3d_collection nbr_dir = {{1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0},
                                    {0.0, 1.0, 0.0}, {0.0, -1.0, 0.0},
                                    {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}};
    octomap::point3d_collection offset = {{0.0, 1.0, 1.0}, {0.0, -1.0, 1.0}, {0.0, -1.0, -1.0}, {0.0, 1.0, -1.0},
                                      {1.0, 0.0, 1.0}, {-1.0, 0.0, 1.0}, {-1.0, 0.0, -1.0}, {1.0, 0.0, -1.0},
                                      {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}, {-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}};

    octomap::point3d check_bbx_min(cur_pose.point.x - sensor_range, cur_pose.point.y - sensor_range, max(1.0, cur_pose.point.z - sensor_range));
    octomap::point3d check_bbx_max(cur_pose.point.x + sensor_range, cur_pose.point.y + sensor_range, min(2.0, cur_pose.point.z + sensor_range));

    if(ocmap == nullptr){
      cout << "[ERROR] the ptr of octomap is null" << endl;
    }
    else{
      for(octomap::OcTree::leaf_bbx_iterator it = ocmap->begin_leafs_bbx(check_bbx_min, check_bbx_max),
       end=ocmap->end_leafs_bbx(); it!= end; ++it)
      {
        double size = it.getSize();
        int depth = it.getDepth();
        octomap::point3d center = it.getCoordinate();
        
        // no need to check obstacles' neighbors
        // no need to check the interior of obstacle
        if(it->getValue() > log(0.6/0.4)){
          continue;
        }

        // check 6 faces
        for(int i = 0; i < nbr_dir.size(); i++){
          octomap::point3d nbr_point = center + nbr_dir[i] * size;
          octomap::OcTreeNode* nbr_node = ocmap->search(nbr_point, depth);
          QuadMesh mesh;
          if(nbr_node == nullptr){
            mesh.center = center + nbr_dir[i] * (size / 2.0);
            mesh.normal = nbr_dir[i];
            mesh.size = size;
            frontiers.insert(mesh);
          }
          else{
            if(nbr_node->hasChildren()){
              // bfs search unknown voxel
              queue<pair<octomap::point3d, int>> bfs_queue;
              octomap::point3d surface = center + nbr_dir[i] * (size / 2.0 + ocmap->getResolution() / 2.0);
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
                  frontiers.insert(mesh);
                }
              }
            }
          }
        }
      }
    }
}
