#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"

#include <iostream>
#include <cmath>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <angles/angles.h>

#include <sstream>

/*********************************************************************************
                                  GLOBAL VARIABLES
*********************************************************************************/

std::vector<std::pair<float, float>> path;

bool new_path = false;

bool gotMap = false;

float map_resolution = 0.0;
std::pair<float, float> map_origin;
std::pair<int, int> map_size;
std::vector<int8_t> map_data;

std::pair<float, float> world_pose;
std::pair<int, int> map_pose;

geometry_msgs::PoseStamped global_goal;


/*********************************************************************************
                                      CALLBACKS
*********************************************************************************/

void readGlobalPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  path.clear();

  new_path = true;
  std::pair<float, float> aux;

  aux.first = 0.0;
  aux.second = 0.0;
  for(int i = 1 ; i < msg->poses.size() ; i++)
  {
    aux.first = msg->poses[i].pose.position.x;
    aux.second = msg->poses[i].pose.position.y;

    path.insert(path.end(), aux); 
    
  }  
  
}

void readMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{

  bool new_costmap = false;

  /*for(int i = 0 ; i < map_data.size() ; i++)
  {
    if(map_data[i] != msg->data[i])
    {
      new_costmap = true;
      ROS_INFO("______________NEW COSTMAP");
      break;
    }
  }*/

  if(!new_costmap)
  {
    //ROS_INFO("______________RENEW COSTMAP");
    map_data.clear();

    map_resolution = msg->info.resolution;
    map_size.first = msg->info.width;
    map_size.second = msg->info.height;
    map_origin.first = msg->info.origin.position.x;
    map_origin.second = msg->info.origin.position.y;

    for(int i = 0 ; i < msg->data.size() ; i++)
    {
      map_data.push_back(msg->data[i]);
    }

    gotMap = true;
  }

  /*ROS_INFO("Tamany mapa: %i", map_data.size());
  ROS_INFO("Resolution: %f", map_resolution);
  ROS_INFO("Width: %i", map_size.first);
  ROS_INFO("Height: %i", map_size.second);
  ROS_INFO("X: %f  Y: %f", map_origin.first, map_origin.second);
  ROS_INFO("Orientació:   X: %f  Y: %f Z: %f  W: %f", msg->info.origin.orientation.x, msg->info.origin.orientation.y, msg->info.origin.orientation.z, msg->info.origin.orientation.w);
  //ROS_INFO("DATA: %i", msg->data[23]); */
}

void readGlobalGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& newgoal)
{

  ROS_INFO("________________NEW_GOAL SENT");
  global_goal.header.frame_id = newgoal->header.frame_id;

  global_goal.pose.position.x = newgoal->pose.position.x;
  global_goal.pose.position.y = newgoal->pose.position.y;
  global_goal.pose.position.z = newgoal->pose.position.z;

  global_goal.pose.orientation.x = newgoal->pose.orientation.x;
  global_goal.pose.orientation.y = newgoal->pose.orientation.y;
  global_goal.pose.orientation.z = newgoal->pose.orientation.z;
  global_goal.pose.orientation.w = newgoal->pose.orientation.w;

  /*ROS_INFO("POSITION:");
  ROS_INFO("X: %f", global_goal.pose.position.x);
  ROS_INFO("Y: %f", global_goal.pose.position.y);
  ROS_INFO("Z: %f", global_goal.pose.position.z);

  ROS_INFO("ORIENTATION:");
  ROS_INFO("X: %f", global_goal.pose.orientation.x);
  ROS_INFO("Y: %f", global_goal.pose.orientation.y);
  ROS_INFO("Z: %f", global_goal.pose.orientation.z);
  ROS_INFO("W: %f", global_goal.pose.orientation.w); */
}



/*********************************************************************************
                                      FUNCTIONS
*********************************************************************************/

float distance(float px1, float py1, float px2, float py2)
  {
    float dist = sqrt((px1 - px2) * (px1 - px2) + (py1 - py2) * (py1 - py2));
    return dist;
  }

void mapToWorld(int mx, int my)
  {
    world_pose.first = map_origin.first + mx * map_resolution;
    world_pose.second = map_origin.second + my * map_resolution;
  }

void worldToMap(float wx, float wy)
  {
    float origin_x = map_origin.first, origin_y = map_origin.second;
    float resolution = map_resolution;

    map_pose.first = floor((wx - origin_x) / resolution);
    map_pose.second = floor((wy - origin_y) / resolution);
  }


bool collision(float x, float y)
  {
    worldToMap(x, y);

    /*if ((map_pose.first < 0) || (map_pose.second < 0) || (map_pose.first >= map_size.first) || (map_pose.second >= map_size.second))
    {
      return true;
    }*/

    int8_t cost = map_data[map_pose.first + map_pose.second * (map_size.first)];
    if (cost != 0 && cost != -1 && cost != 255) // free && unknown
    {
      return true;
    }

    return false;
  }

bool obstacleFree(float px1, float py1, float px2, float py2)
  {
    int m = 0;
    float theta;

    std::pair<float, float> p_m;

    p_m.first = 0.0;
    p_m.second = 0.0;

    float dist = distance(px1, py1, px2, py2);
    float resolution = map_resolution;
   
    int value = int(floor(dist / resolution));
    theta = atan2(py2 - py1, px2 - px1);

    for (int i = 0; i < value; i++)
    {

	p_m.first = px1 + m * (resolution)*cos(theta);
	p_m.second = py1 + m * (resolution)*sin(theta);

	if (collision(p_m.first, p_m.second))
  	   return false;

	m++;
     }
     return true;
  }

/*********************************************************************************
                                      MAIN
*********************************************************************************/
  

int main(int argc, char **argv)
{

  ros::init(argc, argv, "path_validator");

  
  ros::NodeHandle nh;


  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000); // /move_base_simple/goal

  ros::Subscriber actual_path = nh.subscribe("/move_base/DWAPlannerROS/global_plan", 1000, readGlobalPathCallback);

  ros::Subscriber map = nh.subscribe("/map", 1000, readMapCallback); //  /move_base/local_costmap/costmap

  ros::Subscriber actual_goal = nh.subscribe("/move_base_simple/goal", 1000, readGlobalGoalCallback);

  ros::Rate loop_rate(10);

  
  int count = 0;
  int count_aux = 0;

  while (ros::ok())
  {
    
    if(gotMap && /*count >= 60 + count_aux*/new_path)
    {

      count_aux = count;
      /*if(new_path){
        for(int i = 0; i < path.size() ; i++){
          //ROS_INFO("Pose %i: %fX , %fY", i, path[i].first, path[i].second);
        }
        new_path = false;
      }*/
      

      for(int i = 1; i < path.size() ; i++)
      {
        if(obstacleFree(path[i-1].first, path[i-1].second, path[i].first, path[i].second))
        {
          //ROS_INFO("Free");
        } else {
          ROS_INFO("Invalid Path. Planning Again");

          new_path = false;
          global_goal.header.stamp = ros::Time::now();
          goal_pub.publish(global_goal);

        }
        
      }

      gotMap = false;
    }  

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

