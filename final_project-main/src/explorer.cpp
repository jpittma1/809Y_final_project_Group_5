

#include "../include/bot_controller/bot_controller.h"
#include "../include/explorer/explorer.h"

#include <bot_msgs/BotStatus.h>
#include <geometry_msgs/Twist.h>   //for geometry_msgs::Twist
#include <nav_msgs/Odometry.h>     //for nav_msgs::Odometry
#include <sensor_msgs/LaserScan.h> //for laser scans
#include <ros/ros.h>
#include <utility>
#include <tf/transform_datatypes.h> //to manipulate quaternions
#include <geometry_msgs/Twist.h>   //for geometry_msgs::Twist
#include <iostream>
#include "../param/aruco_lookup.yaml"

Explorer::Explorer(ros::NodeHandle* nodehandle const std::string& robot_name) : Bot_Controller(nodehandle, robot_name)
{
    m_initialize_publishers();
    m_initialize_subscribers();
    // start_place = Explorer::get_start_loc();
    
}
std::array<double,2> get_start_loc(){
    std::array<double,2> start_loc;
    start_loc[0] = get_current_x();
    start_loc[1] = get_current_y();
    return start_loc;
};
std::array<std::array<double,2>,4> Explorer::get_goals(){
  ros::NodeHandle nh;

  std::array<std::array<double,2>,4> exp_goals;
  XmlRpc::XmlRpcValue goal_list;
  nh.getParam("target_1", goal_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  
  exp_goal[0][0] = goal_list[0];
  exp_goal[0][1] = goal_list[1];

  nh.getParam("target_2", goal_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  
  exp_goal[1][0] = goal_list[0];
  exp_goal[1][1] = goal_list[1];

  nh.getParam("target_3", goal_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  
  exp_goal[2][0] = goal_list[0];
  exp_goal[2][1] = goal_list[1];

  nh.getParam("target_4", goal_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  
  exp_goal[3][0] = goal_list[0];
  exp_goal[3][1] = goal_list[1];
  return exp_goals;

}
