/**
 * @file explorer.cpp
 * @author 809Y Final Project Group 5
 * @brief Explorer Robot
 * @version 0.1
 * @date 2021-12-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "../include/bot_controller/bot_controller.h"
#include "../include/explorer/explorer.h"

Explorer::Explorer(ros::NodeHandle* nodehandle, const std::string& robot_name) :
    Bot_Controller(nodehandle, robot_name)
{
    m_initialize_publishers();
    m_initialize_subscribers();
    // start_place = Explorer::get_start_loc();
    
}
// std::array<double,2> get_start_loc(){
//     std::array<double,2> start_loc;
//     start_loc[0] = get_current_x();
//     start_loc[1] = get_current_y();
//     return start_loc;
// }
std::array<std::array<double,2>,4> get_goals(){
  ros::NodeHandle nh;

  std::array<std::array<double,2>,4> exp_goal;
  XmlRpc::XmlRpcValue goal_list;
  nh.getParam("target_1", goal_list);
  ROS_ASSERT(goal_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  
  exp_goal[0][0] = goal_list[0];
  exp_goal[0][1] = goal_list[1];

  nh.getParam("target_2", goal_list);
  ROS_ASSERT(goal_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  
  exp_goal[1][0] = goal_list[0];
  exp_goal[1][1] = goal_list[1];

  nh.getParam("target_3", goal_list);
  ROS_ASSERT(goal_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  
  exp_goal[2][0] = goal_list[0];
  exp_goal[2][1] = goal_list[1];

  nh.getParam("target_4", goal_list);
  ROS_ASSERT(goal_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  
  exp_goal[3][0] = goal_list[0];
  exp_goal[3][1] = goal_list[1];
  return exp_goal;

}
