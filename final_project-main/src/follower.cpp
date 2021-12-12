/**
 * @file follower.cpp
  * @author Jerry Pittman, Jr., Nicholas Novak, Orlandis Smith
 *  (jpittma1@umd.edu, nnovak@umd.edu, osmith15@umd.edu)
 * Group 5
 * @brief 
 * @version 0.1
 * @date 2021-12-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "../include/follower/follower.h"
#include "../include/bot_controller/bot_controller.h"

Follower::Follower(ros::NodeHandle* nodehandle, const std::string& robot_name) :
    Bot_Controller(nodehandle, robot_name){
    std::array <int, 4> m_fid{};
    ROS_INFO("Follower Constructor called");
    m_initialize_subscribers();
    m_initialize_publishers();
}