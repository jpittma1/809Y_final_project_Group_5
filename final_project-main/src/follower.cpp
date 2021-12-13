/**
 * @file follower.cpp
 * @author 809Y Final Project Group 5
 * @brief Follower Robot
 * @version 0.1
 * @date 2021-12-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "../include/bot_controller/bot_controller.h"
#include "../include/follower/follower.h"

Follower::Follower(ros::NodeHandle* nodehandle, const std::string& robot_name) :
    Bot_Controller(nodehandle, robot_name){
    // std::array <int, 4> m_fid{};
    ROS_INFO("Follower Constructor called");
    m_initialize_subscribers();
    m_initialize_publishers();
}