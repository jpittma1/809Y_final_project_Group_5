#include "../include/follower/follower.h"

Follower::Follower(ros::NodeHandle* nodehandle, const std::string& robot_name) :
    Bot_Controller(nodehandle, robot_name)
{
    ROS_INFO("Follower Constructor called");
    m_initialize_subscribers();
    m_initialize_publishers();
}
