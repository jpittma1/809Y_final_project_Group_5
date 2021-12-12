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
    Bot_Controller(nodehandle, robot_name)
{
    std::array <int, 4> m_fid{};
    ROS_INFO("Follower Constructor called");
    m_initialize_subscribers();
    m_initialize_publishers();
}

void Follower::m_initialize_subscribers() {
    ROS_INFO("Initializing Subscribers");
    m_pose_subscriber = m_nh.subscribe("/odom", 1000, &Bot_Controller::m_pose_callback, this);
    m_scan_subscriber = m_nh.subscribe("/scan", 1000, &Bot_Controller::m_scan_callback, this);
    m_fiducial_subscriber = m_nh.subscribe("/fiducial_transforms", 1000, &Follower::m_fiducial_callback, this);
    //add more subscribers as needed
}

void Follower::m_fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
    if (!msg->transforms.empty()) {//check marker is detected
        //broadcaster object
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        int count=0;
        int fiducial_id;

        //broadcast the new frame to /tf Topic
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
        transformStamped.child_frame_id = "marker_frame";
        transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
        transformStamped.transform.translation.y = msg->transforms[1].transform.translation.y;
        transformStamped.transform.translation.z = msg->transforms[2].transform.translation.z;
        
        transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
        transformStamped.transform.rotation.y = msg->transforms[1].transform.rotation.y;
        transformStamped.transform.rotation.z = msg->transforms[2].transform.rotation.z;
        transformStamped.transform.rotation.w = msg->transforms[3].transform.rotation.w;
        
        // fiducial_id.at(count)= m_nh.setParam("fiducial_id",->msg->transforms.fiducial_id);.
        // m_fid.at(count)=fiducial_id.at(count);
        // set_fid(->msg->transforms.fiducial_id,count);
        // set_fid(fiducial_id, count);
        count++;

        br.sendTransform(transformStamped);
    }
}