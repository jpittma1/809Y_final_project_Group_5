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
#include "../include/explorer/aruco_confirm.h"

ArcuoNode::ArucoNode(ros::NodeHandle* nodehandle):
    m_nh{ *nodehandle }
{
    m_initialize_publishers();
    m_initialize_subscribers();    
};

void ArucoNode::fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
{
    if (!msg->transforms.empty()) {//check marker is detected
    //broadcaster object
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        //broadcast the new frame to /tf Topic
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
        transformStamped.child_frame_id = "marker_frame"; //name of the frame
        transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
        transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
        transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z;
        //Change from transforms[0]?
        fid_ids[m_count] = msg->transforms[0].fiducial_id;
        m_count ++;


        br.sendTransform(transformStamped); //broadcast the transform on /tf Topic
    }
}

void ArucoNode::marker_listen(tf2_ros::Buffer& tfBuffer, int count) {
  //for listener

    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("map", "marker_frame",
            ros::Time(0));
        ROS_INFO_STREAM("marker in /map frame: ["
        << transformStamped.transform.translation.x << ","
        << transformStamped.transform.translation.y << ","
        << transformStamped.transform.translation.z << "]"
        );
        fid_ids[count] = transformStamped.transform.translation.z;
        transformed_locs[count][0] = transformStamped.transform.translation.x;
        transformed_locs[count][1] = transformStamped.transform.translation.y;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void ArucoNode::detect_subs(){}

void ArucoNode::m_initialize_publishers() {
    ROS_INFO("Initializing Publishers");
    m_velocity_publisher = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
}

void ArucoNode::m_initialize_subscribers() {
    ROS_INFO("Initializing Subscribers");
    m_scan_subscriber = m_nh.subscribe("explorer_tf/camera_rgb_optical_frame", 1000, &ArucoNode::fiducial_callback, this);   
}