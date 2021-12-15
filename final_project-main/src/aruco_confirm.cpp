/**
 * @file explorer.cpp
 * @author 809Y Final Project Group 5
 * @brief  ArucoNode
 * @version 1.0
 * @date 2021-12-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "../include/explorer/aruco_confirm.h"

ArucoNode::ArucoNode(ros::NodeHandle* nodehandle, tf2_ros::Buffer& tfBuffer, int count):
    m_nh{ *nodehandle }
{
    m_initialize_subscribers(tfBuffer, count);
};

void ArucoNode::fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
{
    if (!msg->transforms.empty()) {//check marker is detected
    //broadcaster object
    //Looking for marker_frame before this is called.
        ROS_INFO_STREAM("Setting up broadcaster!\nMessage:" << msg);
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        
        //broadcast the new frame to /tf Topic
        // ROS_INFO_STREAM("I see another sillouetto of a marker: " << msg->transforms[0].fiducial_id);
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
        transformStamped.child_frame_id = "marker_frame"; //name of the frame
        transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
        transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
        transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z;
        
        transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
        transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
        transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
        transformStamped.transform.rotation.w = msg->transforms[0].transform.rotation.w;
        
        fid_ids[m_count] = msg->transforms[0].fiducial_id;
        // ROS_INFO_STREAM("Now I see x at: " << transformStamped.transform.translation.x);
        

        br.sendTransform(transformStamped); //broadcast the transform on /tf Topic
        marker_seen[m_count] = true;
    }
    // else{ROS_INFO_STREAM("Empty Transform.");}
}

void ArucoNode::marker_listen(tf2_ros::Buffer& tfBuffer, int count) {
  //for listener
    
    m_initialize_subscribers(tfBuffer, count);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("map", "marker_frame",
            ros::Time(0));
        ROS_INFO_STREAM("marker in /map frame: ["
        << transformStamped.transform.translation.x << ","
        << transformStamped.transform.translation.y << ","
        << transformStamped.transform.translation.z << "]"
        );
        transformed_locs[count][0] = transformStamped.transform.translation.x;
        transformed_locs[count][1] = transformStamped.transform.translation.y;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        
    }
    ros::spinOnce();

}

void ArucoNode::m_initialize_subscribers(tf2_ros::Buffer& tfBuffer, int count) {
    ROS_INFO_STREAM("Initializing Subscribers\nMessage: "<<this);
    m_fiducial_a_subscriber = m_nh.subscribe("/fiducial_transforms", 1000, &ArucoNode::fiducial_callback, this); 
}