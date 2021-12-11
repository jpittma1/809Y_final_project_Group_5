#include "../include/follower/follower.h"

Follower::Follower(ros::NodeHandle* nodehandle, const std::string& robot_name) :
    Bot_Controller(nodehandle, robot_name)
{
    ROS_INFO("Follower Constructor called");
    m_initialize_subscribers();
    m_initialize_publishers();
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
        transformStamped.transform.translation.x =→ msg->transforms[0].transform.translation.x;
        transformStamped.transform.translation.y =→ msg->transforms[1].transform.translation.y;
        transformStamped.transform.translation.z =→ msg->transforms[2].transform.translation.z;
        
        transformStamped.transform.rotation.x =→ msg->transforms[0].transform.rotation.x;
        transformStamped.transform.rotation.y =→ msg->transforms[1].transform.rotation.y;
        transformStamped.transform.rotation.z =→ msg->transforms[2].transform.rotation.z;
        transformStamped.transform.rotation.w =→ msg->transforms[3].transform.rotation.w;
        
        fiducial_id=->msg->transforms[0].fiducial_id;
        set_fid(fiducial_id, count);
        count++;

        br.sendTransform(transformStamped);
    }
}