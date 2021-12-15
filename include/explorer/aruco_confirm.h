#ifndef ARUCO_H
#define ARUCO_H

#include <geometry_msgs/Twist.h>   //for geometry_msgs::Twist
#include <nav_msgs/Odometry.h>     //for nav_msgs::Odometry
#include <sensor_msgs/LaserScan.h> //for laser scans
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <utility>
#include <string>
#include <array>
#include <algorithm>
#include <tf/transform_datatypes.h> //to manipulate quaternions
#include <geometry_msgs/Quaternion.h>
#include <cmath>

#include "../include/follower/follower.h"

/**
 * @brief A class that detects the Aruco Marker.
 * 
 */
class ArucoNode{
    public:
        ArucoNode(ros::NodeHandle* nodehandle);
        void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);
        void marker_listen(tf2_ros::Buffer& tfBuffer, int count);
        void marker_broadcast();
        void marker_subs();
        void aruco_exists_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);

        // tf2_ros::Buffer tfBuffer;
        void aruco_seen();
        
        bool first_goal = false;

        std::array<int,4> fid_ids{0,0,0,0};
        std::array<std::array<double,2>,4> transformed_locs;

        int temp_id;
        int m_count{0};

        std::array<bool,4> marker_seen {false,false,false,false};
        
        ~ArucoNode() {}
        void m_initialize_subscribers();

    private:
        // void m_initialize_publishers();
        // void m_check_subscribers();


        ros::NodeHandle m_nh;
        // ros::Subscriber m_check_subscriber;
        ros::Subscriber m_fiducial_a_subscriber;
        ros::Subscriber m_fiducial_t_subscriber;

        

        
    
};

#endif