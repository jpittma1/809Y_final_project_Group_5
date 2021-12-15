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
        /**
         * @brief Construct a new Aruco Node object
         * 
         * @param nodehandle 
         */
        ArucoNode(ros::NodeHandle* nodehandle,tf2_ros::Buffer& tfBuffer, int count);

        /**
         * @brief Broadcaster for arUco Markers
         * 
         * @param msg 
         */
        void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);
        
        /**
         * @brief listener for arUco markers
         * 
         * @param tfBuffer 
         * @param count 
         */
        void marker_listen(tf2_ros::Buffer& tfBuffer, int count);
    
        /**
         * @brief First goal met; initially no
         * 
         */
        bool first_goal = false;

        /**
         * @brief Fiducial IDs storage
         * 
         */
        std::array<int,4> fid_ids{0,0,0,0};

        /**
         * @brief Transformed locations
         * 
         */
        std::array<std::array<double,2>,4> transformed_locs;

        int temp_id;
        int m_count{0};

        /**
         * @brief Determines if marker seen; 4 IDs to detect
         * 
         */
        std::array<bool,4> marker_seen {false,false,false,false};
        
        /**
         * @brief Destroy the Aruco Node object
         * 
         */
        ~ArucoNode() {}

        void m_initialize_subscribers(tf2_ros::Buffer& tfBuffer, int count);
        
    private:
        


        ros::NodeHandle m_nh;
       
        /**
         * @brief fiducial subscriber
         * 
         */
        ros::Subscriber m_fiducial_a_subscriber;
        ros::Subscriber m_fiducial_t_subscriber;

    
};

#endif