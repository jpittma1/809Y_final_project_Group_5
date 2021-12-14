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

/**
 * @brief A class that detects the Aruco Marker.
 * 
 */
class ArucoNode{
    public:
        ArucoNode(ros::NodeHandle* nodehandle);
        void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){};
        void marker_listen(tf2_ros::Buffer& tfBuffer, int count){};
        void marker_broadcast(){};
        
        void detect_subs(){};

        

        std::array<int,4> fid_ids;
        std::array<std::array<double,2>,4> transformed_locs;

        int temp_id;
        int m_count{0};
        
        ~ArucoNode() {}
    private:
        void m_initialize_publishers(){};
        void m_initialize_subscribers(){};

        ros::NodeHandle m_nh;
        ros::Subscriber m_pose_subscriber;
        ros::Subscriber m_scan_subscriber;
        ros::Subscriber m_fiducial_subscriber;
        ros::Publisher m_velocity_publisher;

        

        
    
};

#endif