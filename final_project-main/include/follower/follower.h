#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <geometry_msgs/Twist.h>   
#include <nav_msgs/Odometry.h>     //for nav_msgs::Odometry
#include <sensor_msgs/LaserScan.h> //for laser scans
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <utility>
#include <string>
#include <array>
#include <algorithm>
#include <iostream>
#include <tf/transform_datatypes.h> //to manipulate quaternions
#include <cmath>


/**
 * @brief Follower Class for Follower bot
 * Goal is to have Follower go to ArUco markers in fiducial ID order
 *
 * 
 */
class Follower {
    public:
        /**
         * @brief Construct a new Follower object
         * 
         * @param nodehandle 
         * @param robot_name 
         */
        Follower(ros::NodeHandle *nodehandle, const std::string &robot_name);
        
        /**
         * @brief Get robot to goal based on x and y coordinates
         *  Error Tolerance of 0.05
         * 
         * @param x 
         * @param y 
         * @return true 
         * @return false 
         */
        bool go_to_goal(double x, double y);
        void stop();

        /**
         * @brief Computes expected final yaw
         * 
         * @param direction 
         * @param angle_to_rotate 
         * @return double 
         */
        double compute_expected_final_yaw(bool direction, double angle_to_rotate);
        double compute_yaw_deg();
        double compute_yaw_rad();
        double convert_rad_to_deg(double angle);

        /**
         * @brief Destroy the Follower object
         * 
         */
        ~Follower() {}

        /**
         * @brief store fidicual IDs
         * 
         */
        std::array <int, 4> m_fid {};

        /**
         * @brief store marker positions
         * 
         */
        std::array<std::array<double, 2>, 4> m_posit{};


    private:
        ros::NodeHandle m_nh;

        ros::Subscriber m_pose_subscriber;
        ros::Subscriber m_scan_subscriber;
        ros::Subscriber m_fiducial_subscriber;
        ros::Publisher m_velocity_publisher;

        std::string m_robot_name;

        double m_kv; //gain for linear velocity
        double m_kh; //gain for angular velocity
        std::string m_parent_frame;
        std::string m_child_frame;
        std::pair<double, double> m_location;
        geometry_msgs::Quaternion m_orientation;
        double m_linear_speed;
        double m_angular_speed;
        double m_roll;                                                     //rad
        double m_pitch;                                                    //rad
        double m_yaw;   //rad
        
        /**
         * @brief odom callback
         * 
         * @param msg 
         */
        void m_pose_callback(const nav_msgs::Odometry::ConstPtr &msg);
        
        /**
         * @brief Laser Scan
         * 
         * @param msg 
         */
        void m_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg); 
        
        /**
         * @brief To store the locations of the fiducial IDs (waypoints) and the fiducial_ID number
         * Based on explorer's camera
         * 
         * @param msg 
        */
        void m_fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg);
        
        /**
         * @brief Initializes subscribers
         * 
         */
        void m_initialize_subscribers();

        /**
         * @brief Initializes Publishers
         * 
         */
        void m_initialize_publishers();
        double m_compute_distance(const std::pair<double, double> &a, const std::pair<double, double> &b);
        
        /**
         * @brief Move bot based on given angle and linear distance
         * 
         * @param linear 
         * @param angular 
         */
        void m_move(double linear, double angular);

        /**
         * @brief Normalizes the angle to be 0 to 2*M_PI
         * 
         * @param angle Angle to normalize (rad)
         * @return double Normalized angle (rad)
         */
        double m_normalize_angle_positive(double angle);

        /**
         * @brief Normalizes the angle to be -M_PI circle to +M_PI circle
         * 
         * @param angle Angle to normalize (rad)
         * @return double Normalized angle (rad)
         */
        double m_normalize_angle(double angle);

};

#endif