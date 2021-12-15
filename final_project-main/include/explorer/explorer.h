#ifndef EXPLORER_H
#define EXPLORER_H

#include <geometry_msgs/Twist.h>   //for geometry_msgs::Twist
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>     //for nav_msgs::Odometry
#include <sensor_msgs/LaserScan.h> //for laser scans
#include <fiducial_msgs/FiducialTransformArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <utility>
#include <string>
#include <array>
#include <algorithm>
#include <iostream>
#include <tf/transform_datatypes.h> //to manipulate quaternions
#include <cmath>

/**
 * @brief A class to control the explorer and exploration algorithm.
 * 
 */
class Explorer{
    public:
        /**
         * @brief Construct a new Explorer object
         * 
         * @param nodehandle 
         * @param robot_name 
         */
        Explorer(ros::NodeHandle *nodehandle, const std::string &robot_name);
        
        /**
         * @brief Publish velocities to odom
         * 
         * @param msg 
         */
        void publish_velocities(const geometry_msgs::Twist &msg);
        
        /**
         * @brief moves robot straight
         * 
         * @param distance 
         * @param direction 
         */
        void drive_straight(double distance, bool direction);
        
        /**
         * @brief Rotates robot
         * 
         * @param angle_to_rotate 
         * @param direction 
         * @param final_angle 
         */
        void rotate(double angle_to_rotate, bool direction, double final_angle);
        
        /**
         * @brief Moves robot to goal
         * 
         * @param x 
         * @param y 
         * @return true 
         * @return false 
         */
        bool go_to_goal(double x, double y);
        
        /**
         * @brief Stops robot
         * 
         */
        void stop();
        
        /**
         * @brief Compoutes final yaw angle
         * 
         * @param direction 
         * @param angle_to_rotate 
         * @return double 
         */
        double compute_expected_final_yaw(bool direction, double angle_to_rotate);
        double compute_yaw_deg();
        double compute_yaw_rad() ;
        double convert_rad_to_deg(double angle);

        /**
         * @brief Destroy the Explorer object
         * 
         */
        ~Explorer() {}

        /**
         * @brief Get Goal List
         * 
         */
        std::array<std::array<int,3>,4> goal_list {};

        /**
         * @brief Store starting and final position
         * 
         */
        std::array<int,3> start_place {};

        /**
         * @brief moves robot to next location
         * 
         * @param goal_loc 
         */
        void move_next_loc(std::array<double,2> goal_loc){};

       /**
         * @brief Get the goals object from aruco_lookup.yaml
         * 
         * @return std::array<std::array<double,2>,4> 
         */
        std::array<std::array<double,2>,4> get_goals() {};

        ros::Publisher m_velocity_publisher;

        /**
         * @brief Move bot in a linear and angular motion
         * 
         * @param linear 
         * @param angular 
         */
        void m_move(double linear, double angular);


    private:
        ros::NodeHandle m_nh;

        ros::Subscriber m_e_pose_subscriber;
        ros::Subscriber m_e_scan_subscriber;

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
        double m_yaw;       //rad

        /**
         * @brief Odom callback
         * 
         * @param msg 
         */
        void m_pose_callback(const nav_msgs::Odometry::ConstPtr &msg);     
        
        /**
         * @brief Laser Scan callback
         * 
         * @param msg 
         */
        void m_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
        
        /**
         * @brief Initializes subscribers
         * 
         */
        void m_initialize_subscribers();
       
        /**
         * @brief initializes publishers
         * 
         */
        void m_initialize_publishers();

        /**
         * @brief computes distance between two points
         * 
         * @param a 
         * @param b 
         * @return double 
         */
        double m_compute_distance(const std::pair<double, double> &a, const std::pair<double, double> &b);
       
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