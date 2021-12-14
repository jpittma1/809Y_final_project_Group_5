/**
 * @file follower.cpp
 * @author 809Y Final Project Group 5
 * @brief Follower Robot
 * @version 0.1
 * @date 2021-12-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "../include/follower/follower.h"

Follower::Follower(ros::NodeHandle* nodehandle, const std::string& robot_name) :
    m_robot_name{robot_name},
    m_nh{ *nodehandle },
    m_kv{ 0.2 },
    m_kh{ 0.26 },
    m_parent_frame{ "odom" },
    m_child_frame{ "base_footprint" },
    m_location{ 0,0 },
    m_linear_speed{ 0.7 },
    m_angular_speed{ 0.6 },
    m_roll{ 0 },
    m_pitch{ 0 },
    m_yaw{ 0 }
{
    ROS_INFO("FOllower Object Created!");
    m_initialize_subscribers();
    m_initialize_publishers();
}


double Follower::m_normalize_angle_positive(double angle)
{
    const double result = fmod(angle, 2.0 * M_PI);
    if (result < 0) return result + 2.0 * M_PI;
    return result;
}


double Follower::m_normalize_angle(double angle)
{
    const double result = fmod(angle + M_PI, 2.0 * M_PI);
    if (result <= 0.0) return result + M_PI;
    return result - M_PI;
}

void Follower::m_initialize_publishers() {
    ROS_INFO("Initializing Follower Publishers");
    m_velocity_publisher = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
}

void Follower::m_initialize_subscribers() {
    ROS_INFO("Initializing Follower Subscribers");
    m_pose_subscriber = m_nh.subscribe("/odom", 1000, &Follower::m_pose_callback, this);
    m_scan_subscriber = m_nh.subscribe("/scan", 1000, &Follower::m_scan_callback, this);
    m_fiducial_subscriber = m_nh.subscribe("/fiducial_transforms", 1000, &Follower::m_fiducial_callback, this);
   
}

double Follower::convert_rad_to_deg(double angle) {
    return (angle * M_PI / 180.0);
}

void Follower::m_pose_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    m_location.first = odom_msg->pose.pose.position.x;
    m_location.second = odom_msg->pose.pose.position.y;
    m_orientation = odom_msg->pose.pose.orientation;

}

void Follower::m_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO_STREAM("-------------------------");
    ROS_INFO_STREAM("Front: " << msg->ranges[0]);
    ROS_INFO_STREAM("Left: " << msg->ranges[90]);
    ROS_INFO_STREAM("Right: " << msg->ranges[270]);
}

void Follower::m_fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
    if (!msg->transforms.empty()) {//check marker is detected
        //broadcaster object
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

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

        fiducial_id= msg->transforms[0].fiducial_id;

        //Store location of fiducial IDs based on fiducial_ID detected
        if (fiducial_id==0){ 
            m_fid.at(0)=fiducial_id;
            m_posit.at(0)={transformStamped.transform.translation.x, transformStamped.transform.translation.y};
        } else if(fiducial_id==1) {
             m_fid.at(1)=fiducial_id;
             m_posit.at(1)={transformStamped.transform.translation.x, transformStamped.transform.translation.y};
        } else if (fiducial_id==2) {
            m_fid.at(2)=fiducial_id;
            m_posit.at(2)={transformStamped.transform.translation.x, transformStamped.transform.translation.y};
        } else {
             m_fid.at(3)=fiducial_id;
             m_posit.at(3)={transformStamped.transform.translation.x, transformStamped.transform.translation.y};
        }
        
        ROS_INFO("New Fiducial ID detected and location stored");

        //Debug print
        std::cout <<"\nNew marker " << fiducial_id << " added is\n";
        std::cout <<"New Posit (" <<  m_posit.at(fiducial_id).at(0);
        std::cout << ", " << m_posit.at(fiducial_id).at(1)<<")\n";

        br.sendTransform(transformStamped);
    }
}

double Follower::m_compute_distance(const std::pair<double, double>& a, const std::pair<double, double>& b) {
    return  sqrt(pow(b.first - a.first, 2) + pow(b.second - a.second, 2));
}

void Follower::m_move(double linear, double angular) {
    geometry_msgs::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    m_velocity_publisher.publish(msg);
}

void Follower::stop() {
    ROS_DEBUG_STREAM("Stopping robot");
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    m_velocity_publisher.publish(msg);
    ros::shutdown();
}

double Follower::compute_expected_final_yaw(bool direction, double angle_to_rotate) {
    double current_yaw_deg = compute_yaw_deg();

    double final_angle{};//final angle after rotating angle_to_rotate
    if (direction)
        final_angle = current_yaw_deg + angle_to_rotate;
    else
        final_angle = current_yaw_deg - angle_to_rotate;

    return final_angle;
}

double Follower::compute_yaw_deg() {
    double roll{};
    double pitch{};
    double yaw_rad{};
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(m_orientation, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw_rad);
    yaw_rad = m_normalize_angle_positive(yaw_rad);
    double current_yaw_deg = yaw_rad * 180.0 / M_PI;
    return current_yaw_deg;
}

double Follower::compute_yaw_rad() {
    double roll{};
    double pitch{};
    double yaw_rad{};
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(m_orientation, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw_rad);
    return yaw_rad;
}



void Follower::rotate(double angle_to_rotate, bool direction, double final_angle) {
    double current_yaw_deg = compute_yaw_deg();
    ROS_INFO_STREAM("Rotate an angle of [" << angle_to_rotate << " degrees]");
    ROS_INFO_STREAM("Current orientation: " << current_yaw_deg);
    ROS_INFO_STREAM("Final orientation: " << final_angle);

    if (direction) {
        if (final_angle >= current_yaw_deg) {
            m_move(0.0, m_angular_speed);
        }
        else {
            //stop the robot
            stop();
        }
    }
    else {
        if (final_angle <= current_yaw_deg) {
            m_move(0.0, -m_angular_speed);
        }
        else {
            //stop the robot
            stop();
        }
    }

}

void Follower::drive_straight(double distance_to_drive, bool direction) {
    ROS_INFO_STREAM("Drive a distance of [" << distance_to_drive << " m]");
    ROS_INFO_STREAM("Current: " << m_location.first << "," << m_location.second);
    static std::pair<double, double> start_position{ m_location };
    double distance_driven = m_compute_distance(m_location, start_position);

    if (distance_driven <= distance_to_drive) {
        // distance_driven = m_compute_distance(m_location, start_position);
        ROS_INFO_STREAM("Distance driven: [" << distance_driven << "]");
        if (direction)
            m_move(m_linear_speed, 0.0);
        else
            m_move(-m_linear_speed, 0.0);
    }
    else {
        //stop the robot
        stop();
    }
}


bool Follower::go_to_goal(double goal_x, double goal_y) {
    // ROS_INFO_STREAM("Follower Going to goal [" << goal_x << "," << goal_y << "]");
    // ROS_INFO_STREAM("Current location: " << m_location.first << "," << m_location.second);
    std::pair<double, double> goal{ goal_x, goal_y };
    double distance_to_goal = m_compute_distance(m_location, goal);
    double linear_x{};
    double angular_z{};

    if (distance_to_goal > 0.05) {
        distance_to_goal = m_compute_distance(m_location, goal);
        double angle_to_goal = std::atan2(goal_y - m_location.second, goal_x - m_location.first);
        // ROS_INFO_STREAM("Distance to goal: " << distance_to_goal << "m");
        // ROS_INFO_STREAM("Angle to goal: " << angle_to_goal << " rad");

        if (angle_to_goal < 0)
            // angle_to_goal = 2 * M_PI + angle_to_goal;
            angle_to_goal = m_normalize_angle_positive(angle_to_goal);

        //angle to rotate to face the goal
        double w = angle_to_goal - compute_yaw_rad();

        if (w > M_PI) {
            w = w - 2 * M_PI;
            // w = m_normalize_angle_positive(w);
        }

        //proportional control for linear velocity
        linear_x = std::min(m_kv * distance_to_goal, m_linear_speed);


        //proportional control for angular velocity
        angular_z = m_kh * w;
        if (angular_z > 0)
            angular_z = std::min(angular_z, m_angular_speed);
        else
            angular_z = std::max(angular_z, -m_angular_speed);

        m_move(linear_x, angular_z);

    }
    else {
        stop();
        // ros::shutdown();
        return true;
    }
}