#ifndef EXPLORER_H
#define EXPLORER_H

#include "../include/bot_controller/bot_controller.h"

/**
 * @brief A class that inherits from Bot_contreoller to control the explorer and exploration algorithm.
 * 
 */
class Explorer{
    public:
        // main() will need to instantiate a ROS nodehandle, then pass it to the constructor
        Explorer(ros::NodeHandle *nodehandle, const std::string &robot_name);
        void publish_velocities(const geometry_msgs::Twist &msg);
        void drive_straight(double distance, bool direction);
        void rotate(double angle_to_rotate, bool direction, double final_angle);
        bool go_to_goal(double x, double y);
        void stop();
        double compute_expected_final_yaw(bool direction, double angle_to_rotate);
        double compute_yaw_deg();
        double compute_yaw_rad() ;
        double convert_rad_to_deg(double angle);

        ~Explorer() {}

        // ros::NodeHandle exp_node;
        std::array<std::array<int,3>,4> goal_list {};
        std::array<int,3> start_place {};


        void move_next_loc(std::array<double,2> goal_loc){};

        std::array<std::array<double,2>,4> get_goals(){}

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
        double m_yaw;       //rad

        void m_pose_callback(const nav_msgs::Odometry::ConstPtr &msg);     // prototype for callback of example subscriber
        void m_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg); // prototype for callback of example subscriber
        
        void m_initialize_subscribers();
        void m_initialize_publishers();
        double m_compute_distance(const std::pair<double, double> &a, const std::pair<double, double> &b);
        void m_move(double linear, double angular);

        double m_normalize_angle_positive(double angle);
        double m_normalize_angle(double angle);

    
};

#endif
