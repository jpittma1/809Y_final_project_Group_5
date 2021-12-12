#ifndef EXPLORER_H
#define EXPLORER_H

// #include <bot_msgs/BotStatus.h>
#include <geometry_msgs/Twist.h>   //for geometry_msgs::Twist
#include <nav_msgs/Odometry.h>     //for nav_msgs::Odometry
#include <sensor_msgs/LaserScan.h> //for laser scans
#include <ros/ros.h>
#include <utility>
#include <tf/transform_datatypes.h> //to manipulate quaternions
#include <geometry_msgs/Twist.h>   //for geometry_msgs::Twist
#include "../include/bot_controller/bot_controller.h"



/**
 * @brief A class that inherits from Bot_contreoller to control the explorer and exploration algorithm.
 * 
 */
class Explorer : public Bot_Controller {
    public:
        // main() will need to instantiate a ROS nodehandle, then pass it to the constructor
        Explorer(ros::NodeHandle *nodehandle, const std::string &robot_name);
        virtual void publish_velocities(const geometry_msgs::Twist &msg) override;
        virtual void drive_straight(double distance, bool direction) override;
        virtual void rotate(double angle_to_rotate, bool direction, double final_angle) override;
        virtual bool go_to_goal(double x, double y) override;
        virtual void stop() override;
        virtual double compute_expected_final_yaw(bool direction, double angle_to_rotate) override;
        virtual double compute_yaw_deg() override;
        virtual double compute_yaw_rad() override;
        virtual double convert_rad_to_deg(double angle) override;


        ros::NodeHandle exp_node;
        std::array<std::array<int,3>,4> goal_list;
        std::array<int,3> start_place;

        //--Accessors--
        // const double get_current_x(){
        //     return m_location.first;
        // }

        // const double get_current_y(){
        //     return m_location.second;
        // }
        std::array<double,2> get_start_loc(){};

        void move_next_loc(std::array<double,2> goal_loc){};

        std::array<std::array<double,2>,4> get_goals(){}

        // friend std::ostream& operator << (std::ostream& output, const Follower& follower) {
        // output << "--------------------------\n";
        // output << "Next Fiducial ID: " << follower.m_fiducial_id << '\n';
        // output << "ID location (x, y, z): (" << follower.m_x << ", ";
        // output << follower.m_y << ", " follower.m_z <<")\n";
        // output << "\n--------------------------";
        // return output;
        // }
};

#endif
