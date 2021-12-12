#ifndef FOLLOWER_H
#define FOLLOWER_H

#include "../include/bot_controller/bot_controller.h"
#include "bot_msgs/BotStatus.h"
#include <geometry_msgs/Twist.h>   //for geometry_msgs::Twist
#include <nav_msgs/Odometry.h>     //for nav_msgs::Odometry
#include <sensor_msgs/LaserScan.h> //for laser scans
#include <ros/ros.h>
#include <utility>
#include <array>
#include <tf/transform_datatypes.h> //to manipulate quaternions
#include <geometry_msgs/Twist.h>   //for geometry_msgs::Twist


/**
 * @brief Public Inheritance Child class of Bot_controller Class
 * to go to ArUco markers found by
 * Explorer child
 * 
 */
class Follower : public Bot_Controller {
    public:
        // main() will need to instantiate a ROS nodehandle, then pass it to the constructor
        Follower(ros::NodeHandle *nodehandle, const std::string &robot_name);
        virtual void publish_velocities(const geometry_msgs::Twist &msg) override;
        virtual void drive_straight(double distance, bool direction) override;
        virtual void rotate(double angle_to_rotate, bool direction, double final_angle) override;
        virtual bool go_to_goal(double x, double y) override;
        virtual void stop() override;
        virtual double compute_expected_final_yaw(bool direction, double angle_to_rotate) override;
        virtual double compute_yaw_deg() override;
        virtual double compute_yaw_rad() override;
        virtual double convert_rad_to_deg(double angle) override;

        //--Bot Controller Accessors--
        const double get_current_x(){
            return m_location.first;
        }

        const double get_current_y(){
            return m_location.second;
        }
        
        //accessor
        const std::array& get_fid() const {
            return m_fid;
        }
       
        //mutator
        void set_fid(int id, int location) {
            m_fid[location] = id;
        }

    private:
        std::Array <int, 4> m_fid{};

        /**
         * @brief To store the locations of the fiducial IDs (waypoints) and the fiducial_ID number
         * 
         * @param msg 
         */
        void m_fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);

};

#endif