#ifndef FOLLOWER_H
#define FOLLOWER_H

#include "../include/bot_controller/bot_controller.h"


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

        // virtual ~Follower() {}

        std::array <int, 4> m_fid {};               //store fidicual IDs
        std::array<std::array<double, 2>, 4> m_posit{};  //store marker positions


        //Follower accessor
        // const std::array <int, 4> get_fid() {
        //     return m_fid;
        // }
    
       
        // //Follower mutator
        // void set_fid(int id, int location) {
        //     m_fid.at(location) = id;
        // }

    // private:
        // std::array <int, 4> m_fid {};

};

#endif