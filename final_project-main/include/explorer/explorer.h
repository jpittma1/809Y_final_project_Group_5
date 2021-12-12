#ifndef EXPLORER_H
#define EXPLORER_H

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
        std::array<std::array<int,3>,4> goal_list {};
        std::array<int,3> start_place {};


        void move_next_loc(std::array<double,2> goal_loc){};

        std::array<std::array<double,2>,4> get_goals(){}

    
};

#endif
