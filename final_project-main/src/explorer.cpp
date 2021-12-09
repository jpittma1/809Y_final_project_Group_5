

#include "../include/bot_controller/bot_controller.h"
#include "../include/explorer/explorer.h"

#include "bot_msgs/BotStatus.h"
#include <geometry_msgs/Twist.h>   //for geometry_msgs::Twist
#include <nav_msgs/Odometry.h>     //for nav_msgs::Odometry
#include <sensor_msgs/LaserScan.h> //for laser scans
#include <ros/ros.h>
#include <utility>
#include <tf/transform_datatypes.h> //to manipulate quaternions
#include <geometry_msgs/Twist.h>   //for geometry_msgs::Twist
#include <iostream>
#include "../param/aruco_lookup.yaml"

Explorer::Explorer(ros::NodeHandle* nodehandle const std::string& robot_name) : Bot_Controller(nodehandle, robot_name)
{
    start_place = Explorer::get_start_loc();

    void get_goals(){
        exp_node.parse(aruco_lookup.yaml,"yaml");
        std::cout << exp_node["target 1"].as_float64() << std::endl;
        goal_list[0][0] = exp_node["target 1"][0].as_float64();
        goal_list[0][1] = exp_node["target 1"][1].as_float64();
        goal_list[1][0] = exp_node["target 2"][0].as_float64();
        goal_list[0][1] = exp_node["target 2"][1].as_float64();
        goal_list[1][0] = exp_node["target 3"][0].as_float64();
        goal_list[0][1] = exp_node["target 3"][1].as_float64();
        goal_list[1][0] = exp_node["target 4"][0].as_float64();
        goal_list[0][1] = exp_node["target "][1].as_float64();


    }
    


}

int main(){
    exp_node.parse(aruco_lookup.yaml,"yaml");
    std::cout << exp_node["target 1"].as_float64() << std::endl;
    
}