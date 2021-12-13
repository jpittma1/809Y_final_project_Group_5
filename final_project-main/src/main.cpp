

#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <array>
#include <iostream>
#include <algorithm>
#include <utility>
#include <string>

#include "../include/follower/follower.h"
#include "../include/explorer/explorer.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// forward declaration
void print_usage(std::string error = "");

void print_usage(std::string error)
{
  if (!error.empty())  // if not empty string
    ROS_ERROR_STREAM(error << "\n");
  ros::shutdown();
}

void broadcast() {
  //for broadcaster
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  //broadcast the new frame to /tf Topic
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
  transformStamped.child_frame_id = "my_frame";

  transformStamped.transform.translation.x = 0.5;
  transformStamped.transform.translation.y = 0.5;
  transformStamped.transform.translation.z = 0.2;
  transformStamped.transform.rotation.x = 0;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;
  ROS_INFO("Broadcasting");
  br.sendTransform(transformStamped);
}

void listen(tf2_ros::Buffer& tfBuffer) {
  //for listener

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "my_frame", ros::Time(0));
    auto trans_x = transformStamped.transform.translation.x;
    auto trans_y = transformStamped.transform.translation.y;
    auto trans_z = transformStamped.transform.translation.z;

    ROS_INFO_STREAM("Position in map frame: ["
      << trans_x << ","
      << trans_y << ","
      << trans_z << "]"
    );
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char** argv)
{
  
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;

  ros::init(argc, argv, "follower");
  ros::init(argc, argv, "explorer");

  // ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;
  geometry_msgs::TransformStamped transformStamped;

  std::string robot_name;
  // std::string robot_name_follow;

  //Initialize Follower and Explorer class objects
  Follower follower(&nh, "follower");
  
 
  // if (nh.hasParam("robot_name")) {
  //   nh.getParam("robot_name", robot_name_follow);
  //   ROS_INFO_STREAM("robot name: " << robot_name_follow);
  // }
  // else {
  //   print_usage("missing argument: _robot_name_follow:= <name>");
  // }

  Explorer explorer(&nh, "explorer");
  
  // if (nh.hasParam("robot_name")) {
  //   nh.getParam("robot_name", robot_name_explore);
  //   ROS_INFO_STREAM("robot name: " << robot_name_explore);
  // }
  // else {
  //   print_usage("missing argument: _robot_name_explore:= <name>");
  // }
  
  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);


  move_base_msgs::MoveBaseGoal explorer_goal;
  move_base_msgs::MoveBaseGoal follower_goal;

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }
  
  // move_base_msgs::MoveBaseGoal explorer_goal;
  // move_base_msgs::MoveBaseGoal follower_goal;


  //Build goal for explorer
  // explorer_goal.target_pose.header.frame_id = "map";
  // explorer_goal.target_pose.header.stamp = ros::Time::now();
  // explorer_goal.target_pose.pose.position.x = 7.710214;
  // explorer_goal.target_pose.pose.position.y = -1.716889;
  // explorer_goal.target_pose.pose.orientation.w = 1.0;

  // explorer_client.waitForResult();

  // ROS_INFO("Sending goal");
  // explorer_client.waitForResult();

  std::string motion_type;
  if (nh.hasParam("motion"))
  {
    nh.getParam("motion", motion_type);
  }
  else
  {
    print_usage("missing argument: _motion:= <s/r/g/h>");
  }

  double drive_value;

  std::string direction_s;
  bool direction_b;
  if (motion_type == "s" || motion_type == "r")
  {
    if (nh.hasParam("value"))
    {
      nh.getParam("value", drive_value);
    }
    else
    {
      print_usage("_value:= <double>");
    }

    if (nh.hasParam("direction"))
    {
      nh.getParam("direction", direction_s);
      if (direction_s == "f")
        direction_b = true;
      else if (direction_s == "b")
        direction_b = false;
      else
      {
        print_usage("_direction:=<f/b>");
      }
    }
    else
    {
      print_usage("_direction:= <f/b>");
    }
  }

  //---Data to pull from array "m_fid" and "m_posit"
  int fiducial_id;
  double goal_x;
  double goal_y;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);

  static double final_angle{ 0 };
  
  std::array<std::array<double,2>,4> goal_list = explorer.get_goals();
 
  while (ros::ok()) {
    //*****EXPLORER*****//

    int i = 0;
    for(i = 0; i < 4; i++){
      goal_x = goal_list[i][0];
      goal_y = goal_list[i][1];

      ROS_INFO_STREAM("Goal Explorer Pos: "
          << goal_x
          << goal_y
          << '.');

      explorer_goal.target_pose.header.frame_id = "map";
      explorer_goal.target_pose.header.stamp = ros::Time::now();
      explorer_goal.target_pose.pose.position.x = goal_x;
      explorer_goal.target_pose.pose.position.y = goal_y;
      explorer_goal.target_pose.pose.orientation.w = 1.0;


      if (!explorer_goal_sent){
        ROS_INFO("Sending goal for explorer");
        explorer_client.sendGoal(explorer_goal);//this should be sent only once
        explorer_goal_sent = true;
      }
      if (motion_type == "h")
        explorer.stop();
      else if (motion_type == "s")
        explorer.drive_straight(drive_value, direction_b);

      // explorer.go_to_goal(explorer.goal_list[i][0],explorer.goal_list[i][1]);
      
      ros::Duration(0.5).sleep();

    
      if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, explorer reached goal");
      }
    

    try {
        transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));
        ROS_INFO_STREAM("Position in /map frame: ["
          << transformStamped.transform.translation.x << ","
          << transformStamped.transform.translation.y << ","
          << transformStamped.transform.translation.z << "]"
        );

      
      }
      catch (tf2::TransformException& ex) {
          ROS_WARN("%s", ex.what());
          ros::Duration(1.0).sleep();
      }}
    
      //*****FOLLOWER*******//
      
     
      if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      //---Send Follower to IDs O through 3 ---//
        for (int i=0;i<4;i++) {
          //--Set Goal for next ID--//
          fiducial_id=follower.m_fid.at(i);
          goal_x=follower.m_posit.at(i).at(0);
          goal_y=follower.m_posit.at(i).at(1);
          //Build goal for follower using Move_base
          follower_goal.target_pose.header.frame_id = "map";
          follower_goal.target_pose.header.stamp = ros::Time::now();
          follower_goal.target_pose.pose.position.x = goal_x;//
          follower_goal.target_pose.pose.position.y = goal_y;//
          follower_goal.target_pose.pose.orientation.w = 1.0;

          ROS_INFO("Sending goal");
          follower_client.sendGoal(follower_goal);
          follower_client.waitForResult();
          
          if (motion_type == "h")
            follower.stop();
          else if (motion_type == "s")
            follower.drive_straight(drive_value, direction_b);
          else if (motion_type == "r")
          {
            if (final_angle == 0)
              final_angle = follower.compute_expected_final_yaw(direction_b, drive_value);
            follower.rotate(drive_value, direction_b, final_angle);
          }
          else if (motion_type == "g")
          {
            ROS_INFO("Sending follower to goal for");
            std::cout << "\nFiducial ID: "<< fiducial_id;
            std::cout << "\nLocated at: ("<< goal_x << ", "<<goal_y<<")";
            follower.go_to_goal(goal_x, goal_y);
            ROS_INFO("Hooray, follower reached goal");
          }

          //---Send Follower to Start Position (-4,3.5) ---//
          if (i==3) {
            ROS_INFO("Sending follower to Start Position");
            goal_x=-4;
            goal_y=3.5;
            follower.go_to_goal(goal_x, goal_y);
            ROS_INFO("Hooray, follower reached starting position");
            ros::shutdown();
          }
       }
      }

    broadcast();
    listen(tfBuffer);
    ros::spinOnce();
    loop_rate.sleep();

  }


}