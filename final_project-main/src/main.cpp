/**
 * @file main.cpp
 * @author Jerry Pittman, Nicholas Novak, Orlandis Smith
 * @brief 809Y Final Project Group 5
 * @version 1.0
 * @date 2021-12-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "../include/explorer/explorer.h"
#include "../include/explorer/aruco_confirm.h"
#include "../include/follower/follower.h"

#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpc.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void print_usage(std::string error = "");

void print_usage(std::string error)
{
  if (!error.empty())  // if not empty string
    ROS_ERROR_STREAM(error << "\n");
  ros::shutdown();
}

int main(int argc, char** argv) {
  
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;

  ros::init(argc, argv, "final_project");
 
  ros::NodeHandle nh;
  
  geometry_msgs::TransformStamped transformStamped;

  //Initialize Explorer, Follower, and ArucoNode class objects
  ArucoNode aruco_node(&nh);
  
  Explorer explorer(&nh, "explorer");

  Follower follower(&nh, "follower");

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }
  
  move_base_msgs::MoveBaseGoal explorer_goal;
  move_base_msgs::MoveBaseGoal follower_goal;


  //---Data to pull from array "m_fid" and "m_posit"
  int fiducial_id;
  double goal_x;
  double goal_y;
  bool delayed_start=false;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);

  static double final_angle{ 0 };
  
  //****Explorer Get Goals from aruco_lookup.yaml*****
  std::array<std::array<double,2>,4> goal_list={};
  // std::array<std::array<double,2>,4> exp_goal = {};

  XmlRpc::XmlRpcValue exp_goal;
  nh.getParam("/aruco_lookup_locations/target_1", exp_goal);
  // ROS_INFO_STREAM("Goal 1: " << nh.getParam("/aruco_lookup_locations/target_1",exp_goal));
  ROS_ASSERT(exp_goal.getType() == XmlRpc::XmlRpcValue::TypeArray);
  goal_list[0][0] = exp_goal[0];
  goal_list[0][1] = exp_goal[1];

  nh.getParam("/aruco_lookup_locations/target_2", exp_goal);
  // ROS_INFO_STREAM("Goal 2: " << nh.getParam("/aruco_lookup_locations/target_1",exp_goal));

  ROS_ASSERT(exp_goal.getType() == XmlRpc::XmlRpcValue::TypeArray);

  goal_list[1][0] = exp_goal[0];
  goal_list[1][1] = exp_goal[1];

  nh.getParam("/aruco_lookup_locations/target_3", exp_goal);
  // ROS_INFO_STREAM("Goal 1:" << nh.getParam("/aruco_lookup_locations/target_1",exp_goal));

  ROS_ASSERT(exp_goal.getType() == XmlRpc::XmlRpcValue::TypeArray);

  goal_list[2][0] = exp_goal[0];
  goal_list[2][1] = exp_goal[1];

  nh.getParam("/aruco_lookup_locations/target_4", exp_goal);
  // ROS_INFO_STREAM("Goal 1:" << nh.getParam("/aruco_lookup_locations/target_1",exp_goal));

  ROS_ASSERT(exp_goal.getType() == XmlRpc::XmlRpcValue::TypeArray);

  goal_list[3][0] = exp_goal[0];
  goal_list[3][1] = exp_goal[1];


  while (ros::ok()) {
    //*****EXPLORER*****//

    for(int i = 0; i < 4; i++){
      goal_x = goal_list[i][0];
      goal_y = goal_list[i][1];

      std::cout <<"Explorer Goal Pos " << i <<" : (";
      std::cout << goal_x << ", " << goal_y << ")\n";

      explorer_goal.target_pose.header.frame_id = "map";
      explorer_goal.target_pose.header.stamp = ros::Time::now();
      explorer_goal.target_pose.pose.position.x = goal_x;
      explorer_goal.target_pose.pose.position.y = goal_y;
      explorer_goal.target_pose.pose.orientation.w = 1.0;


      // aruco_node.m_initialize_subscribers();
      
      if (!explorer_goal_sent){
        ROS_INFO("Sending goal for explorer");
        explorer_client.sendGoalAndWait(explorer_goal);//this should be sent only once
        explorer_goal_sent = true;
      }

      // ROS_INFO_STREAM("fiducial id of current and last: " << aruco_node.fid_ids[i] << "\t" << aruco_node.fid_ids[i-1]);
     
      explorer_client.waitForResult();

      explorer.go_to_goal(goal_x,goal_y);

      explorer_client.waitForResult();
      ros::Duration(0.5).sleep();

    
      if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, explorer reached goal");
        explorer_goal_sent = false;
        follower.m_test=true;
        // aruco_node.marker_listen(tfBuffer, i);
        
        while(!aruco_node.marker_seen[i]){
          explorer.m_move(0.0,0.5);
          aruco_node.marker_listen(tfBuffer, i);
        }

        aruco_node.m_count ++; 
      }    
  
   }//Explorer for loop
    
    
    //---SEND Explorer Home---
    goal_x=-4;
    goal_y=2.5;
    ROS_INFO_STREAM("Returning Explorer Home.");
    explorer_goal.target_pose.pose.position.x = goal_x;
    explorer_goal.target_pose.pose.position.y = goal_y;

    if (!explorer_goal_sent){
        ROS_INFO("Sending goal for explorer");
        explorer_client.sendGoalAndWait(explorer_goal);//this should be sent only once
        explorer_goal_sent = true;
      }

    explorer_client.waitForResult();

    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, explorer reached goal");
      explorer_goal_sent = false;
    }

    explorer_client.waitForResult();

    ros::Duration(2.0).sleep();
    delayed_start=true;

    follower.setup_goals();
    std::cout <<"The first goal is: "<<follower.m_posit[0][0]<<"\t"<<follower.m_posit[0][1];

    //*****FOLLOWER*******//
    //--STEP 01. Let Follower Get home---
    //Wait until explorer "home" = (-4,2.5) before follower leave
    //---Send Follower to IDs O through 3 ---//
    for (int j=0; j<4 ;j++) {
      //--STEP 02. Set Goal for next ID--//
      fiducial_id=follower.m_fid.at(j);
      goal_x=follower.m_posit.at(j).at(0);
      goal_y=follower.m_posit.at(j).at(1);

      //Build goal for follower using Move_base
      follower_goal.target_pose.header.frame_id = "map";
      follower_goal.target_pose.header.stamp = ros::Time::now();
      follower_goal.target_pose.pose.position.x = goal_x;
      follower_goal.target_pose.pose.position.y = goal_y;
      follower_goal.target_pose.pose.orientation.w = 1.0;

      ROS_INFO("\nFollower moving to goal for ");
      std::cout << "Fiducial ID: "<< fiducial_id;
      std::cout << "\nLocated at: ("<< goal_x << ", "<<goal_y<<")\n";

      if (!follower_goal_sent){
        // ROS_INFO("Sending follower goal");
        follower_client.sendGoalAndWait(follower_goal);
        follower_goal_sent = true;
      }
      
      follower_client.waitForResult();
      
      if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, follower reached goal!!");
        follower_goal_sent = false;
      }

      ROS_INFO("Conducting Triage..");
      ros::Duration(0.5).sleep();

      //--STEP 03. Send Follower to Start Position (-4,3.5) --//
      if (j==3) {
        ROS_INFO("\nSending follower to Final Position");
        goal_x=-4;
        goal_y=3.5;
        follower_goal.target_pose.pose.position.x = goal_x;
        follower_goal.target_pose.pose.position.y = goal_y;

        if (!follower_goal_sent){
          // ROS_INFO("Sending follower goal");
          follower_client.sendGoalAndWait(follower_goal);
          follower_goal_sent = true;
        }

        follower_client.waitForResult();

        if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
          ROS_INFO("\n\n========================================");
          ROS_INFO("Hooray, follower reached FINAL position!");
          ROS_INFO("========================================");
        }
        ros::shutdown();

      } else {
          ros::spinOnce();
        }//if j==3 loop
    } //Follower "j" loop

    // }//if loop
  
  ros::spinOnce();
  loop_rate.sleep();
  }//while ros OK loop
}//main