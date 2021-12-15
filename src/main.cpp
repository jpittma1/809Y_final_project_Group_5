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


// forward declaration
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
  // ros::init(argc, argv, "explorer");

  ros::NodeHandle nh;
  
  geometry_msgs::TransformStamped transformStamped;

  tf2_ros::Buffer tfBuffer;

  // std::string robot_name;
  // std::string robot_name_follow;
  ArucoNode aruco_node(&nh);

  //Initialize Follower and Explorer class objects
  Explorer explorer(&nh, "explorer");
  // ros::Duration(1.0).sleep();
  Follower follower(&nh, "follower");
  

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // ros::Duration(1.0).sleep();

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

  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);

  static double final_angle{ 0 };
  
  //EXplorer Get Goals from aruco_lookup.yaml
  // std::array<std::array<double,2>,4> goal_list = explorer.get_goals();
  //hardcode
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
      
      explorer_goal_sent = false;
      
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
      // explorer_client.waitForResult();

      if (!explorer_goal_sent){
        ROS_INFO("Sending goal for explorer");
        explorer_client.sendGoal(explorer_goal);//this should be sent only once
        explorer_goal_sent = true;
      }

      explorer_client.waitForResult();

      explorer.go_to_goal(goal_x,goal_y);


      // ROS_INFO_STREAM("fiducial id of current and last: " << aruco_node.fid_ids[i] << "\t" << aruco_node.fid_ids[i-1]);
      
      explorer_client.waitForResult();
      ros::Duration(0.5).sleep();

    
      if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, explorer reached goal");
        
        // follower.m_test = true;
        
        while(!aruco_node.marker_seen[i]){
          explorer.m_move(0.0,0.5);
          aruco_node.marker_listen(tfBuffer, i);
        }
        aruco_node.transformed_locs[i][0] = follower.temp_x;
        aruco_node.transformed_locs[i][1] = follower.temp_y;

        ROS_INFO_STREAM("X and Y transform: "<<follower.temp_x<<", "<<follower.temp_y);

        // ROS_INFO_STREAM("Looking for goal: " << goal_list[i+1][0] <<  goal_list[i+1][1]
        //               "\n Last Fid:" <<  );


        // aruco_node.m_count ++;

        // explorer.stop();


      }
    
    }
    //---SEND Explorer Home---
    goal_x=-4;
    goal_y=2.5;
    ROS_INFO_STREAM("Returning Explorer Home.");

    explorer_goal.target_pose.header.frame_id = "map";
    explorer_goal.target_pose.header.stamp = ros::Time::now();
    explorer_goal.target_pose.pose.position.x = goal_x;
    explorer_goal.target_pose.pose.position.y = goal_y;
    explorer_goal.target_pose.pose.orientation.w = 1.0;

    explorer_client.sendGoal(explorer_goal);//this should be sent only once

    explorer_client.waitForResult();

    ros::Duration(2.0).sleep();
    delayed_start=true;

    // follower.setup_goals();
    //*****FOLLOWER*******//
    
    //Wait until explorer "home" = (-4,2.5) before follower leave
    // if (delayed_start){
      //---Send Follower to IDs O through 3 ---//
      std::array<std::array<double,2>,4> follow_list={};

      for (int k=0; k<4;k++){
        
        if (aruco_node.fid_ids.at(k)==0){ 
            follow_list[0]=goal_list[k];
        //     m_posit.at(0)={a_node.transformed_locs[0][0], a_node.transformed_locs[0][1]};
        } else if(aruco_node.fid_ids.at(k)==1) {
            follow_list[1]=goal_list[k];        
        //      m_posit.at(1)={a_node.transformed_locs[1][0], a_node.transformed_locs[1][1]};
        } else if (aruco_node.fid_ids.at(k)==2) {
            follow_list[2]=goal_list[k];        
        //     m_posit.at(2)={a_node.transformed_locs[2][0], a_node.transformed_locs[2][1]};
        } else if (aruco_node.fid_ids.at(k)==3){
            follow_list[3]=goal_list[k];        
      }}
      for (int j=0; j<4 ;j++) {
        follower_goal_sent = false;
        //--Set Goal for next ID--//
        fiducial_id=follower.m_fid.at(j);
        // goal_x=follower.m_posit.at(j).at(0);
        // goal_y=follower.m_posit.at(j).at(1);

        goal_x = goal_list[j][0];
        goal_y = goal_list[j][1];
        ROS_INFO_STREAM("The first goal is: "<<goal_x<<"\t"<<goal_y);

        
        //Build goal for follower using Move_base
        follower_goal.target_pose.header.frame_id = "map";
        follower_goal.target_pose.header.stamp = ros::Time::now();
        follower_goal.target_pose.pose.position.x = goal_x;
        follower_goal.target_pose.pose.position.y = goal_y;
        follower_goal.target_pose.pose.orientation.w = 1.0;

        // follower_client.waitForResult();

        if (!follower_goal_sent){
          ROS_INFO("Sending follower goal");
          follower_client.sendGoalAndWait(follower_goal);
          follower_goal_sent = true;
        }
        
        follower_client.waitForResult();
        
        
          ROS_INFO("Follower moving to goal");
          std::cout << "\nFiducial ID: "<< fiducial_id;
          std::cout << "\nLocated at: ("<< goal_x << ", "<<goal_y<<")";
          follower.go_to_goal(goal_x, goal_y);
          ROS_INFO("Hooray, follower reached goal");
        // }

        ros::Duration(0.5).sleep();
        //---Send Follower to Start Position (-4,3.5) ---//
        
     } //for "j" loop
     ROS_INFO("Sending follower to Start Position");
          goal_x=-4;
          goal_y=3.5;
          //TBD if need for going home
          follower_goal.target_pose.header.frame_id = "map";
          follower_goal.target_pose.header.stamp = ros::Time::now();
          follower_goal.target_pose.pose.position.x = goal_x;
          follower_goal.target_pose.pose.position.y = goal_y;
          follower_goal.target_pose.pose.orientation.w = 1.0;
          follower_client.sendGoalAndWait(follower_goal);
          follower.go_to_goal(goal_x, goal_y);
          ROS_INFO("Hooray, follower reached starting position");
          ros::shutdown();

    // //---Send Follower to Start Position (-4,3.5) ---//
    // ROS_INFO("Sending follower to Start Position");
    // goal_x=-4;
    // goal_y=3.5;
    // //TBD if need for going home
    // // follower_goal.target_pose.header.frame_id = "map";
    // // follower_goal.target_pose.header.stamp = ros::Time::now();
    // // follower_goal.target_pose.pose.position.x = goal_x;
    // // follower_goal.target_pose.pose.position.y = goal_y;
    // // follower_goal.target_pose.pose.orientation.w = 1.0;
    // follower.go_to_goal(goal_x, goal_y);
    // ROS_INFO("Hooray, follower reached starting position");
    // ros::shutdown();
  
    // }//if loop
    ros::spinOnce();
    loop_rate.sleep();
    
  }//while ros OK loop
}//main