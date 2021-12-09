#include "..include/bot_controller/bot_controller.h"
#include "..include/follower/follower.h"
#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <array>
#include <iostream>
#include <algorithm>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


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

  //An array to store marker locations from explorer
  //4 ID arrays with ID, x, y, and orientation stored in each array
  std::array<std::array<int, 4>, 4> markers{};

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  ros::init(argc, argv, "follower_bot");
  
  std::string robot_name;
  if (nh.hasParam("robot_name")) {
    nh.getParam("robot_name", robot_name);
    ROS_INFO_STREAM("robot name: " << robot_name);
  }
  else {
    print_usage("missing argument: _robot_name:= <name>");
  }
  
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

    // I suggest you create an std::array of size 4 and store each marker,
    // based on its ID, inside this array. Marker with ID 0 will be stored at index 0 in the array, marker
    // with ID 1 will be stored at index 1 in the array, and so on. The ID of the detected marker can be
    // retrieved from the field fiducial_id from data published to the Topic /fiducial_transforms.
  
  //Build goal for explorer
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.position.x = 7.710214;//
  explorer_goal.target_pose.pose.position.y = -1.716889;//
  explorer_goal.target_pose.pose.orientation.w = 1.0;

  //Build goal for follower
  follower_goal.target_pose.header.frame_id = "map";
  follower_goal.target_pose.header.stamp = ros::Time::now();
  follower_goal.target_pose.pose.position.x = -0.289296;//
  follower_goal.target_pose.pose.position.y = -1.282680;//
  follower_goal.target_pose.pose.orientation.w = 1.0;


  // explorer_client.waitForResult();

  // ROS_INFO("Sending goal");
  // follower_client.sendGoal(follower_goal);
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
  //---Data to pull from array "markers"
  int fiducial_id;
  double goal_x;
  double goal_y;

  // if (motion_type == "g")
  // {
  //   if (!nh.hasParam("goal_x"))
  //     print_usage("missing argument: _goal_x:=<double>");
  //   else
  //     nh.getParam("goal_x", goal_x);

  //   if (!nh.hasParam("goal_y"))
  //     print_usage("missing argument: _goal_y:=<double>");
  //   else
  //     nh.getParam("goal_y", goal_y);
  // }

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);

  static double final_angle{ 0 };

  while (ros::ok()) {
    //*****EXPLORER*****//
    if (!explorer_goal_sent)     {
      ROS_INFO("Sending goal for explorer");
      explorer_client.sendGoal(explorer_goal);//this should be sent only once
      explorer_goal_sent = true;
    }
    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, robot reached goal");
    }
    // if (!follower_goal_sent) {
    //   ROS_INFO("Sending goal for follower");
    //   follower_client.sendGoal(follower_goal);//this should be sent only once
    //   follower_goal_sent = true;
    // }
    // if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //   ROS_INFO("Hooray, robot reached goal");
    // }

    //---See and store Marker Locations--
    markers.at(marker).at(0)=m_fiducial_subscriber; //Fiducial ID
    markers.at(marker).at(2)=m_orientation;  //orientation
    try {
        transformStamped = tfBuffer.lookupTransform("map", "marker_frame",
        ,→ ros::Time(0));
        ROS_INFO_STREAM("marker in /map frame: ["
          << transformStamped.transform.translation.x << ","
          << transformStamped.transform.translation.y << ","
          << transformStamped.transform.translation.z << "]"
        );
        //!!NEED TO DEFINE "marker"!!!!
        markers.at(marker).at(1) =transformStamped.transform.translation.x;  //x
        markers.at(marker).at(2)=transformStamped.transform.translation.y;  //y

      }
      catch (tf2::TransformException& ex) {
          ROS_WARN("%s", ex.what());
          ros::Duration(1.0).sleep();
      }
    
    //*****FOLLOWER*******//
    //---STEP 01a. Sort/organize std::array to go IDs 0->4
    for (int h=0;h<4;h++) {
      std::cout <<"Pre-sort Array " << h << "is" << markers.at(h)<<"\n";
    }
    std::sort(markers.begin(), markers.end());
    ROS_INFO("Array sorted by IDs");
    //Print to test array sorted correctly
    for (int j=0;j<4;j++) {
      std::cout <<"Post-sort Array " << j << "is" << markers.at(j)<<"\n";
    }

    //---STEP 02. Send Follower to IDs O through 3 ---//
    for (int i=0;i<4;i++) {
      //--Set Goal for next ID--//
      fiducial_id=array.at(i).at(0);
      goal_x=array.at(i).at(1);
      goal_y=array.at(i).at(2);
      
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
        std::cout << "\nFiducial ID: "<< fiducual_id;
        std::cout << "\nLocated at: ("<< goal_x << ", "<<goal_y<<")";
        follower.go_to_goal(goal_x, goal_y);
        ROS_INFO("Hooray, follower reached goal");
      }
    }

    broadcast();
    listen(tfBuffer);
    ros::spinOnce();
    loop_rate.sleep();

    //---STEP 03. Send Follower to Start Position (-4,3.5) ---//
    ROS_INFO("Sending follower to Start Position");
    goal_x=-4;
    goal_y=3.5;
    follower.go_to_goal(goal_x, goal_y);
    ROS_INFO("Hooray, follower reached starting position");
    ros::shutdown();
  }


}