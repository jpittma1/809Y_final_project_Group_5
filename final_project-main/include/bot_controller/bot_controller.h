#ifndef BOT_CONTROLLER_H
#define BOT_CONTROLLER_H

#include <geometry_msgs/Twist.h>   //for geometry_msgs::Twist
#include <nav_msgs/Odometry.h>     //for nav_msgs::Odometry
#include <sensor_msgs/LaserScan.h> //for laser scans
#include <ros/ros.h>
#include <utility>
#include <tf/transform_datatypes.h> //to manipulate quaternions


/**
 * @brief Controller class to drive a turtlebot.
 *
 */
class Bot_Controller
{
public:
    // main() will need to instantiate a ROS nodehandle, then pass it to the constructor
    Bot_Controller(ros::NodeHandle *nodehandle, const std::string &robot_name);
    //parent overrides
    virtual void publish_velocities(const geometry_msgs::Twist &msg) =0;
    virtual void drive_straight(double distance, bool direction)=0;
    virtual void rotate(double angle_to_rotate, bool direction, double final_angle)=0;
    virtual bool go_to_goal(double x, double y) =0; 
    virtual void stop() =0;
    virtual double compute_expected_final_yaw(bool direction, double angle_to_rotate) =0;
    virtual double compute_yaw_deg() =0;
    virtual double compute_yaw_rad() =0;
    virtual double convert_rad_to_deg(double angle) =0;
    
    virtual ~Bot_Controller() {};


    const double get_current_x(){
        return m_location.first;
    }

    const double get_current_y(){
        return m_location.second;
    }

protected: //for inheritance
    ros::NodeHandle m_nh;

    ros::Subscriber m_pose_subscriber;
    ros::Subscriber m_scan_subscriber;
    ros::Subscriber m_fiducial_subscriber;
    ros::Publisher m_velocity_publisher;
    ros::Publisher m_bot_status_publisher; //slide 19 task

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
    double m_yaw;                                                      //rad
    void m_pose_callback(const nav_msgs::Odometry::ConstPtr &msg);     // prototype for callback of example subscriber
    void m_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg); // prototype for callback of example subscriber
    // void m_fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg);
        
    void m_initialize_subscribers();
    void m_initialize_publishers();
    double m_compute_distance(const std::pair<double, double> &a, const std::pair<double, double> &b);
    void m_move(double linear, double angular);

    /**
     * @brief Normalizes the angle to be 0 to 2*M_PI
     * 
     * @param angle Angle to normalize (rad)
     * @return double Normalized angle (rad)
     */
    double m_normalize_angle_positive(double angle);

    /**
     * @brief Normalizes the angle to be -M_PI circle to +M_PI circle
     * 
     * @param angle Angle to normalize (rad)
     * @return double Normalized angle (rad)
     */
    double m_normalize_angle(double angle);
};
#endif