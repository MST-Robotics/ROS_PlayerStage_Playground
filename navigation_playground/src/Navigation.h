/*******************************************************************************
* @file Navigation.h
* @author Jason Gassel <jmgkn6@mst.edu>
* @version 1.0
* @date 9/28/12
* @brief Navigation class for stageros playground.
******************************************************************************/
#ifndef NAVIGATION_H
#define NAVIGATION_H

/***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"

/***********************************************************
* Message includes
***********************************************************/
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/***********************************************************
* Other includes
***********************************************************/
#include <string>
#include <vector>

class Navigation
{
private:
    /*-----------------------------------
	ROS variables
	-----------------------------------*/
    ros::Subscriber s_odom;
    ros::Subscriber s_base_scan;
    ros::Subscriber s_base_pose_ground_truth;
    ros::Publisher p_cmd_vel;
    
    void initializeSubsPubs(ros::NodeHandle n);
    void publishCmdVel();
    
    /*-----------------------------------
	Velocity and sensor data variables
	-----------------------------------*/
    double m_linear;
    double m_angular;
    
    struct Data3 { double x, y, z; };
    struct Data4 { double x, y, z, w; };
    struct Odom
    {
        ros::Time timestamp;
        std::string frameId;
        Data3 posePosition;
        Data4 poseOrientation;
        double poseCovariance[36];
        Data3 twistLinear;
        Data3 twistAngular;
        double twistCovariance[36];
    };
    Odom m_odom;
    Odom m_basePoseGroundTruth;
    
    struct LaserScan
    {
        ros::Time timestamp;
        std::string frameId;
        double angleMin;
        double angleMax;
        double angleIncrement;
        double timeIncrement;
        double scanTime;
        double rangeMin;
        double rangeMax;
        std::vector<double> ranges;
        std::vector<double> intensities;
    };
    LaserScan m_baseScan;

public:
    Navigation(ros::NodeHandle n);
    ~Navigation();
    bool run();
    
    /*-----------------------------------
    ROS methods
    -----------------------------------*/
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void baseScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void basePoseGroundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

#endif //NAVIGATION_H
