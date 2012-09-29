/*******************************************************************************
* @file Navigation.cpp
* @author Jason Gassel <jmgkn6@mst.edu>
* @version 1.0
* @date 9/28/12
* @brief Navigation class for stageros playground.
******************************************************************************/
#include "Navigation.h"

/******************************************************************************
* ROS wiki references for messages below, odom and base_pose_ground_truth are
* both nav_msgs/Odometry messages. The* difference is that odom is a simulated
* value obtained using the robot sensors and base_pose_ground_truth is an
* unrealisticly perfect pose.
*
* The messages are copied in their entirety and can be accessed through the
* member variables, prefixed with m_. See the .h file and the references below
* for the contents of the messages.
*
* stageros: (includes tf transforms)
* http://www.ros.org/wiki/stage
*
* nav_msgs/Odometry:
* http://www.ros.org/doc/api/nav_msgs/html/msg/Odometry.html
*
* sensor_msgs/LaserScan:
* http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html
*
* geometry_msgs/Twist:
* http://www.ros.org/doc/api/geometry_msgs/html/msg/Twist.html
*
* (Note that the sensors are not perfectly aligned with the center of the robot,
* transforms are provided by stageros to allow you to relatively easily
* transform the sensor data using the tf ROS stack. This probably won't be
* necessary to get started but it will be helpful to learn.)
******************************************************************************/

Navigation::Navigation(ros::NodeHandle n)
{
    //Subscribe to ROS messages for sensor data and create publisher for
    //  velocity commands
    initializeSubsPubs(n);
}

Navigation::~Navigation()
{
}

bool Navigation::run()
{   
    //TODO add robot navigation code here, currently just drives in a circle
    m_linear = 1.0;
    m_angular = -1.0;
    
    //Publish the velocity command then return success
    publishCmdVel();
    return true;
}




/******************************************************************************
* Probably not necessary to modify the below but look at them to see how the
* ROS functionality works.
******************************************************************************/
void Navigation::initializeSubsPubs(ros::NodeHandle n)
{
    //Subscribe to messages being published by stageros
    s_odom = n.subscribe("/odom", 1, &Navigation::odomCallback, this);
    s_base_scan = n.subscribe("/base_scan", 1, &Navigation::baseScanCallback, this);
    s_base_pose_ground_truth = n.subscribe("/base_pose_ground_truth", 1, &Navigation::basePoseGroundTruthCallback, this);
    
    //Advertise our messages so stageros can subscribe
    p_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
}

void Navigation::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    m_odom.timestamp = msg->header.stamp;
    m_odom.frameId = msg->header.frame_id;
    m_odom.posePosition.x = msg->pose.pose.position.x;
    m_odom.posePosition.y = msg->pose.pose.position.y;
    m_odom.posePosition.z = msg->pose.pose.position.z;
    m_odom.poseOrientation.x = msg->pose.pose.orientation.x;
    m_odom.poseOrientation.y = msg->pose.pose.orientation.y;
    m_odom.poseOrientation.z = msg->pose.pose.orientation.z;
    m_odom.poseOrientation.w = msg->pose.pose.orientation.w;
    m_odom.twistLinear.x = msg->twist.twist.linear.x;
    m_odom.twistLinear.x = msg->twist.twist.linear.y;
    m_odom.twistLinear.x = msg->twist.twist.linear.z;
    m_odom.twistLinear.x = msg->twist.twist.angular.x;
    m_odom.twistLinear.x = msg->twist.twist.angular.y;
    m_odom.twistLinear.x = msg->twist.twist.angular.z;
    for(int i=0; i<36; i++)
    {
        m_odom.poseCovariance[i] = msg->pose.covariance[i];
        m_odom.twistCovariance[i] = msg->pose.covariance[i];
    }
}

void Navigation::baseScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    m_baseScan.timestamp = msg->header.stamp;
    m_baseScan.frameId = msg->header.frame_id;
    m_baseScan.angleMin = msg->angle_min;
    m_baseScan.angleMax = msg->angle_max;
    m_baseScan.angleIncrement = msg->angle_increment;
    m_baseScan.timeIncrement = msg->time_increment;
    m_baseScan.scanTime = msg->scan_time;
    m_baseScan.rangeMin = msg->range_min;
    m_baseScan.rangeMax = msg->range_max;
    m_baseScan.ranges.clear();
    m_baseScan.intensities.clear();
    for(unsigned int i=0; i<msg->ranges.size(); i++)
    {
        m_baseScan.ranges.push_back(msg->ranges[i]);
        m_baseScan.intensities.push_back(msg->intensities[i]);
    }
}

void Navigation::basePoseGroundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    m_basePoseGroundTruth.timestamp = msg->header.stamp;
    m_basePoseGroundTruth.frameId = msg->header.frame_id;
    m_basePoseGroundTruth.posePosition.x = msg->pose.pose.position.x;
    m_basePoseGroundTruth.posePosition.y = msg->pose.pose.position.y;
    m_basePoseGroundTruth.posePosition.z = msg->pose.pose.position.z;
    m_basePoseGroundTruth.poseOrientation.x = msg->pose.pose.orientation.x;
    m_basePoseGroundTruth.poseOrientation.y = msg->pose.pose.orientation.y;
    m_basePoseGroundTruth.poseOrientation.z = msg->pose.pose.orientation.z;
    m_basePoseGroundTruth.poseOrientation.w = msg->pose.pose.orientation.w;
    m_basePoseGroundTruth.twistLinear.x = msg->twist.twist.linear.x;
    m_basePoseGroundTruth.twistLinear.x = msg->twist.twist.linear.y;
    m_basePoseGroundTruth.twistLinear.x = msg->twist.twist.linear.z;
    m_basePoseGroundTruth.twistLinear.x = msg->twist.twist.angular.x;
    m_basePoseGroundTruth.twistLinear.x = msg->twist.twist.angular.y;
    m_basePoseGroundTruth.twistLinear.x = msg->twist.twist.angular.z;
    for(int i=0; i<36; i++)
    {
        m_basePoseGroundTruth.poseCovariance[i] = msg->pose.covariance[i];
        m_basePoseGroundTruth.twistCovariance[i] = msg->pose.covariance[i];
    }
}

void Navigation::publishCmdVel()
{
    //Publish twist message (sets velocity, stage robot is subscribed)    
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = m_linear;
    cmd_vel.angular.z = m_angular;    
    p_cmd_vel.publish(cmd_vel);    
}

