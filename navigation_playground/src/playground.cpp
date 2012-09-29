/*******************************************************************************
* @file playground.cpp
* @author Jason Gassel <jmgkn6@mst.edu>
* @version 1.0
* @date 9/28/12
* @brief Main file for stageros playground.
******************************************************************************/
#include "ros/ros.h"
#include "Navigation.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "playground");
    ros::NodeHandle n;
    Navigation nav( n );
   
    ros::Rate loop_rate(2);    
 
    while( ros::ok() && nav.run() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

