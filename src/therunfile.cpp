/**
The run file 
 */
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <fstream>  
#include <string>
#include "kalman/kalman.h"
#include "kalman/Roshdl.h"



int main(int argc, char** argv)
{

   if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
  {
     ros::console::notifyLoggerLevelsChanged();
  }

  ROS_DEBUG("initializing");
  ros::init(argc, argv, "the_run_file");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");
  Roshdl handler_ros(nh, nh_local);
  ROS_DEBUG("initializing1");
  double frequency;
  nh_local.param<double>("frequency", frequency, 25);
  ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
  ros::Rate rate(frequency);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }


  

  return 0;
}

  


  
  
