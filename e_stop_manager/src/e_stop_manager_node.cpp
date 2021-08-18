#include <ros/ros.h>
#include "e_stop_manager/e_stop_manager.h"

int main( int argc, char** argv )
{
  ros::init( argc, argv, ROS_PACKAGE_NAME );

  ros::NodeHandle nh;
  ros::NodeHandle pnh( "~" );

  e_stop_manager::EStopManager e_stop_manager( nh, pnh );

  ros::spin();

  return 0;
}