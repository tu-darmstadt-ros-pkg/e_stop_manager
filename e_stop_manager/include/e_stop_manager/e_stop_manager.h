#pragma once

#include <ros/ros.h>
#include "std_msgs/Bool.h"

#include "e_stop_manager_msgs/SetEStop.h"
#include "e_stop_manager_msgs/EStopList.h"

namespace e_stop_manager
{

class EStopManager
{
public:
  EStopManager( ros::NodeHandle& nh, ros::NodeHandle& pnh );

private:

  bool SetEStopServiceCB( e_stop_manager_msgs::SetEStop::Request& request,
                          e_stop_manager_msgs::SetEStop::Response& response );


  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;


  ros::ServiceServer set_e_stop_service_;

  e_stop_manager_msgs::EStopList e_stop_list_msg_;
  ros::Publisher e_stop_list_pub_;

  std_msgs::Bool e_stop_msg_;
  ros::Publisher e_stop_pub_;

  std::vector<std::string> e_stop_names_;
  std::vector<uint8_t> e_stop_values_;
};
}



