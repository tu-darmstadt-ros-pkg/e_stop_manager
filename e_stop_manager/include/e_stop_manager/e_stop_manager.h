#ifndef E_STOP_MANAGER_E_STOP_MANAGER_H
#define E_STOP_MANAGER_E_STOP_MANAGER_H

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
  void publishEStops( bool force_e_stop = false );

  bool setEStopServiceCB( e_stop_manager_msgs::SetEStop::Request& request,
                          e_stop_manager_msgs::SetEStop::Response& response );


  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;


  ros::ServiceServer set_e_stop_service_;

  e_stop_manager_msgs::EStopList e_stop_list_msg_;
  ros::Publisher e_stop_list_pub_;

  std::map<ros::Publisher, std::vector<std::string>> e_stop_pub_;
};
}

#endif
