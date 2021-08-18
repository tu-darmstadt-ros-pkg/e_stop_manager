#include "e_stop_manager/e_stop_manager.h"

namespace e_stop_manager
{

EStopManager::EStopManager( ros::NodeHandle& nh, ros::NodeHandle& pnh ) : nh_( nh ), pnh_( pnh )
{
  // get e_stop names and set init values
  e_stop_names_ = pnh_.param<std::vector<std::string>>( "e_stop_list", std::vector<std::string>());

  bool init_e_stop_value = pnh_.param<bool>( "init_e_stop_value", true );
  e_stop_values_.resize( e_stop_names_.size(), init_e_stop_value );


  // init service
  set_e_stop_service_ = pnh_.advertiseService( "set_e_stop", &EStopManager::SetEStopServiceCB, this );


  // init topics
  std::string list_topic = pnh_.param<std::string>( "e_stop_list_topic", "e_stop_list" );
  e_stop_list_pub_ = nh_.advertise<e_stop_manager_msgs::EStopList>( list_topic, 5, true );

  std::string e_stop_topic = pnh_.param<std::string>( "e_stop_topic", "e_stop" );
  e_stop_pub_ = nh_.advertise<std_msgs::Bool>( e_stop_topic, 5, true );
}


bool EStopManager::SetEStopServiceCB( e_stop_manager_msgs::SetEStop::Request& request,
                                      e_stop_manager_msgs::SetEStop::Response& response )
{
  bool name_found = false;

  // search for name, set its corresponding value
  for ( int i = 0; i < e_stop_names_.size(); i++ )
  {
    if ( e_stop_names_[i] == request.e_stop_name )
    {
      e_stop_values_[i] = request.e_stop_value;
      name_found = true;
      break;
    }
  }

  if ( !name_found )
  {
    ROS_ERROR_STREAM( "e_stop_manager: e_stop_name " + request.e_stop_name + " not found!" );
    response.result = response.INVALID_ESTOP_NAME;
    return true; // return true so that caller gets error code
  }



  // publish e_stop name and value lists
  e_stop_list_msg_.e_stop_names = e_stop_names_;
  e_stop_list_msg_.e_stop_values = e_stop_values_;
  e_stop_list_pub_.publish( e_stop_list_msg_ );

  // publish accumulated e_stop message
  // check if any values is true (estop pressed) --> publish true
  e_stop_msg_.data = std::any_of( e_stop_values_.begin(), e_stop_values_.end(), []( bool v ) { return v; } );
  e_stop_pub_.publish( e_stop_msg_ );


  response.result = response.SUCCESS;

  return true;
}
}