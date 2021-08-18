#include "e_stop_manager/e_stop_manager.h"

namespace e_stop_manager
{

EStopManager::EStopManager( ros::NodeHandle &nh, ros::NodeHandle &pnh ) : nh_( nh ), pnh_( pnh )
{
  e_stop_pub_ = pnh_.advertise<std_msgs::Bool>( "e_stop", 5, true );
  e_stop_list_pub_ = pnh_.advertise<e_stop_manager_msgs::EStopList>( "e_stop_list", 5, true );

  XmlRpc::XmlRpcValue e_stop_list;
  if ( !pnh_.getParam( "e_stop_list", e_stop_list ))
  {
    ROS_ERROR_NAMED( "e_stop_manager", "Parameter e_stop_list not provided!" );
    publishEStops( true );
    return;
  }
  if ( e_stop_list.getType() != XmlRpc::XmlRpcValue::TypeArray )
  {
    ROS_ERROR_NAMED( "e_stop_manager", "Parameter e_stop_list is not a list!" );
    publishEStops( true );
    return;
  }

  for ( size_t i = 0; i < e_stop_list.size(); ++i )
  {
    XmlRpc::XmlRpcValue entry = e_stop_list[i];
    if ( !entry.hasMember( "name" ))
    {
      ROS_WARN_NAMED( "e_stop_manager", "Parameter e_stop_list contains entry without name! Ignored." );
      continue;
    }
    e_stop_list_msg_.names.push_back( entry["name"] );
    // If value is omitted, set to true per default
    e_stop_list_msg_.values.push_back( !entry.hasMember( "value" ) || (bool) entry["value"] );
  }
  if ( e_stop_list.getType() != XmlRpc::XmlRpcValue::TypeArray )
  {
    ROS_ERROR_NAMED( "e_stop_manager", "No valid e_stop in list!" );
    publishEStops( true );
    return;
  }

  // init service
  set_e_stop_service_ = pnh_.advertiseService( "set_e_stop", &EStopManager::setEStopServiceCB, this );

  publishEStops();
  ROS_INFO_NAMED( "e_stop_manager", "e_stop_manager initialized!" );
}


bool EStopManager::setEStopServiceCB( e_stop_manager_msgs::SetEStop::Request &request,
                                      e_stop_manager_msgs::SetEStop::Response &response )
{
  bool name_found = false;

  // search for name, set its corresponding value
  for ( int i = 0; i < e_stop_list_msg_.names.size(); i++ )
  {
    if ( e_stop_list_msg_.names[i] == request.name )
    {
      e_stop_list_msg_.values[i] = request.value;
      name_found = true;
      break;
    }
  }

  if ( !name_found )
  {
    ROS_ERROR_STREAM_NAMED( "e_stop_manager", "EStop : '" << request.name << "' not found!" );
    response.result = response.INVALID_ESTOP_NAME;
    return true; // return true so that caller gets error code
  }

  publishEStops();

  response.result = response.SUCCESS;
  return true;
}

void EStopManager::publishEStops( bool force_e_stop )
{

  // publish e_stop name and value lists
  e_stop_list_pub_.publish( e_stop_list_msg_ );
  // publish accumulated e_stop message
  // check if any values is true (estop pressed) --> publish true
  std_msgs::Bool msg;
  msg.data = force_e_stop ||
             std::any_of( e_stop_list_msg_.values.begin(), e_stop_list_msg_.values.end(), []( bool v ) { return v; } );
  e_stop_pub_.publish( msg );
}
}
