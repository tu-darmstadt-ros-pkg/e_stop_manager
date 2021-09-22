#include "e_stop_manager/e_stop_manager.h"

namespace e_stop_manager
{

EStopManager::EStopManager( ros::NodeHandle& nh, ros::NodeHandle& pnh ) : nh_( nh ), pnh_( pnh )
{

  e_stop_list_pub_ = pnh_.advertise<e_stop_manager_msgs::EStopList>( "e_stop_list", 5, true );


  XmlRpc::XmlRpcValue e_stop_topics;

  // check if parameter e_stop_topics exists and is a list
  if ( !pnh_.getParam( "e_stop_topics", e_stop_topics ) || e_stop_topics.getType() != XmlRpc::XmlRpcValue::TypeArray )
  {
    ROS_ERROR_NAMED( "e_stop_manager",
                     "Parameter e_stop_topic not provided or is not a list! E-stops will only published on e-stop-list topic!" );
  }
  else
  {

    // get topic names and names of e-stops which should be published on these topics
    for ( size_t i = 0; i < e_stop_topics.size(); ++i )
    {

      // here there is only one entry per list entry (e.g. not as in e_stop_list where there are two entries (name and value) per list entry)
      for ( auto& it : e_stop_topics[i] )
      {
        std::string topic_name = it.first;

        XmlRpc::XmlRpcValue e_stops = it.second;

        // get e-stop names for current topic
        std::vector<std::string> e_stop_names;

        for ( size_t j = 0; j < e_stops.size(); ++j )
        {
          e_stop_names.push_back( e_stops[j] );
        }

        ros::Publisher pub = pnh_.advertise<std_msgs::Bool>( topic_name, 5, true );

        // add publisher and its e-stop names to map
        e_stop_pub_.emplace( pub, e_stop_names );
      }
    }
  }

  XmlRpc::XmlRpcValue e_stop_list;

  // check if parameter e_stop_list exists and is a list
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

  // get names and values of e-stops
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


  // check if there was a valid e-stop
  if ( e_stop_list_msg_.names.empty() )
  {
    ROS_ERROR_NAMED( "e_stop_manager", "No valid e_stop in list!" );
    publishEStops( true );
    return;
  }



  // check if all names listed for the topics are in e_stop_list and erase others
  // (in order to avoid that these false names are searched in each call of publishEStops)
  for ( auto& pub : e_stop_pub_ )
  {
    pub.second.erase( std::remove_if( pub.second.begin(),
                                      pub.second.end(),
                                      [ & ]( const std::string& name )
                                      {
                                        // if name is not in list, remove it
                                        if ( std::find( e_stop_list_msg_.names.begin(),
                                                        e_stop_list_msg_.names.end(), name ) ==
                                             e_stop_list_msg_.names.end())
                                        {
                                          ROS_WARN_STREAM_NAMED( "e_stop_manager",
                                                                 "The e_stop named \""
                                                                   << name
                                                                   << "\" was in e_stop_topics but not in e_stop_list. Ignored." );
                                          return true;
                                        }

                                        // if name was found, return false (= do not remove it)
                                        return false;
                                      } ),
                      pub.second.end());
  }



  // init service
  set_e_stop_service_ = pnh_.advertiseService( "set_e_stop", &EStopManager::setEStopServiceCB, this );

  publishEStops();
  ROS_INFO_NAMED( "e_stop_manager", "e_stop_manager initialized!" );
}


bool EStopManager::setEStopServiceCB( e_stop_manager_msgs::SetEStop::Request& request,
                                      e_stop_manager_msgs::SetEStop::Response& response )
{

  bool name_found = false;

  // search for name, set its corresponding value
  for ( int i = 0; i < e_stop_list_msg_.names.size(); i++ )
  {
    if ( e_stop_list_msg_.names[i] == request.name )
    {
      // if value has not changed, do nothing and return
      if(e_stop_list_msg_.values[i] == request.value)
      {
        response.result = response.SUCCESS;
        return true;
      }

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

  ROS_INFO_STREAM_NAMED("e_stop_manager", "EStop '" << request.name + "' sent value: " << (int)request.value << ".");

  response.result = response.SUCCESS;
  return true;
}

void EStopManager::publishEStops( bool force_e_stop )
{

  // publish e_stop name and value lists
  e_stop_list_pub_.publish( e_stop_list_msg_ );


  // publish accumulated e_stop messages according to the e-stops listed for each topic
  // check if any value is true (e-stop pressed) --> publish true
  for ( auto pub : e_stop_pub_ )
  {
    std_msgs::Bool msg;
    msg.data = force_e_stop ||
               std::any_of( pub.second.begin(), pub.second.end(),
                            [ & ]( const std::string& name )
                            {
                              // find name (and its index) in list to retrieve its value
                              auto it = std::find( e_stop_list_msg_.names.begin(), e_stop_list_msg_.names.end(), name );
                              if ( it != e_stop_list_msg_.names.end())
                              {
                                int idx = it - e_stop_list_msg_.names.begin();
                                return (bool) e_stop_list_msg_.values[idx];
                              }
                              // if name not found, return false
                              return false;
                            } );

    pub.first.publish( msg );
  }
}
}
