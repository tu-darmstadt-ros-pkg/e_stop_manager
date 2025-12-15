#include "e_stop_manager/e_stop_manager.h"

#include <algorithm>
#include <cctype>
#include <functional>
#include <set>

namespace e_stop_manager
{

EStopManager::EStopManager( const rclcpp::NodeOptions &options ) : node_( std::make_shared<rclcpp::Node>( "e_stop_manager", options ) )
{
  param_listener_ = std::make_unique<e_stop_manager_config::ParamListener>( node_ );
  params_ = param_listener_->get_params();

  const auto reliable_transient_qos = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).reliable().transient_local();

  e_stop_list_pub_ = node_->create_publisher<e_stop_manager_msgs::msg::EStopList>( "~/e_stop_list", reliable_transient_qos );
  std::set<std::string> aggregated_topics;
  const std::string overall_aggregated_topic = "overall";
  aggregated_topics.insert( overall_aggregated_topic );

  e_stop_list_msg_.names = params_.e_stop_names;

  for ( const auto &e_stop_name : params_.e_stop_names ) {
    const auto config_it = params_.e_stop_config.e_stop_names_map.find( e_stop_name );
    if ( config_it == params_.e_stop_config.e_stop_names_map.end() ) {
      RCLCPP_ERROR( node_->get_logger(), "Configuration for e-stop '%s' not found.", e_stop_name.c_str() );
      e_stop_state_[e_stop_name] = true; // fail-safe: treat unknown as triggered
      e_stop_list_msg_.values.push_back( true );
      aggregated_members_[overall_aggregated_topic].push_back( e_stop_name );
      continue;
    }

    const auto &config = config_it->second;

    // sanitize aggregated topic name
    // if starts with ~ or /, remove that character
    std::string fixed_topic = sanitizeTopicName( config.aggregated_topic );

    aggregated_topics.insert( fixed_topic );
    e_stop_state_[e_stop_name] = config.initial_value;
    e_stop_list_msg_.values.push_back( config.initial_value );
    aggregated_members_[fixed_topic].push_back( e_stop_name );
    aggregated_members_[overall_aggregated_topic].push_back( e_stop_name );

    if ( config.tracked_topic ) {
      tracked_subscriptions_[e_stop_name] = node_->create_subscription<std_msgs::msg::Bool>(
          "~/" + e_stop_name, reliable_transient_qos,
          [this, e_stop_name]( const std_msgs::msg::Bool::SharedPtr msg ) { this->handleTrackedUpdate( e_stop_name, msg->data ); } );
    } else {
      managed_publishers_[e_stop_name] = node_->create_publisher<std_msgs::msg::Bool>( "~/" + e_stop_name, reliable_transient_qos );
    }
  }

  for ( const auto &aggregated_topic : aggregated_topics ) {
    const auto full_topic = "~/aggregated_state/" + aggregated_topic;
    aggregated_publishers_[aggregated_topic] = node_->create_publisher<std_msgs::msg::Bool>( full_topic, reliable_transient_qos );
    e_stop_list_msg_.aggregated_names.push_back( aggregated_topic );
  }

  set_e_stop_service_ = node_->create_service<e_stop_manager_msgs::srv::SetEStop>(
      "~/set_e_stop", std::bind( &EStopManager::setEStopServiceCB, this, std::placeholders::_1, std::placeholders::_2 ) );

  publishEStops();
  RCLCPP_INFO( node_->get_logger(), "e_stop_manager initialized with %zu e-stops.", params_.e_stop_names.size() );
}

void EStopManager::handleTrackedUpdate( const std::string &name, bool value )
{
  const auto state_it = e_stop_state_.find( name );
  if ( state_it == e_stop_state_.end() ) {
    RCLCPP_ERROR( node_->get_logger(), "Received update for unknown tracked e-stop '%s'.", name.c_str() );
    return;
  }

  if ( state_it->second == value ) {
    return;
  }

  RCLCPP_INFO( node_->get_logger(), "Tracked e-stop '%s' updated to %d.", name.c_str(), value );
  state_it->second = value;
  publishEStops();
}

void EStopManager::setEStopServiceCB( const std::shared_ptr<e_stop_manager_msgs::srv::SetEStop::Request> request,
                                      std::shared_ptr<e_stop_manager_msgs::srv::SetEStop::Response> response )
{

  const auto config_it = params_.e_stop_config.e_stop_names_map.find( request->name );
  const auto state_it = e_stop_state_.find( request->name );

  if ( config_it == params_.e_stop_config.e_stop_names_map.end() || state_it == e_stop_state_.end() ) {
    RCLCPP_ERROR( node_->get_logger(), "E-stop '%s' not found.", request->name.c_str() );
    response->result = e_stop_manager_msgs::srv::SetEStop_Response::INVALID_ESTOP_NAME;
    return;
  }

  const auto &config = config_it->second;
  if ( config.tracked_topic ) {
    RCLCPP_WARN( node_->get_logger(), "Service request rejected: e-stop '%s' is tracked and cannot be set via service.",
                 request->name.c_str() );
    response->result = e_stop_manager_msgs::srv::SetEStop_Response::FAILURE;
    return;
  }

  if ( state_it->second == request->value ) {
    RCLCPP_DEBUG( node_->get_logger(), "Requested state for managed e-stop '%s' unchanged.", request->name.c_str() );
    response->result = e_stop_manager_msgs::srv::SetEStop_Response::SUCCESS;
    return;
  }

  state_it->second = request->value;
  RCLCPP_INFO( node_->get_logger(), "Managed e-stop '%s' set to %s via service.", request->name.c_str(),
               request->value ? "activated" : "deactivated" );

  publishEStops();
  response->result = e_stop_manager_msgs::srv::SetEStop_Response::SUCCESS;
}

std::string EStopManager::sanitizeTopicName( const std::string &name )
{
  // due to the validator used the topic is already valid
  // here we only remove leasing '~' or '/' if present, since we use the topic name as part of another topic
  std::string sanitized_name = name;
  if ( name.front() == '~' ) {
    sanitized_name.erase( 0, 1 );
  }
  if ( sanitized_name.front() == '/' ) {
    sanitized_name.erase( 0, 1 );
  }
  return sanitized_name;
}
void EStopManager::publishEStops()
{
  // Update individual e-stop values in list
  for ( size_t i = 0; i < e_stop_list_msg_.names.size(); ++i ) {
    const auto &name = e_stop_list_msg_.names[i];
    const auto state_it = e_stop_state_.find( name );
    if ( state_it != e_stop_state_.end() ) {
      e_stop_list_msg_.values[i] = state_it->second;
    }
  }

  // Publish managed e-stop topics hosted by this node
  for ( const auto &[e_stop_name, pub] : managed_publishers_ ) {
    std_msgs::msg::Bool msg;
    msg.data = e_stop_state_[e_stop_name];
    pub->publish( msg );
  }

  // Compute aggregated states and publish them
  e_stop_list_msg_.aggregated_values.clear();
  for ( const auto &aggregated_topic : e_stop_list_msg_.aggregated_names ) {
    bool aggregated_state = false;
    auto members_it = aggregated_members_.find( aggregated_topic );
    if ( members_it != aggregated_members_.end() ) {
      aggregated_state = std::any_of( members_it->second.begin(), members_it->second.end(),
                                      [&]( const std::string &member ) { return e_stop_state_[member]; } );
    }
    e_stop_list_msg_.aggregated_values.push_back( aggregated_state );

    auto pub_it = aggregated_publishers_.find( aggregated_topic );
    if ( pub_it != aggregated_publishers_.end() ) {
      std_msgs::msg::Bool msg;
      msg.data = aggregated_state;
      pub_it->second->publish( msg );
    }
  }

  e_stop_list_pub_->publish( e_stop_list_msg_ );
}

} // namespace e_stop_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE( e_stop_manager::EStopManager );
