#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <e_stop_manager/e_stop_manager_parameters.hpp>
#include <e_stop_manager_msgs/msg/e_stop_list.hpp>
#include <e_stop_manager_msgs/srv/set_e_stop.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace e_stop_manager
{

class EStopManager
{
public:
  explicit EStopManager( const rclcpp::NodeOptions &options = rclcpp::NodeOptions() );

  // required if not subclassing Node, see https://github.com/ros2/demos/blob/humble/composition/src/node_like_listener_component.cpp
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const { return this->node_->get_node_base_interface(); }

private:
  static std::string sanitizeTopicName( const std::string &name, bool &changed );
  void publishEStops();

  void handleTrackedUpdate( const std::string &name, bool value );

  void setEStopServiceCB( std::shared_ptr<e_stop_manager_msgs::srv::SetEStop::Request> request,
                          std::shared_ptr<e_stop_manager_msgs::srv::SetEStop::Response> response );

  rclcpp::Service<e_stop_manager_msgs::srv::SetEStop>::SharedPtr set_e_stop_service_;

  e_stop_manager_msgs::msg::EStopList e_stop_list_msg_;
  rclcpp::Publisher<e_stop_manager_msgs::msg::EStopList>::SharedPtr e_stop_list_pub_;
  std::map<std::string, std::vector<std::string>> aggregated_members_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> aggregated_publishers_;
  std::map<std::string, std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>>> tracked_subscriptions_;
  std::map<std::string, std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>> managed_publishers_;
  std::map<std::string, bool> e_stop_state_;
  e_stop_manager_config::Params params_;
  std::unique_ptr<e_stop_manager_config::ParamListener> param_listener_;
  rclcpp::Node::SharedPtr node_;
};
} // namespace e_stop_manager
