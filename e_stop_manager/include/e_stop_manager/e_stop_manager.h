#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include <e_stop_manager_msgs/srv/set_e_stop.hpp>
#include <e_stop_manager_msgs/msg/e_stop_list.hpp>
#include "std_msgs/msg/bool.hpp"

namespace e_stop_manager
{

    class EStopManager
    {
    public:
        explicit EStopManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        // required if not subclassing Node, see https://github.com/ros2/demos/blob/humble/composition/src/node_like_listener_component.cpp
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
            return this->node_->get_node_base_interface();
        }
    private:
        void publishEStops( bool force_e_stop = false );

        bool setEStopServiceCB(const std::shared_ptr<e_stop_manager_msgs::srv::SetEStop::Request> request,
                                std::shared_ptr<e_stop_manager_msgs::srv::SetEStop::Response> response );



        rclcpp::Service<e_stop_manager_msgs::srv::SetEStop>::SharedPtr set_e_stop_service_;

        e_stop_manager_msgs::msg::EStopList e_stop_list_msg_;
        rclcpp::Publisher<e_stop_manager_msgs::msg::EStopList>::SharedPtr e_stop_list_pub_;
        std::map<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>, std::vector<std::string> > e_stop_pub_;
        rclcpp::Node::SharedPtr node_;
    };
}