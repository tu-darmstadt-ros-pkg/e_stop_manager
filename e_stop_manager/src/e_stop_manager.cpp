#include "e_stop_manager/e_stop_manager.h"


namespace e_stop_manager
{

    EStopManager::EStopManager(const rclcpp::NodeOptions & options):
            node_(std::make_shared<rclcpp::Node>("e_stop_manager", options))
    {
        node_->declare_parameter<std::vector<std::string>>("e_stop_names",std::vector<std::string>{});
        node_->declare_parameter<std::vector<std::string>>("e_stop_topics",std::vector<std::string>{});
        e_stop_list_pub_ = node_->create_publisher<e_stop_manager_msgs::msg::EStopList>("e_stop_list", 5);

        rclcpp::Parameter e_stop_topics_param;
        std::vector<std::string> e_stop_topics;
        std::vector<std::string> e_stop_names;
        // load e-stop topics and names
        if (node_->get_parameter("e_stop_topics", e_stop_topics_param) && e_stop_topics_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            e_stop_topics = e_stop_topics_param.as_string_array();
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("e_stop_manager"), "Parameter e_stop_topic not provided or is not a list! E-stops will only be published on e-stop-list topic!");
        }

        if (node_->get_parameter("e_stop_names", e_stop_topics_param) && e_stop_topics_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
            e_stop_names = e_stop_topics_param.as_string_array();
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("e_stop_manager"), "Parameter e_stop_topic not provided or is not a list! E-stops will only be published on e-stop-list topic!");
        }

        // declare parameters per e-stop
        for(const auto& e_stop_name: e_stop_names)
        {
            node_->declare_parameter<bool>(e_stop_name+"_start_value", false);
            node_->declare_parameter<std::string>(e_stop_name+"_topic", e_stop_topics.empty()?"missing_topics":e_stop_topics[0]);
        }
        std::map<std::string, std::vector<std::string>> e_stops_per_topic;
        std::map<std::string, bool> e_stops_start_values;
        for(const auto& e_stop_name:e_stop_names)
        {
            e_stops_start_values[e_stop_name] = node_->get_parameter(e_stop_name+"_start_value").as_bool();
            std::string topic = node_->get_parameter(e_stop_name+"_topic").as_string();
            if(e_stops_per_topic.find(topic) != e_stops_per_topic.end())
            {
                e_stops_per_topic[topic].push_back(e_stop_name);
            }
            else
            {
                e_stops_per_topic[topic] = {e_stop_name};
            }
        }

        // create publishers
        for (auto& entry : e_stop_topics)
        {
            std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> pub = node_->create_publisher<std_msgs::msg::Bool>(entry, 5);

            e_stop_pub_.emplace(pub,e_stops_per_topic[entry]);
        }

        // create messages
        for (auto& e_stop_name : e_stop_names)
        {
            e_stop_list_msg_.names.push_back(e_stop_name);
            e_stop_list_msg_.values.push_back(e_stops_start_values[e_stop_name]);
        }

        if (e_stop_list_msg_.names.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("e_stop_manager"), "No valid e_stop in list!");
            publishEStops(true);
            return;
        }

        for (auto& pub : e_stop_pub_)
        {
            pub.second.erase(std::remove_if(pub.second.begin(), pub.second.end(),
                                            [&](const std::string& name)
                                            {
                                                auto it = std::find(e_stop_list_msg_.names.begin(),
                                                                    e_stop_list_msg_.names.end(), name);
                                                if (it == e_stop_list_msg_.names.end())
                                                {
                                                    RCLCPP_WARN(rclcpp::get_logger("e_stop_manager"),
                                                                "The e_stop named \"%s\" was in e_stop_topics but not in e_stop_list. Ignored.", name.c_str());
                                                    return true;
                                                }
                                                return false;
                                            }),
                             pub.second.end());
        }

        set_e_stop_service_ = node_->create_service<e_stop_manager_msgs::srv::SetEStop>("set_e_stop", std::bind(&EStopManager::setEStopServiceCB, this, std::placeholders::_1, std::placeholders::_2));

        publishEStops();
        RCLCPP_INFO(rclcpp::get_logger("e_stop_manager"), "e_stop_manager initialized!");
    }

    bool EStopManager::setEStopServiceCB(
            const std::shared_ptr<e_stop_manager_msgs::srv::SetEStop::Request> request,
            std::shared_ptr<e_stop_manager_msgs::srv::SetEStop::Response> response)
    {

        RCLCPP_INFO(rclcpp::get_logger("e_stop_manager"), "EStop '%s' sent value: %d.", request->name.c_str(), request->value);

        bool name_found = false;

        for (size_t i = 0; i < e_stop_list_msg_.names.size(); ++i)
        {
            if (e_stop_list_msg_.names[i] == request->name)
            {
                // if value has not changed, do nothing and return
                if (e_stop_list_msg_.values[i] == request->value)
                {
                    RCLCPP_INFO(rclcpp::get_logger("e_stop_manager"), "Do nothing. Requested state already true");
                    response->result = response->SUCCESS;
                    return true;
                }

                e_stop_list_msg_.values[i] = request->value;
                name_found = true;
                break;
            }
        }

        if (!name_found)
        {

            RCLCPP_ERROR(rclcpp::get_logger("e_stop_manager"), "EStop : '%s' not found!", request->name.c_str());
            response->result = response->INVALID_ESTOP_NAME;
            return true;
        }

        publishEStops();

        RCLCPP_INFO(rclcpp::get_logger("e_stop_manager"), "EStop '%s' sent value: %d.", request->name.c_str(), request->value);

        response->result = response->SUCCESS;
        return true;
    }

    void EStopManager::publishEStops(bool force_e_stop)
    {
        e_stop_list_pub_->publish(e_stop_list_msg_);

        for (auto& pub : e_stop_pub_)
        {
            auto msg = std::make_unique<std_msgs::msg::Bool>();
            msg->data = force_e_stop ||
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

            pub.first->publish(std::move(msg));
        }
    }

}
