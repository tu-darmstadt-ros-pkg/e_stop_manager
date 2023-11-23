#include <rclcpp/rclcpp.hpp>
#include "e_stop_manager/e_stop_manager.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;
    auto e_stop_manager = std::make_shared<e_stop_manager::EStopManager>();
    exe.add_node(e_stop_manager->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();

    return 0;
}
