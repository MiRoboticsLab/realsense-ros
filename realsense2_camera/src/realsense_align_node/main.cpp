#include <memory>

#include "align_node_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    auto align_node_factory = std::make_shared<realsense2_camera::AlignNodeFactory>(options);
    exec.add_node(align_node_factory->get_node_base_interface());

    exec.spin();

    rclcpp::shutdown();

    return 0;
}