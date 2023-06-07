#include <memory>

#include "realsense_node_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    std::shared_ptr<realsense2_camera::RealSenseNodeFactory> realsese_node_factory = std::make_shared<realsense2_camera::RealSenseNodeFactory>(options);
    exec.add_node(realsese_node_factory->get_node_base_interface());
    
    exec.spin();

    rclcpp::shutdown();

    return 0;
}
