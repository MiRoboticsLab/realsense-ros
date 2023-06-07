#include <memory>

#include "actuator_node_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    LOGGER_MAIN_INSTANCE("RealSenseActuator");

    std::shared_ptr<realsense2_camera::ActuatorNodeFactory> actuator_node_factory = std::make_shared<realsense2_camera::ActuatorNodeFactory>("RealSenseActuator");
    
    std::thread t([&](){
        actuator_node_factory->Spin();
    });
    actuator_node_factory->init();
    t.join();
    return 0;
}
