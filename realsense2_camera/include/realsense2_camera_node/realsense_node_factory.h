// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

// cpplint: c system headers
#include "constants.h"
#include "base_realsense_node.h"
#include <builtin_interfaces/msg/time.hpp>
#include <console_bridge/console.h>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include <algorithm>
#include <csignal>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>

#include "nav2_util/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace realsense2_camera
{
    class RealSenseNodeFactory : public nav2_util::LifecycleNode
    {
    public:
        explicit RealSenseNodeFactory(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
        RealSenseNodeFactory(
            const std::string & node_name, const std::string & ns,
            const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
        virtual ~RealSenseNodeFactory();

    private:
        void init();
        void closeDevice();
        void startDevice();
        void changeDeviceCallback(rs2::event_information& info);
        void getDevice(rs2::device_list list);
        void tryGetLogSeverity(rs2_log_severity& severity) const;
        static std::string parseUsbPort(std::string line);
        //节点初始化
        bool _init;
        //数据拉起成功
        bool _state;
        //是否数据激活
        bool _activate;
        std::string node_name;
        rclcpp::Node::SharedPtr _node;
        rs2::device _device;
        std::unique_ptr<BaseRealSenseNode> _realSenseNode;
        rs2::context _ctx;
        std::string _serial_no;
        std::string _usb_port_id;
        std::string _device_type;
        std::string _rosbag_filename;
        double _wait_for_device_timeout;
        double _reconnect_timeout;
        bool _initial_reset;
        std::thread _query_thread;
        bool _is_alive;
        rclcpp::Logger _logger;
        std::shared_ptr<Parameters> _parameters;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _client;
        bool _recovery = false;
    protected:
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
        // nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
    };

}//end namespace
