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

#include "cyberdog_machine/cyberdog_fs_machine.hpp"

enum class DemoCode : int32_t
{
  kDemoError1 = 21,
  kDemoError2 = 22,
  kDemoError3 = 23
};

namespace realsense2_camera
{
    class ActuatorNodeFactory : public cyberdog::machine::MachineActuator
    {
    public:
        explicit ActuatorNodeFactory(const std::string &node_name);
        void init();
        void Spin();
        virtual ~ActuatorNodeFactory();

    private:
        void getDevice(rs2::device_list list);
        static std::string parseUsbPort(std::string line);
        
        std::string _node_name;
        rclcpp::Node::SharedPtr _node;
        rs2::device _device;
        rs2::context _ctx;
        std::string _serial_no;
        std::string _usb_port_id;
        std::string _device_type;
        //Actuator
    public:
        std::string Uninitialized_V = std::string("Uninitialized");
        std::string SetUp_V = std::string("SetUp");
        std::string TearDown_V = std::string("TearDown");
        std::string SelfCheck_V = std::string("SelfCheck");
        std::string Active_V = std::string("Active");
        std::string DeActive_V = std::string("DeActive");
        std::string Protected_V = std::string("Protected");
        std::string LowPower_V = std::string("LowPower");
        std::string OTA_V = std::string("OTA");
        std::string Error_V = std::string("Error");
        std::shared_ptr<cyberdog::system::CyberdogCode<DemoCode>> code_ptr_ {nullptr};

        int32_t OnSetUp();
        int32_t ONTearDown();
        int32_t OnSelfCheck();
        int32_t OnActive();
        int32_t OnDeActive();
        int32_t OnProtected();
        int32_t OnLowPower();
        int32_t OnOTA();
        int32_t OnError();
    };

}//end namespace
