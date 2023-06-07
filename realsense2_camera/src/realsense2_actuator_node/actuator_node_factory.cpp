// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved
#include "actuator_node_factory.h"
#include <iostream>
#include <map>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <thread>
#include <sys/time.h>
#include <regex>

#include <unistd.h>
using namespace realsense2_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

ActuatorNodeFactory::ActuatorNodeFactory(const std::string &node_name):
MachineActuator(node_name),_serial_no(""),_usb_port_id("")
{	
	_node_name = node_name;
    _node = rclcpp::Node::make_shared(_node_name);
    code_ptr_ = std::make_shared<cyberdog::system::CyberdogCode<DemoCode>>(
      cyberdog::system::ModuleCode::kCameraServer);
}

ActuatorNodeFactory::~ActuatorNodeFactory()
{
}

std::string ActuatorNodeFactory::parseUsbPort(std::string line)
{
    std::string port_id;
    std::regex self_regex("(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*", std::regex_constants::ECMAScript);
    std::smatch base_match;
    bool found = std::regex_match(line, base_match, self_regex);
    if (found)
    {
        port_id = base_match[1].str();
        if (base_match[2].str().size() == 0)    //This is libuvc string. Remove counter is exists.
        {
            std::regex end_regex = std::regex(".+(-[0-9]+$)", std::regex_constants::ECMAScript);
            bool found_end = std::regex_match(port_id, base_match, end_regex);
            if (found_end)
            {
                port_id = port_id.substr(0, port_id.size() - base_match[1].str().size());
            }
        }
    }
    return port_id;
}

void ActuatorNodeFactory::getDevice(rs2::device_list list)
{
	if (!_device)
	{
		if (0 == list.size())
		{
			INFO("No RealSense devices were found!");
		}
		else
		{
			bool found = false;
			rs2::device dev;
			for (size_t count = 0; count < list.size(); count++)
			{
				try
				{
					dev = list[count];
				}
				catch(const std::exception& ex)
				{
					continue;
				}
				auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
				std::string pn = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
				std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
				std::vector<std::string> results;
				std::string port_id = parseUsbPort(pn);
				if (port_id.empty())
				{
					std::stringstream msg;
					msg << "Error extracting usb port from device with physical ID: " << pn << std::endl << "Please report on github issue at https://github.com/IntelRealSense/realsense-ros";
					if (_usb_port_id.empty())
					{
					}
					else
					{
						INFO("Please use serial number instead of usb port.");
					}
				}
				else
				{

				}
				bool found_device_type(true);
				if (!_device_type.empty())
				{
					std::smatch match_results;
					std::regex device_type_regex(_device_type.c_str(), std::regex::icase);
					found_device_type = std::regex_search(name, match_results, device_type_regex);
				}

				if ((_serial_no.empty() || sn == _serial_no) && (_usb_port_id.empty() || port_id == _usb_port_id) && found_device_type)
				{
					_device = dev;
					_serial_no = sn;
					found = true;
					break;
				}
			}
			if (!found)
			{
				std::string msg ("The requested device with ");
				bool add_and(false);
				if (!_serial_no.empty())
				{
					msg += "serial number " + _serial_no;
					add_and = true;
				}
				if (!_usb_port_id.empty())
				{
					if (add_and)
					{
						msg += " and ";
					}
					msg += "usb port id " + _usb_port_id;
					add_and = true;
				}
				if (!_device_type.empty())
				{
					if (add_and)
					{
						msg += " and ";
					}
					msg += "device name containing " + _device_type;
				}
				msg += " is NOT found. Will Try again.";
			}
			else
			{
				if (_device.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
				{
					std::string usb_type = _device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
					if (usb_type.find("2.") != std::string::npos)
					{
						std::cout<<"Device " << _serial_no << " is connected using a " << usb_type << " port. Reduced performance is expected."<<std::endl;
					}
				}
			}
		}
	}
}


void ActuatorNodeFactory::init()
{
	//Actuator
	std::string actuator_config_file = "/opt/ros2/cyberdog/share/params/toml_config/manager/state_machine_config.toml";
	if (!this->MachineActuatorInit(actuator_config_file.c_str(),_node))
    {
      ERROR("Init failed, actuator init error.");
    }
    this->RegisterStateCallback(SetUp_V, std::bind(&ActuatorNodeFactory::OnSetUp, this));
    this->RegisterStateCallback(TearDown_V, std::bind(&ActuatorNodeFactory::ONTearDown, this));
    this->RegisterStateCallback(SelfCheck_V, std::bind(&ActuatorNodeFactory::OnSelfCheck, this));
    this->RegisterStateCallback(Active_V, std::bind(&ActuatorNodeFactory::OnActive, this));
    this->RegisterStateCallback(DeActive_V, std::bind(&ActuatorNodeFactory::OnDeActive, this));
    this->RegisterStateCallback(Protected_V, std::bind(&ActuatorNodeFactory::OnProtected, this));
    this->RegisterStateCallback(LowPower_V, std::bind(&ActuatorNodeFactory::OnLowPower, this));
    this->RegisterStateCallback(OTA_V, std::bind(&ActuatorNodeFactory::OnOTA, this));
    this->RegisterStateCallback(Error_V, std::bind(&ActuatorNodeFactory::OnError, this));
    this->ActuatorStart();
}

void ActuatorNodeFactory::Spin()
{
    INFO("ActuatorDemo: %s spin.", _node_name.c_str());
    rclcpp::spin(_node);	
}

int32_t ActuatorNodeFactory::OnSetUp()
{
	INFO("ActuatorDemo on setup.");
	return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}
int32_t ActuatorNodeFactory::ONTearDown()
{
	INFO("ActuatorDemo on teardown.");
	return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}
int32_t ActuatorNodeFactory::OnSelfCheck()
{
	INFO("ActuatorDemo on selfcheck.");
	getDevice(_ctx.query_devices());
	if(_device)
	{
		return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
	}
	else
	{
		INFO("RealSense was not found");
		return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kSelfCheckFailed);
	}
}
int32_t ActuatorNodeFactory::OnActive()
{
	INFO("ActuatorDemo on active.");
	return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}
int32_t ActuatorNodeFactory::OnDeActive()
{
	INFO("ActuatorDemo on deactive.");
	return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}
int32_t ActuatorNodeFactory::OnProtected()
{
	INFO("ActuatorDemo on protected.");
	return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}
int32_t ActuatorNodeFactory::OnLowPower()
{
	INFO("ActuatorDemo on lowpower.");
	return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}
int32_t ActuatorNodeFactory::OnOTA()
{
	INFO("ActuatorDemo on OTA.");
	return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}
int32_t ActuatorNodeFactory::OnError()
{
	INFO("ActuatorDemo on OTA.");
	return code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}
