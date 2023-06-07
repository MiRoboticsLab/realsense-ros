// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#include "align_realsense_node.h"
#include <yaml-cpp/yaml.h>
#include <ros_utils.h>
#include <iomanip>

using namespace realsense2_camera;

void AlignRealSenseNode::getParameters()
{
    ROS_INFO("getParameters...");

    std::string param_name;
    param_name = std::string("camera_name");
    _camera_name = _parameters->setParam<std::string>(param_name, "camera");
    _parameters_names.push_back(param_name);

    param_name = std::string("publish_tf");
    _publish_tf = _parameters->setParam<bool>(param_name, PUBLISH_TF);
    _parameters_names.push_back(param_name);

    param_name = std::string("tf_publish_rate");
    _parameters->setParamT(param_name, _tf_publish_rate, [this](const rclcpp::Parameter& )
            {
                startDynamicTf();
            });
    _parameters_names.push_back(param_name);
    startDynamicTf();

    param_name = std::string("diagnostics_period");
    _diagnostics_period = _parameters->setParam<double>(param_name, DIAGNOSTICS_PERIOD);
    _parameters_names.push_back(param_name);

    param_name = std::string("enable_sync");
    _parameters->setParamT(param_name, _sync_frames);
    _parameters_names.push_back(param_name);

    param_name = std::string("json_file_path");
    _json_file_path = _parameters->setParam<std::string>(param_name, "");
    _parameters_names.push_back(param_name);

    param_name = std::string("clip_distance");
    _clipping_distance = _parameters->setParam<double>(param_name, -1.0);
    _parameters_names.push_back(param_name);

    param_name = std::string("linear_accel_cov");
    _linear_accel_cov = _parameters->setParam<double>(param_name, 0.01);
    _parameters_names.push_back(param_name);

    param_name = std::string("angular_velocity_cov");
    _angular_velocity_cov = _parameters->setParam<double>(param_name, 0.01);
    _parameters_names.push_back(param_name);
   
    param_name = std::string("hold_back_imu_for_frames");
    _hold_back_imu_for_frames = _parameters->setParam<bool>(param_name, HOLD_BACK_IMU_FOR_FRAMES);
    _parameters_names.push_back(param_name);

    param_name = std::string("publish_odom_tf");
    _publish_odom_tf = _parameters->setParam<bool>(param_name, PUBLISH_ODOM_TF);
    _parameters_names.push_back(param_name);

    param_name = std::string("base_frame_id");
    _base_frame_id = _parameters->setParam<std::string>(param_name, DEFAULT_BASE_FRAME_ID);
    _base_frame_id = (static_cast<std::ostringstream&&>(std::ostringstream() << _camera_name << "_" << _base_frame_id)).str();
    _parameters_names.push_back(param_name);

    param_name = std::string("align_config_file_path");
    _align_config_file_path = _parameters->setParam<std::string>(param_name, "");
    _parameters_names.push_back(param_name);
}

void AlignRealSenseNode::setDynamicParams()
{
    // Set default values:
    _imu_sync_method = imu_sync_method::NONE;

    auto imu_sync_method_string = [](imu_sync_method value) 
    { 
        switch (value)
        {
        case imu_sync_method::COPY:
            return "COPY";
        case imu_sync_method::LINEAR_INTERPOLATION:
            return "LINEAR_INTERPOLATION";
        default:
            return "NONE";
        }
    };

    // Register ROS parameter:
    std::string param_name("unite_imu_method");

    std::vector<std::pair<std::string, int> > enum_vec;
    size_t longest_desc(0);
    for (int i=0; i<=int(imu_sync_method::LINEAR_INTERPOLATION); i++)
    {
        std::string enum_str(imu_sync_method_string(imu_sync_method(i)));
        enum_vec.push_back(std::make_pair(enum_str, i));
        longest_desc = std::max(longest_desc, enum_str.size());
    }
    sort(enum_vec.begin(), enum_vec.end(), [](std::pair<std::string, int> e1, std::pair<std::string, int> e2){return (e1.second < e2.second);});
    std::stringstream enum_str_values;
    for (auto vec_iter : enum_vec)
    {
        enum_str_values << std::setw(longest_desc+6) << std::left << vec_iter.first << " : " << vec_iter.second << std::endl;
    }

    rcl_interfaces::msg::ParameterDescriptor crnt_descriptor;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = int(imu_sync_method::NONE);
    range.to_value = int(imu_sync_method::LINEAR_INTERPOLATION);
    crnt_descriptor.integer_range.push_back(range);
    std::stringstream desc;
    desc << "Available options are:" << std::endl << enum_str_values.str();
    crnt_descriptor.description = desc.str();
    _parameters->setParam<int>(param_name, int(imu_sync_method::NONE), 
                            [this](const rclcpp::Parameter& parameter)
                            {
                                _imu_sync_method = imu_sync_method(parameter.get_value<int>());
                            }, crnt_descriptor);
    _parameters_names.push_back(param_name);
}

void AlignRealSenseNode::clearParameters()
{
    while ( !_parameters_names.empty() )
    {
        auto name = _parameters_names.back();
        _parameters->removeParam(name);
        _parameters_names.pop_back();        
    }
}

void AlignRealSenseNode::readAlignParameters()
{
    std::string extrinsic_config_file = _align_config_file_path + "/params_extrinsic.yaml";
    YAML::Node extrinsic_config;
    try
    {
        extrinsic_config = YAML::LoadFile(extrinsic_config_file.c_str());
    }
    catch(const std::exception& e)
    {
        ROS_INFO_STREAM(" the extrinsics.yaml is not exist,using the default params");
        _other_intrinsics = {
            640, 480,
            320.8957528102377, 240.586864355793,
            289.5161106804787, 291.2975243243869,
            RS2_DISTORTION_BROWN_CONRADY,
            {
                0.023975130328665446, -0.042382138044280766, 0.0, 0.0033467922545996304,-0.0006137133656299577
            }
        };
        double data[16] = {
            0.9999590721286711, 0.002358948027165192, -0.008734382163175344, -0.019713832460504378,
            0.003871507437917554, 0.7609871081130068, 0.6487554490838293, 0.02153622077368339,
            0.0081771326102376, -0.6487627121297955, 0.7609468298457303, -0.02653976665265355,
            0.0, 0.0, 0.0, 1.0
            };
        cv::Mat T_other_depth(4,4,CV_64FC1,data);
        cv::Mat T_other_depth_inverse;
        cv::invert(T_other_depth,T_other_depth_inverse,cv::DECOMP_LU);

        _z_to_other = {
            {
                T_other_depth_inverse.at<double>(0,0),T_other_depth_inverse.at<double>(0,1),T_other_depth_inverse.at<double>(0,2),
                T_other_depth_inverse.at<double>(1,0),T_other_depth_inverse.at<double>(1,1),T_other_depth_inverse.at<double>(1,2),
                T_other_depth_inverse.at<double>(2,0),T_other_depth_inverse.at<double>(2,1),T_other_depth_inverse.at<double>(2,2)
            },
            {
                T_other_depth_inverse.at<double>(0,3),T_other_depth_inverse.at<double>(1,3),T_other_depth_inverse.at<double>(2,3)
            }
        };

        ROS_INFO("/********other camera infromation********/\n instrinsics: [fx,fx,cx,cy] = [%lf,%lf,%lf,%lf]\n resolution = %dx%d\n distortion_coeffs:[k1, k2, p1, p2, k3] = [%lf,%lf,%lf,%lf,%lf]\n " ,
        _other_intrinsics.fx,_other_intrinsics.fy,_other_intrinsics.ppx,_other_intrinsics.ppy,
        _other_intrinsics.width,_other_intrinsics.height,
        _other_intrinsics.coeffs[0],_other_intrinsics.coeffs[1],_other_intrinsics.coeffs[2],_other_intrinsics.coeffs[3],_other_intrinsics.coeffs[4]);
        return;    
    }

    YAML::Node T_other_depth_node = extrinsic_config["cam3"]["T_cn_c0"];
    YAML::Node intrinsics_node = extrinsic_config["cam3"]["intrinsics"];
    YAML::Node resolution_node = extrinsic_config["cam3"]["resolution"];
    YAML::Node distortion_coeffs_node = extrinsic_config["cam3"]["distortion_coeffs"];

    cv::Mat T_other_depth(4,4,CV_64FC1,cv::Scalar(0));
    for(size_t i=0;i<4;i++)
    {
        for (size_t j=0; j<4; j++)
        {
            T_other_depth.at<double>(i,j) = T_other_depth_node[i][j].as<double>();
        }
    }
    
    cv::Mat T_other_depth_inverse;
    cv::invert(T_other_depth,T_other_depth_inverse,cv::DECOMP_LU);

    int width = resolution_node[0].as<int>();
    int height = resolution_node[1].as<int>();

    double intrinsics[4];
    for(size_t i=0;i<4;i++)
        intrinsics[i] = intrinsics_node[i].as<double>();

    double distortion_coeffs[5];
    for(size_t i=0;i<5;i++)
        distortion_coeffs[i] = distortion_coeffs_node[i].as<double>();


    _other_intrinsics = {
        width, height,
        intrinsics[2],intrinsics[3],
        intrinsics[0],intrinsics[1],
        RS2_DISTORTION_BROWN_CONRADY,
        {distortion_coeffs[0],distortion_coeffs[1],distortion_coeffs[2],distortion_coeffs[3],distortion_coeffs[4]}
        };

    _z_to_other = {
        {
            T_other_depth_inverse.at<double>(0,0),T_other_depth_inverse.at<double>(0,1),T_other_depth_inverse.at<double>(0,2),
            T_other_depth_inverse.at<double>(1,0),T_other_depth_inverse.at<double>(1,1),T_other_depth_inverse.at<double>(1,2),
            T_other_depth_inverse.at<double>(2,0),T_other_depth_inverse.at<double>(2,1),T_other_depth_inverse.at<double>(2,2)
        },
        {
            T_other_depth_inverse.at<double>(0,3),T_other_depth_inverse.at<double>(1,3),T_other_depth_inverse.at<double>(2,3)
        }
    };

    ROS_INFO("/********other camera infromation********/\n instrinsics: [fx,fx,cx,cy] = [%lf,%lf,%lf,%lf]\n resolution = %dx%d\n distortion_coeffs:[k1, k2, p1, p2, k3] = [%lf,%lf,%lf,%lf,%lf]\n " ,
    _other_intrinsics.fx,_other_intrinsics.fy,_other_intrinsics.ppx,_other_intrinsics.ppy,
    _other_intrinsics.width,_other_intrinsics.height,
    _other_intrinsics.coeffs[0],_other_intrinsics.coeffs[1],_other_intrinsics.coeffs[2],_other_intrinsics.coeffs[3],_other_intrinsics.coeffs[4]);
}
