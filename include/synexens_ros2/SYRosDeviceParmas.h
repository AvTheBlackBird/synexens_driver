#ifndef SYRosDeviceParmas_H
#define SYRosDeviceParmas_H

#include "SYRosTypes.h"
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std;

// config params struct
struct SYCameraParams
{
    string tf_prefix = "";
    int fps = 30;
    bool point_cloud_enabled = true;

    // +++++++++++++++++++++++++++++++++++++++++++++++++ CS30 CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++
    bool CS30_depth_enabled = true;
    string CS30_depth_resolution = "480P";
    bool CS30_ir_enabled = true;
    bool CS30_color_enabled = true;
    string CS30_color_resolution = "540P";
    bool CS30_mapping_enabled = false;

    int CS30_exposure = 3000;
    int CS30_exposure_range_min = 0;
    int CS30_exposure_range_max = 3000;

    int CS30_depth_image_filter = -1;
    float CS30_filter_amplititud_value = -1.0f;
    float CS30_filter_median_value_01 = -1.0f;
    float CS30_filter_median_value_02 = -1.0f;
    float CS30_filter_gauss_value_01 = -1.0f;
    float CS30_filter_gauss_value_02 = -1.0f;
    float CS30_filter_edge_value = -1.0f;
    float CS30_filter_speckle_value_01 = -1.0f;
    float CS30_filter_speckle_value_02 = -1.0f;
    float CS30_filter_sobel_value = -1.0f;
    float CS30_filter_edge_mad_value = -1.0f;
    float CS30_filter_okada_value = -1.0f;

    // +++++++++++++++++++++++++++++++++++++++++++++++++ CS20 CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++
    bool CS20_depth_enabled = true;
    string CS20_depth_resolution = "240P";
    bool CS20_ir_enabled = true;

    int CS20_exposure = 3000;
    int CS20_exposure_range_min = 0;
    int CS20_exposure_range_max = 3000;

    int CS20_depth_image_filter = -1;
    float CS20_filter_amplititud_value = -1.0f;
    float CS20_filter_median_value_01 = -1.0f;
    float CS20_filter_median_value_02 = -1.0f;
    float CS20_filter_gauss_value_01 = -1.0f;
    float CS20_filter_gauss_value_02 = -1.0f;
    float CS20_filter_edge_value = -1.0f;
    float CS20_filter_speckle_value_01 = -1.0f;
    float CS20_filter_speckle_value_02 = -1.0f;
    float CS20_filter_sobel_value = -1.0f;
    float CS20_filter_edge_mad_value = -1.0f;
    float CS20_filter_okada_value = -1.0f;

    // +++++++++++++++++++++++++++++++++++++++++++++++++ CS20P CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++
    bool CS20P_depth_enabled = true;
    string CS20P_depth_resolution = "240P";
    bool CS20P_ir_enabled = true;

    int CS20P_exposure = 3000;
    int CS20P_exposure_range_min = 0;
    int CS20P_exposure_range_max = 3000;

    int CS20P_depth_image_filter = -1;
    float CS20P_filter_amplititud_value = -1.0f;
    float CS20P_filter_median_value_01 = -1.0f;
    float CS20P_filter_median_value_02 = -1.0f;
    float CS20P_filter_gauss_value_01 = -1.0f;
    float CS20P_filter_gauss_value_02 = -1.0f;
    float CS20P_filter_edge_value = -1.0f;
    float CS20P_filter_speckle_value_01 = -1.0f;
    float CS20P_filter_speckle_value_02 = -1.0f;
    float CS20P_filter_sobel_value = -1.0f;
    float CS20P_filter_edge_mad_value = -1.0f;
    float CS20P_filter_okada_value = -1.0f;

    // +++++++++++++++++++++++++++++++++++++++++++++++++ CS40 CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++
    bool CS40_depth_enabled = true;
    string CS40_depth_resolution = "480P";
    bool CS40_ir_enabled = true;

    int CS40_exposure = 3000;
    int CS40_exposure_range_min = 0;
    int CS40_exposure_range_max = 3000;

    int CS40_depth_image_filter = -1;
    float CS40_filter_amplititud_value = -1.0f;
    float CS40_filter_median_value_01 = -1.0f;
    float CS40_filter_median_value_02 = -1.0f;
    float CS40_filter_gauss_value_01 = -1.0f;
    float CS40_filter_gauss_value_02 = -1.0f;
    float CS40_filter_edge_value = -1.0f;
    float CS40_filter_speckle_value_01 = -1.0f;
    float CS40_filter_speckle_value_02 = -1.0f;
    float CS40_filter_sobel_value = -1.0f;
    float CS40_filter_edge_mad_value = -1.0f;
    float CS40_filter_okada_value = -1.0f;    
};

class SYRosDeviceParmas
{
public:
    // set params
    void SetParams(rclcpp::Node::SharedPtr node_);
    // get parmas
    SYCameraParams GetParams(); 
    // get camera config
    void GetCameraConfig(SYCameraConfig *cameraConfig);
private:
    SYCameraParams config_params_;
};

#endif