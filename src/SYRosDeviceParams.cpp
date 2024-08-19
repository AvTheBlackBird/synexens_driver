#include "synexens_ros2/SYRosDeviceParmas.h"

void SYRosDeviceParmas::SetParams(rclcpp::Node::SharedPtr node_)
{
    // Prefix added to tf frame IDs. It typically contains a trailing '_' unless empty.
    node_->declare_parameter<string>("tf_prefix", "");
    node_->get_parameter("tf_prefix", config_params_.tf_prefix);
    // The FPS of the RGB and Depth cameras. Options are: 5, 7, 15, 30
    node_->declare_parameter<int>("fps", 30);
    node_->get_parameter("fps", config_params_.fps);
    // Generate a point cloud from depth data. Requires depth_enabled
    node_->declare_parameter<bool>("point_cloud_enabled", true);
    node_->get_parameter("point_cloud_enabled", config_params_.point_cloud_enabled);

    // +++++++++++++++++++++++++++++++++++++++++++++++++ CS30 CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++
    // Enable or disable the depth camera
    node_->declare_parameter<bool>("CS30_depth_enabled", true);
    node_->get_parameter("CS30_depth_enabled", config_params_.CS30_depth_enabled);
    // The resolution of the depth frame. Options are: 240P, 480P
    node_->declare_parameter<string>("CS30_depth_resolution", "480P");
    node_->get_parameter("CS30_depth_resolution", config_params_.CS30_depth_resolution);
    // Enable or disable the ir camera
    node_->declare_parameter<bool>("CS30_ir_enabled", true);
    node_->get_parameter("CS30_ir_enabled", config_params_.CS30_ir_enabled);
    // Enable or disable the color camera
    node_->declare_parameter<bool>("CS30_color_enabled", true);
    node_->get_parameter("CS30_color_enabled", config_params_.CS30_color_enabled);
    // Resolution at which to run the color camera. Valid options: 1080P 540P 480P
    node_->declare_parameter<string>("CS30_color_resolution", "540P");
    node_->get_parameter("CS30_color_resolution", config_params_.CS30_color_resolution);
    // True if mapped depth in color space should be enabled mapping depth resolution 480P rgb resolution 1080P 540P
    node_->declare_parameter<bool>("CS30_mapping_enabled", false);
    node_->get_parameter("CS30_mapping_enabled", config_params_.CS30_mapping_enabled);

    // The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=3000
    node_->declare_parameter<int>("CS30_exposure", 3000);
    node_->get_parameter("CS30_exposure", config_params_.CS30_exposure);
    // The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=0
    node_->declare_parameter<int>("CS30_exposure_range_min", 0);
    node_->get_parameter("CS30_exposure_range_min", config_params_.CS30_exposure_range_min);
    // The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=3000
    node_->declare_parameter<int>("CS30_exposure_range_max", 3000);
    node_->get_parameter("CS30_exposure_range_max", config_params_.CS30_exposure_range_max);

    // 0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1
    node_->declare_parameter<int>("CS30_depth_image_filter", -1);
    node_->get_parameter("CS30_depth_image_filter", config_params_.CS30_depth_image_filter);
    // AMPLITITUD value sett. Use default setting if value=-1. Min value 0 , Max value 100, Recommend value 6
    node_->declare_parameter<float>("CS30_filter_amplititud_value", -1.0f);
    node_->get_parameter("CS30_filter_amplititud_value", config_params_.CS30_filter_amplititud_value);
    // MEDIAN_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 3
    node_->declare_parameter<float>("CS30_filter_median_value_01", -1.0f);
    node_->get_parameter("CS30_filter_median_value_01", config_params_.CS30_filter_median_value_01);
    // MEDIAN_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
    node_->declare_parameter<float>("CS30_filter_median_value_02", -1.0f);
    node_->get_parameter("CS30_filter_median_value_02", config_params_.CS30_filter_median_value_02);
    // GAUSS_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 1
    node_->declare_parameter<float>("CS30_filter_gauss_value_01", -1.0f);
    node_->get_parameter("CS30_filter_gauss_value_01", config_params_.CS30_filter_gauss_value_01);
    // GAUSS_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
    node_->declare_parameter<float>("CS30_filter_gauss_value_02", -1.0f);
    node_->get_parameter("CS30_filter_gauss_value_02", config_params_.CS30_filter_gauss_value_02);
    // EDGE value sett. Use default setting if value=-1. Min value 20, Max value 200, Recommend value 50
    node_->declare_parameter<float>("CS30_filter_edge_value", -1.0f);
    node_->get_parameter("CS30_filter_edge_value", config_params_.CS30_filter_edge_value);
    // SPECKLE_01 value sett. Use default setting if value=-1. Min value 24, Max value 200, Recommend value 40
    node_->declare_parameter<float>("CS30_filter_speckle_value_01", -1.0f);
    node_->get_parameter("CS30_filter_speckle_value_01", config_params_.CS30_filter_speckle_value_01);
    // SPECKLE_02 value sett. Use default setting if value=-1. Min value 40, Max value 200
    node_->declare_parameter<float>("CS30_filter_speckle_value_02", -1.0f);
    node_->get_parameter("CS30_filter_speckle_value_02", config_params_.CS30_filter_speckle_value_02);
    // SOBEL value sett. Use default setting if value=-1. Min value 20, Max value 300, Recommend value 150
    node_->declare_parameter<float>("CS30_filter_sobel_value", -1.0f);
    node_->get_parameter("CS30_filter_sobel_value", config_params_.CS30_filter_sobel_value);
    // EDGE_MAD value sett. Use default setting if value=-1. Min value 5, Max value 100, Recommend value 15
    node_->declare_parameter<float>("CS30_filter_edge_mad_value", -1.0f);
    node_->get_parameter("CS30_filter_edge_mad_value", config_params_.CS30_filter_edge_mad_value);
    // OKADA value sett. Use default setting if value=-1. Min value 10, Max value 100, Recommend value 10
    node_->declare_parameter<float>("CS30_filter_okada_value", -1.0f);
    node_->get_parameter("CS30_filter_okada_value", config_params_.CS30_filter_okada_value);

    // +++++++++++++++++++++++++++++++++++++++++++++++++ CS20 CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++
    // Enable or disable the depth camera
    node_->declare_parameter<bool>("CS20_depth_enabled", true);
    node_->get_parameter("CS20_depth_enabled", config_params_.CS20_depth_enabled);
    // The resolution of the depth frame. Options are: 240P, 480P. SINGLE not supported 480P
    node_->declare_parameter<string>("CS20_depth_resolution", "240p");
    node_->get_parameter("CS20_depth_resolution", config_params_.CS20_depth_resolution);
    // Enable or disable the ir camera
    node_->declare_parameter<bool>("CS20_ir_enabled", true);
    node_->get_parameter("CS20_ir_enabled", config_params_.CS20_ir_enabled);

    // The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=3000
    node_->declare_parameter<int>("CS20_exposure", 3000);
    node_->get_parameter("CS20_exposure", config_params_.CS20_exposure);
    // The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=0
    node_->declare_parameter<int>("CS20_exposure_range_min", 0);
    node_->get_parameter("CS20_exposure_range_min", config_params_.CS20_exposure_range_min);
    // The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=3000
    node_->declare_parameter<int>("CS20_exposure_range_max", 3000);
    node_->get_parameter("CS20_exposure_range_max", config_params_.CS20_exposure_range_max);

    // 0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1
    node_->declare_parameter<int>("CS20_depth_image_filter", -1);
    node_->get_parameter("CS20_depth_image_filter", config_params_.CS20_depth_image_filter);
    // AMPLITITUD value sett. Use default setting if value=-1. Min value 0 , Max value 100, Recommend value 6
    node_->declare_parameter<float>("CS20_filter_amplititud_value", -1.0f);
    node_->get_parameter("CS20_filter_amplititud_value", config_params_.CS20_filter_amplititud_value);
    // MEDIAN_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 3
    node_->declare_parameter<float>("CS20_filter_median_value_01", -1.0f);
    node_->get_parameter("CS20_filter_median_value_01", config_params_.CS20_filter_median_value_01);
    // MEDIAN_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
    node_->declare_parameter<float>("CS20_filter_median_value_02", -1.0f);
    node_->get_parameter("CS20_filter_median_value_02", config_params_.CS20_filter_median_value_02);
    // GAUSS_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 1
    node_->declare_parameter<float>("CS20_filter_gauss_value_01", -1.0f);
    node_->get_parameter("CS20_filter_gauss_value_01", config_params_.CS20_filter_gauss_value_01);
    // GAUSS_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
    node_->declare_parameter<float>("CS20_filter_gauss_value_02", -1.0f);
    node_->get_parameter("CS20_filter_gauss_value_02", config_params_.CS20_filter_gauss_value_02);
    // EDGE value sett. Use default setting if value=-1. Min value 20, Max value 200, Recommend value 50
    node_->declare_parameter<float>("CS20_filter_edge_value", -1.0f);
    node_->get_parameter("CS20_filter_edge_value", config_params_.CS20_filter_edge_value);
    // SPECKLE_01 value sett. Use default setting if value=-1. Min value 24, Max value 200, Recommend value 40
    node_->declare_parameter<float>("CS20_filter_speckle_value_01", -1.0f);
    node_->get_parameter("CS20_filter_speckle_value_01", config_params_.CS20_filter_speckle_value_01);
    // SPECKLE_02 value sett. Use default setting if value=-1. Min value 40, Max value 200
    node_->declare_parameter<float>("CS20_filter_speckle_value_02", -1.0f);
    node_->get_parameter("CS20_filter_speckle_value_02", config_params_.CS20_filter_speckle_value_02);
    // SOBEL value sett. Use default setting if value=-1. Min value 20, Max value 300, Recommend value 150
    node_->declare_parameter<float>("CS20_filter_sobel_value", -1.0f);
    node_->get_parameter("CS20_filter_sobel_value", config_params_.CS20_filter_sobel_value);
    // EDGE_MAD value sett. Use default setting if value=-1. Min value 5, Max value 100, Recommend value 15
    node_->declare_parameter<float>("CS20_filter_edge_mad_value", -1.0f);
    node_->get_parameter("CS20_filter_edge_mad_value", config_params_.CS20_filter_edge_mad_value);
    // OKADA value sett. Use default setting if value=-1. Min value 10, Max value 100, Recommend value 10
    node_->declare_parameter<float>("CS20_filter_okada_value", -1.0f);
    node_->get_parameter("CS20_filter_okada_value", config_params_.CS20_filter_okada_value);

    // +++++++++++++++++++++++++++++++++++++++++++++++++ CS20P CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++
    // Enable or disable the depth camera
    node_->declare_parameter<bool>("CS20P_depth_enabled", true);
    node_->get_parameter("CS20P_depth_enabled", config_params_.CS20P_depth_enabled);
    // The resolution of the depth frame. Options are: 240P.
    node_->declare_parameter<string>("CS20P_depth_resolution", "240P");
    node_->get_parameter("CS20P_depth_resolution", config_params_.CS20P_depth_resolution);
    // Enable or disable the ir camera
    node_->declare_parameter<bool>("CS20P_ir_enabled", true);
    node_->get_parameter("CS20P_ir_enabled", config_params_.CS20P_ir_enabled);

    // The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=3000
    node_->declare_parameter<int>("CS20P_exposure", 3000);
    node_->get_parameter("CS20P_exposure", config_params_.CS20P_exposure);
    // The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=0
    node_->declare_parameter<int>("CS20P_exposure_range_min", 0);
    node_->get_parameter("CS20P_exposure_range_min", config_params_.CS20P_exposure_range_min);
    // The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=3000
    node_->declare_parameter<int>("CS20P_exposure_range_max", 3000);
    node_->get_parameter("CS20P_exposure_range_max", config_params_.CS20P_exposure_range_max);

    // 0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1
    node_->declare_parameter<int>("CS20P_depth_image_filter", -1);
    node_->get_parameter("CS20P_depth_image_filter", config_params_.CS20P_depth_image_filter);
    // AMPLITITUD value sett. Use default setting if value=-1. Min value 0 , Max value 100, Recommend value 6
    node_->declare_parameter<float>("CS20P_filter_amplititud_value", -1.0f);
    node_->get_parameter("CS20P_filter_amplititud_value", config_params_.CS20P_filter_amplititud_value);
    // MEDIAN_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 3
    node_->declare_parameter<float>("CS20P_filter_median_value_01", -1.0f);
    node_->get_parameter("CS20P_filter_median_value_01", config_params_.CS20P_filter_median_value_01);
    // MEDIAN_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
    node_->declare_parameter<float>("CS20P_filter_median_value_02", -1.0f);
    node_->get_parameter("CS20P_filter_median_value_02", config_params_.CS20P_filter_median_value_02);
    // GAUSS_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 1
    node_->declare_parameter<float>("CS20P_filter_gauss_value_01", -1.0f);
    node_->get_parameter("CS20P_filter_gauss_value_01", config_params_.CS20P_filter_gauss_value_01);
    // GAUSS_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
    node_->declare_parameter<float>("CS20P_filter_gauss_value_02", -1.0f);
    node_->get_parameter("CS20P_filter_gauss_value_02", config_params_.CS20P_filter_gauss_value_02);
    // EDGE value sett. Use default setting if value=-1. Min value 20, Max value 200, Recommend value 50
    node_->declare_parameter<float>("CS20P_filter_edge_value", -1.0f);
    node_->get_parameter("CS20P_filter_edge_value", config_params_.CS20P_filter_edge_value);
    // SPECKLE_01 value sett. Use default setting if value=-1. Min value 24, Max value 200, Recommend value 40
    node_->declare_parameter<float>("CS20P_filter_speckle_value_01", -1.0f);
    node_->get_parameter("CS20P_filter_speckle_value_01", config_params_.CS20P_filter_speckle_value_01);
    // SPECKLE_02 value sett. Use default setting if value=-1. Min value 40, Max value 200
    node_->declare_parameter<float>("CS20P_filter_speckle_value_02", -1.0f);
    node_->get_parameter("CS20P_filter_speckle_value_02", config_params_.CS20P_filter_speckle_value_02);
    // SOBEL value sett. Use default setting if value=-1. Min value 20, Max value 300, Recommend value 150
    node_->declare_parameter<float>("CS20P_filter_sobel_value", -1.0f);
    node_->get_parameter("CS20P_filter_sobel_value", config_params_.CS20P_filter_sobel_value);
    // EDGE_MAD value sett. Use default setting if value=-1. Min value 5, Max value 100, Recommend value 15
    node_->declare_parameter<float>("CS20P_filter_edge_mad_value", -1.0f);
    node_->get_parameter("CS20P_filter_edge_mad_value", config_params_.CS20P_filter_edge_mad_value);
    // OKADA value sett. Use default setting if value=-1. Min value 10, Max value 100, Recommend value 10
    node_->declare_parameter<float>("CS20P_filter_okada_value", -1.0f);
    node_->get_parameter("CS20P_filter_okada_value", config_params_.CS20P_filter_okada_value);

    // +++++++++++++++++++++++++++++++++++++++++++++++++ CS40 CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++
    // Enable or disable the depth camera
    node_->declare_parameter<bool>("CS40_depth_enabled", true);
    node_->get_parameter("CS40_depth_enabled", config_params_.CS40_depth_enabled);
    // The resolution of the depth frame. Options are: 480P.
    node_->declare_parameter<string>("CS40_depth_resolution", "480P");
    node_->get_parameter("CS40_depth_resolution", config_params_.CS40_depth_resolution);
    // Enable or disable the ir camera
    node_->declare_parameter<bool>("CS40_ir_enabled", true);
    node_->get_parameter("CS40_ir_enabled", config_params_.CS40_ir_enabled);

    // The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=3000
    node_->declare_parameter<int>("CS40_exposure", 1510);
    node_->get_parameter("CS40_exposure", config_params_.CS40_exposure);
    // The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=0
    node_->declare_parameter<int>("CS40_exposure_range_min", 10);
    node_->get_parameter("CS40_exposure_range_min", config_params_.CS40_exposure_range_min);
    // The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=3000
    node_->declare_parameter<int>("CS40_exposure_range_max", 3000);
    node_->get_parameter("CS40_exposure_range_max", config_params_.CS40_exposure_range_max);

    // 0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1
    node_->declare_parameter<int>("CS40_depth_image_filter", -1);
    node_->get_parameter("CS40_depth_image_filter", config_params_.CS40_depth_image_filter);
    // AMPLITITUD value sett. Use default setting if value=-1. Min value 0 , Max value 100, Recommend value 6
    node_->declare_parameter<float>("CS40_filter_amplititud_value", -1.0f);
    node_->get_parameter("CS40_filter_amplititud_value", config_params_.CS40_filter_amplititud_value);
    // MEDIAN_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 3
    node_->declare_parameter<float>("CS40_filter_median_value_01", -1.0f);
    node_->get_parameter("CS40_filter_median_value_01", config_params_.CS40_filter_median_value_01);
    // MEDIAN_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
    node_->declare_parameter<float>("CS40_filter_median_value_02", -1.0f);
    node_->get_parameter("CS40_filter_median_value_02", config_params_.CS40_filter_median_value_02);
    // GAUSS_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 1
    node_->declare_parameter<float>("CS40_filter_gauss_value_01", -1.0f);
    node_->get_parameter("CS40_filter_gauss_value_01", config_params_.CS40_filter_gauss_value_01);
    // GAUSS_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
    node_->declare_parameter<float>("CS40_filter_gauss_value_02", -1.0f);
    node_->get_parameter("CS40_filter_gauss_value_02", config_params_.CS40_filter_gauss_value_02);
    // EDGE value sett. Use default setting if value=-1. Min value 20, Max value 200, Recommend value 50
    node_->declare_parameter<float>("CS40_filter_edge_value", -1.0f);
    node_->get_parameter("CS40_filter_edge_value", config_params_.CS40_filter_edge_value);
    // SPECKLE_01 value sett. Use default setting if value=-1. Min value 24, Max value 200, Recommend value 40
    node_->declare_parameter<float>("CS40_filter_speckle_value_01", -1.0f);
    node_->get_parameter("CS40_filter_speckle_value_01", config_params_.CS40_filter_speckle_value_01);
    // SPECKLE_02 value sett. Use default setting if value=-1. Min value 40, Max value 200
    node_->declare_parameter<float>("CS40_filter_speckle_value_02", -1.0f);
    node_->get_parameter("CS40_filter_speckle_value_02", config_params_.CS40_filter_speckle_value_02);
    // SOBEL value sett. Use default setting if value=-1. Min value 20, Max value 300, Recommend value 150
    node_->declare_parameter<float>("CS40_filter_sobel_value", -1.0f);
    node_->get_parameter("CS40_filter_sobel_value", config_params_.CS40_filter_sobel_value);
    // EDGE_MAD value sett. Use default setting if value=-1. Min value 5, Max value 100, Recommend value 15
    node_->declare_parameter<float>("CS40_filter_edge_mad_value", -1.0f);
    node_->get_parameter("CS40_filter_edge_mad_value", config_params_.CS40_filter_edge_mad_value);
    // OKADA value sett. Use default setting if value=-1. Min value 10, Max value 100, Recommend value 10
    node_->declare_parameter<float>("CS40_filter_okada_value", -1.0f);
    node_->get_parameter("CS40_filter_okada_value", config_params_.CS40_filter_okada_value);
}

SYCameraParams SYRosDeviceParmas::GetParams()
{
    return config_params_;
}

void SYRosDeviceParmas::GetCameraConfig(SYCameraConfig *cameraConfig)
{
    // CS30 RGB
    if (config_params_.CS30_color_resolution == "1080P")
    {
        cameraConfig->CS30RGBResolution = Synexens::SYRESOLUTION_1920_1080;
    }
    else if (config_params_.CS30_color_resolution == "540P")
    {
        cameraConfig->CS30RGBResolution = Synexens::SYRESOLUTION_960_540;
    }
    else
    {
        cameraConfig->CS30RGBResolution = Synexens::SYRESOLUTION_1920_1080;
    }
    // CS30 Depth
    if (config_params_.CS30_depth_resolution == "240P")
    {
        cameraConfig->CS30DepthResolution = Synexens::SYRESOLUTION_320_240;
    }
    else if (config_params_.CS30_depth_resolution == "480P")
    {
        cameraConfig->CS30DepthResolution = Synexens::SYRESOLUTION_640_480;
    }
    else
    {
        cameraConfig->CS30DepthResolution = Synexens::SYRESOLUTION_320_240;
    }

    // CS30 stream type
    if (config_params_.CS30_color_enabled && config_params_.CS30_depth_enabled && config_params_.CS30_ir_enabled)
    {
        cameraConfig->CS30StreamType = Synexens::SYSTREAMTYPE_DEPTHIRRGB;
    }
    else if (config_params_.CS30_depth_enabled && config_params_.CS30_ir_enabled)
    {
        cameraConfig->CS30StreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
    }
    else if (config_params_.CS30_depth_enabled && config_params_.CS30_color_enabled)
    {
        cameraConfig->CS30StreamType = Synexens::SYSTREAMTYPE_DEPTHRGB;
    }
    else if (config_params_.CS30_depth_enabled)
    {
        cameraConfig->CS30StreamType = Synexens::SYSTREAMTYPE_DEPTH;
    }

    if (config_params_.CS30_mapping_enabled)
    {
        cameraConfig->CS30StreamType = Synexens::SYSTREAMTYPE_RGBD;
    }

    // CS20 depth
    if (config_params_.CS20_depth_resolution == "240P")
    {
        cameraConfig->CS20DepthResolution = Synexens::SYRESOLUTION_320_240;
    }
    else if (config_params_.CS20_depth_resolution == "480P")
    {
        cameraConfig->CS20DepthResolution = Synexens::SYRESOLUTION_640_480;
    }
    else
    {
        cameraConfig->CS20DepthResolution = Synexens::SYRESOLUTION_320_240;
    }

    // CS20 stream type
    if (config_params_.CS20_depth_enabled && config_params_.CS20_ir_enabled)
    {
        cameraConfig->CS20StreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
    }
    else if (config_params_.CS20_depth_enabled)
    {
        cameraConfig->CS20StreamType = Synexens::SYSTREAMTYPE_DEPTH;
    }
    else
    {
        cameraConfig->CS20StreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
    }

    // CS20P depth
    if (config_params_.CS20P_depth_resolution == "240P")
    {
        cameraConfig->CS20PDepthResolution = Synexens::SYRESOLUTION_320_240;
    }
    else
    {
        cameraConfig->CS20PDepthResolution = Synexens::SYRESOLUTION_320_240;
    }

    // CS20P stream type
    if (config_params_.CS20P_depth_enabled && config_params_.CS20P_ir_enabled)
    {
        cameraConfig->CS20PStreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
    }
    else if (config_params_.CS20P_depth_enabled)
    {
        cameraConfig->CS20PStreamType = Synexens::SYSTREAMTYPE_DEPTH;
    }
    else
    {
        cameraConfig->CS20PStreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
    }

    // CS40 depth
    if (config_params_.CS40_depth_resolution == "480P")
    {
        cameraConfig->CS40DepthResolution = Synexens::SYRESOLUTION_640_480;
    }
    else
    {
        cameraConfig->CS40DepthResolution = Synexens::SYRESOLUTION_640_480;
    }

    // CS40 stream type
    if (config_params_.CS40_depth_enabled && config_params_.CS40_ir_enabled)
    {
        cameraConfig->CS40StreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
    }
    else if (config_params_.CS40_depth_enabled)
    {
        cameraConfig->CS40StreamType = Synexens::SYSTREAMTYPE_DEPTH;
    }
    else
    {
        cameraConfig->CS40StreamType = Synexens::SYSTREAMTYPE_DEPTHIR;
    }
}
