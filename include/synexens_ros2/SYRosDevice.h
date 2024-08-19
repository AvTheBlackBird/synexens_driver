#ifndef SYRosDevice_H
#define SYRosDevice_H

#include <iostream>
#include "SYSDKInterface.h"
#include "SYDataDefine.h"
#include "SYRosTypes.h"
#include "SYRosDeviceParmas.h"
#include "SYCalibrationTransformData.h"
#include "SYObserverInterface.h"

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cstring>
#include <thread>

using namespace Synexens;
using namespace sensor_msgs;
using namespace rclcpp;
using namespace msg;

enum PUBLISHER_TYPE
{
    // DEPTH
    DEPTH,
    // ir
    IR,
    // RGB
    RGB,
    // points
    POINTS
};

class SYRosDevice : public Synexens::ISYFrameObserver
{
private:
    /* data */
public:
    SYRosDevice(rclcpp::Node::SharedPtr nd);
    ~SYRosDevice();

    // start cameras
    SYErrorCode startCameras();
    // stop cameras
    void stopCameras();
    void PrintErrorCode(std::string strFunc, Synexens::SYErrorCode errorCode);
    std::map<unsigned int, Synexens::SYStreamType> m_mapStreamType;
    void ProcessFrameData(unsigned int nDeviceID, Synexens::SYFrameData *pFrameData = nullptr);

    // setoption
    void SetOption(unsigned int nDeviceID, SYDeviceType deviceType);

    // fill point cloud
    void FillPointCloud(int nWidth, int nHeight, SYPointCloudData *pPCLData, PointCloud2::SharedPtr &point_cloud);

    // frame publisher thread
    void framePublisherThread();

    // ros publisher map
    std::map<int, std::map<PUBLISHER_TYPE, rclcpp::Publisher<msg::CameraInfo>::SharedPtr>> m_mapRosPublisher;
    // image publisher map
    std::map<int, std::map<PUBLISHER_TYPE, image_transport::Publisher>> m_mapImagePublisher;
    // points pulisher map
    std::map<int, std::map<PUBLISHER_TYPE, rclcpp::Publisher<PointCloud2>::SharedPtr>> m_mapPointsPublisher;

    // ros node
    rclcpp::Node::SharedPtr m_node;
    // calibration data
    SYCalibrationTransformData m_calibrationData;

    // depth camera info
    msg::CameraInfo m_depthCameraInfo;
    // rgb camera info
    msg::CameraInfo m_rgbCameraInfo;
    // ir camera info
    msg::CameraInfo m_irCameraInfo;

    // camera config
    SYCameraConfig m_cameraConfig;
    // ros config params obj
    SYRosDeviceParmas m_parmas;
    // ros config params
    SYCameraParams m_cameraParams;

    // run status
    bool m_bRunStatus = false;
    // thread
    std::thread m_framePublisherThread;
    // device info
    Synexens::SYDeviceInfo *m_pDeviceInfo = nullptr;
    // device count
    int m_nCount = 0;
    // device open status
    bool *m_pOpen = nullptr;
    // setOpton status
    bool m_bSetOptionStatus = true;
    
public:
    void OnFrameNotify(unsigned int nDeviceID, Synexens::SYFrameData *pFrameData = nullptr);
};

#endif