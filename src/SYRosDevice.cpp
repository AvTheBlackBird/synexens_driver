#include "synexens_ros2/SYRosDevice.h"

SYRosDevice::SYRosDevice(rclcpp::Node::SharedPtr nd) : m_node(nd)
{
    // set params
    m_parmas.SetParams(m_node);
    // get params
    m_cameraParams = m_parmas.GetParams();

    // get coamer config
    m_parmas.GetCameraConfig(&m_cameraConfig);
}

SYRosDevice::~SYRosDevice()
{
    stopCameras();
}

void SYRosDevice::stopCameras()
{
    Synexens::UnInitSDK();
}

void SYRosDevice::OnFrameNotify(unsigned int nDeviceID, Synexens::SYFrameData *pFrameData)
{
    ProcessFrameData(nDeviceID, pFrameData);
}

void SYRosDevice::PrintErrorCode(std::string strFunc, Synexens::SYErrorCode errorCode)
{
    RCLCPP_ERROR(m_node->get_logger(), "%s errorcode:%d", strFunc.c_str(), errorCode);
}

SYErrorCode SYRosDevice::startCameras()
{
    // init m_imageTranspoort
    image_transport::ImageTransport m_imageTranspoort(m_node);
    // ============= SDK init ============= //
    int nSDKVersionLength = 0;
    Synexens::SYErrorCode errorCodeGetSDKVersion = Synexens::GetSDKVersion(nSDKVersionLength, nullptr);
    if (errorCodeGetSDKVersion == Synexens::SYERRORCODE_SUCCESS)
    {
        if (nSDKVersionLength > 0)
        {
            char *pStringSDKVersion = new char[nSDKVersionLength];
            errorCodeGetSDKVersion = Synexens::GetSDKVersion(nSDKVersionLength, pStringSDKVersion);
            if (errorCodeGetSDKVersion == Synexens::SYERRORCODE_SUCCESS)
            {
                RCLCPP_INFO(m_node->get_logger(), "SDKVersion:%s", pStringSDKVersion);
            }
            else
            {
                PrintErrorCode("GetSDKVersion2", errorCodeGetSDKVersion);
                return errorCodeGetSDKVersion;
            }
            delete[] pStringSDKVersion;
        }
    }
    else
    {
        PrintErrorCode("GetSDKVersion", errorCodeGetSDKVersion);
    }

    Synexens::SYErrorCode errorCodeInitSDK = Synexens::InitSDK();
    if (errorCodeInitSDK != Synexens::SYERRORCODE_SUCCESS)
    {
        PrintErrorCode("InitSDK", errorCodeInitSDK);
        return errorCodeInitSDK;
    }

    m_calibrationData.initialize(m_cameraParams, m_node);

    int nCount = 0;
    Synexens::SYErrorCode errorCode = Synexens::FindDevice(nCount);

    if (errorCode == Synexens::SYERRORCODE_SUCCESS && nCount > 0)
    {
        Synexens::SYDeviceInfo *pDeviceInfo = new Synexens::SYDeviceInfo[nCount];
        errorCode = Synexens::FindDevice(nCount, pDeviceInfo);

        if (errorCode == Synexens::SYERRORCODE_SUCCESS)
        {
            bool *pOpen = new bool[nCount];
            memset(pOpen, 0, sizeof(bool) * nCount);
            for (int i = 0; i < nCount; i++)
            {
                // ============= register topic ============= //
                std::map<PUBLISHER_TYPE, rclcpp::Publisher<CameraInfo>::SharedPtr> mapRosPublisher;
                std::map<PUBLISHER_TYPE, image_transport::Publisher> mapImagePublisher;
                std::map<PUBLISHER_TYPE, rclcpp::Publisher<PointCloud2>::SharedPtr> mapPointsPublisher;

                // ros pulisher
                // depth
                std::string sTopicTopName = "camera" + std::to_string(pDeviceInfo[i].m_nDeviceID);
                std::string sTopicTempName = sTopicTopName + "/depth_info";
                mapRosPublisher.insert(std::pair<PUBLISHER_TYPE, rclcpp::Publisher<CameraInfo>::SharedPtr>(DEPTH, m_node->create_publisher<CameraInfo>(sTopicTempName.c_str(), 1)));
                // ir
                sTopicTempName = sTopicTopName + "/ir_info";
                mapRosPublisher.insert(std::pair<PUBLISHER_TYPE, rclcpp::Publisher<CameraInfo>::SharedPtr>(IR, m_node->create_publisher<CameraInfo>(sTopicTempName.c_str(), 1)));

                // image pulisher
                // depth
                sTopicTempName = sTopicTopName + "/depth_raw";
                mapImagePublisher.insert(std::pair<PUBLISHER_TYPE, image_transport::Publisher>(DEPTH, m_imageTranspoort.advertise(sTopicTempName.c_str(), 1)));
                // ir
                sTopicTempName = sTopicTopName + "/ir_raw";
                mapImagePublisher.insert(std::pair<PUBLISHER_TYPE, image_transport::Publisher>(IR, m_imageTranspoort.advertise(sTopicTempName.c_str(), 1)));

                // rgb
                if ((pDeviceInfo[i].m_deviceType == Synexens::SYDEVICETYPE_CS30_DUAL || pDeviceInfo[i].m_deviceType == Synexens::SYDEVICETYPE_CS30_SINGLE) && m_cameraParams.CS30_color_enabled)
                {
                    sTopicTempName = sTopicTopName + "/rgb_raw";
                    mapImagePublisher.insert(std::pair<PUBLISHER_TYPE, image_transport::Publisher>(RGB, m_imageTranspoort.advertise(sTopicTempName.c_str(), 1)));
                }

                if (m_cameraParams.point_cloud_enabled)
                {
                    sTopicTempName = sTopicTopName + "/points2";
                    mapPointsPublisher.insert(std::pair<PUBLISHER_TYPE, rclcpp::Publisher<PointCloud2>::SharedPtr>(POINTS, m_node->create_publisher<PointCloud2>(sTopicTempName.c_str(), 1)));

                    m_mapPointsPublisher.insert(std::pair<int, std::map<PUBLISHER_TYPE, rclcpp::Publisher<PointCloud2>::SharedPtr>>(pDeviceInfo[i].m_nDeviceID, mapPointsPublisher));
                }

                m_mapRosPublisher.insert(std::pair<int, std::map<PUBLISHER_TYPE, rclcpp::Publisher<CameraInfo>::SharedPtr>>(pDeviceInfo[i].m_nDeviceID, mapRosPublisher));
                m_mapImagePublisher.insert(std::pair<int, std::map<PUBLISHER_TYPE, image_transport::Publisher>>(pDeviceInfo[i].m_nDeviceID, mapImagePublisher));
                // ============= register topic ============= //

                Synexens::SYErrorCode errorCodeOpenDevice = Synexens::OpenDevice(pDeviceInfo[i]);
                if (errorCodeOpenDevice == Synexens::SYERRORCODE_SUCCESS)
                {
                    // get sn
                    int nStringLength = 0;
                    Synexens::SYErrorCode errorCodeGetSN = Synexens::GetDeviceSN(pDeviceInfo[i].m_nDeviceID, nStringLength, nullptr);
                    if (errorCodeGetSN == Synexens::SYERRORCODE_SUCCESS)
                    {
                        if (nStringLength > 0)
                        {
                            char *pStringSN = new char[nStringLength];
                            errorCodeGetSN = Synexens::GetDeviceSN(pDeviceInfo[i].m_nDeviceID, nStringLength, pStringSN);
                            if (errorCodeGetSN == Synexens::SYERRORCODE_SUCCESS)
                            {
                                RCLCPP_INFO(m_node->get_logger(), "SN%d:%s\n", i, pStringSN);
                                printf("SN%d:%s\n", i, pStringSN);
                            }
                            else
                            {
                                PrintErrorCode("GetDeviceSN", errorCodeGetSN);
                            }
                            delete[] pStringSN;
                        }
                    }
                    else
                    {
                        PrintErrorCode("GetDeviceSN", errorCodeGetSN);
                    }

                    // get version
                    nStringLength = 0;
                    Synexens::SYErrorCode errorCodeGetHWVersion = Synexens::GetDeviceHWVersion(pDeviceInfo[i].m_nDeviceID, nStringLength, nullptr);
                    if (errorCodeGetHWVersion == Synexens::SYERRORCODE_SUCCESS)
                    {
                        if (nStringLength > 0)
                        {
                            char *pStringFWVersion = new char[nStringLength];
                            errorCodeGetHWVersion = Synexens::GetDeviceHWVersion(pDeviceInfo[i].m_nDeviceID, nStringLength, pStringFWVersion);
                            if (errorCodeGetHWVersion == Synexens::SYERRORCODE_SUCCESS)
                            {
                                RCLCPP_INFO(m_node->get_logger(), "HWVersion%d:%s\n", i, pStringFWVersion);
                            }
                            else
                            {
                                PrintErrorCode("GetDeviceHWVersion2", errorCodeGetHWVersion);
                            }
                            delete[] pStringFWVersion;
                        }
                    }
                    else
                    {
                        PrintErrorCode("GetDeviceHWVersion", errorCodeGetHWVersion);
                    }

                    // query devices support frame type
                    int nSupportTypeCount = 0;
                    Synexens::SYErrorCode errorCodeQueryFrameType = Synexens::QueryDeviceSupportFrameType(pDeviceInfo[i].m_nDeviceID, nSupportTypeCount);
                    if (errorCodeQueryFrameType == Synexens::SYERRORCODE_SUCCESS && nSupportTypeCount > 0)
                    {
                        Synexens::SYSupportType *pSupportType = new Synexens::SYSupportType[nSupportTypeCount];
                        errorCodeQueryFrameType = Synexens::QueryDeviceSupportFrameType(pDeviceInfo[i].m_nDeviceID, nSupportTypeCount, pSupportType);
                        if (errorCodeQueryFrameType == Synexens::SYERRORCODE_SUCCESS && nSupportTypeCount > 0)
                        {
                            for (int j = 0; j < nSupportTypeCount; j++)
                            {
                                int nResolutionCount = 0;
                                Synexens::SYErrorCode errorCodeQueryResolution = Synexens::QueryDeviceSupportResolution(pDeviceInfo[i].m_nDeviceID, pSupportType[j], nResolutionCount);
                                if (errorCodeQueryResolution == Synexens::SYERRORCODE_SUCCESS && nResolutionCount > 0)
                                {
                                    Synexens::SYResolution *pResolution = new Synexens::SYResolution[nResolutionCount];
                                    errorCodeQueryResolution = Synexens::QueryDeviceSupportResolution(pDeviceInfo[i].m_nDeviceID, pSupportType[j], nResolutionCount, pResolution);
                                    if (errorCodeQueryResolution == Synexens::SYERRORCODE_SUCCESS && nResolutionCount > 0)
                                    {
                                        for (int k = 0; k < nResolutionCount; k++)
                                        {
                                            // RCLCPP_INFO(m_node->get_logger(), "FrameType%d:%d,Resolution%d:%d\n", j, pSupportType[j], k, pResolution[k]);
                                        }
                                    }
                                    else
                                    {
                                        PrintErrorCode("QueryDeviceSupportResolution2", errorCodeQueryResolution);
                                    }
                                    delete[] pResolution;
                                }
                                else
                                {
                                    PrintErrorCode("QueryDeviceSupportResolution", errorCodeQueryResolution);
                                }
                            }
                        }
                        else
                        {
                            PrintErrorCode("QueryDeviceSupportFrameType2", errorCodeQueryFrameType);
                        }
                        delete[] pSupportType;
                    }
                    else
                    {
                        PrintErrorCode("QueryDeviceSupportFrameType", errorCodeQueryFrameType);
                    }

                    // set camera type start
                    switch (pDeviceInfo[i].m_deviceType)
                    {
                    case Synexens::SYDEVICETYPE_CS30_SINGLE:
                    case Synexens::SYDEVICETYPE_CS30_DUAL:
                    {
                        errorCode = Synexens::SetFrameResolution(pDeviceInfo[i].m_nDeviceID, Synexens::SYFRAMETYPE_DEPTH, m_cameraConfig.CS30DepthResolution);
                        if (errorCode == Synexens::SYERRORCODE_SUCCESS)
                        {
                            errorCode = Synexens::SetFrameResolution(pDeviceInfo[i].m_nDeviceID, Synexens::SYFRAMETYPE_RGB, m_cameraConfig.CS30RGBResolution);
                            if (errorCode == Synexens::SYERRORCODE_SUCCESS)
                            {
                                Synexens::SYStreamType streamType = m_cameraConfig.CS30StreamType;
                                errorCode = Synexens::StartStreaming(pDeviceInfo[i].m_nDeviceID, streamType);
                                if (errorCode == Synexens::SYERRORCODE_SUCCESS)
                                {
                                    auto itStreamFind = m_mapStreamType.find(pDeviceInfo[i].m_nDeviceID);
                                    if (itStreamFind != m_mapStreamType.end())
                                    {
                                        itStreamFind->second = streamType;
                                    }
                                    else
                                    {
                                        m_mapStreamType.insert(std::pair<unsigned int, Synexens::SYStreamType>(pDeviceInfo[i].m_nDeviceID, streamType));
                                    }

                                    pOpen[i] = true;
                                }
                                else
                                {
                                    PrintErrorCode("StartStreaming", errorCode);
                                }
                            }
                            else
                            {
                                PrintErrorCode("SetFrameResolution RGB", errorCode);
                            }
                        }
                        else
                        {
                            PrintErrorCode("SetFrameResolution Depth", errorCode);
                        }

                        break;
                    }
                    case Synexens::SYDEVICETYPE_CS20_SINGLE:
                    {
                        errorCode = Synexens::SetFrameResolution(pDeviceInfo[i].m_nDeviceID, Synexens::SYFRAMETYPE_DEPTH, m_cameraConfig.CS20DepthResolution);
                        if (errorCode == Synexens::SYERRORCODE_SUCCESS)
                        {
                            Synexens::SYStreamType streamType = m_cameraConfig.CS20StreamType;
                            errorCode = Synexens::StartStreaming(pDeviceInfo[i].m_nDeviceID, streamType);
                            if (errorCode == Synexens::SYERRORCODE_SUCCESS)
                            {
                                auto itStreamFind = m_mapStreamType.find(pDeviceInfo[i].m_nDeviceID);
                                if (itStreamFind != m_mapStreamType.end())
                                {
                                    itStreamFind->second = streamType;
                                }
                                else
                                {
                                    m_mapStreamType.insert(std::pair<unsigned int, Synexens::SYStreamType>(pDeviceInfo[i].m_nDeviceID, streamType));
                                }
                                pOpen[i] = true;
                            }
                            else
                            {
                                PrintErrorCode("StartStreaming", errorCode);
                            }
                        }
                        else
                        {
                            PrintErrorCode("SetFrameResolution Depth", errorCode);
                        }

                        break;
                    }
                    case Synexens::SYDEVICETYPE_CS20_DUAL:
                    {
                        errorCode = Synexens::SetFrameResolution(pDeviceInfo[i].m_nDeviceID, Synexens::SYFRAMETYPE_DEPTH, m_cameraConfig.CS20DepthResolution);
                        if (errorCode == Synexens::SYERRORCODE_SUCCESS)
                        {
                            Synexens::SYStreamType streamType = m_cameraConfig.CS20StreamType;
                            errorCode = Synexens::StartStreaming(pDeviceInfo[i].m_nDeviceID, streamType);
                            if (errorCode == Synexens::SYERRORCODE_SUCCESS)
                            {
                                auto itStreamFind = m_mapStreamType.find(pDeviceInfo[i].m_nDeviceID);
                                if (itStreamFind != m_mapStreamType.end())
                                {
                                    itStreamFind->second = streamType;
                                }
                                else
                                {
                                    m_mapStreamType.insert(std::pair<unsigned int, Synexens::SYStreamType>(pDeviceInfo[i].m_nDeviceID, streamType));
                                }

                                pOpen[i] = true;
                            }
                            else
                            {
                                PrintErrorCode("StartStreaming", errorCode);
                            }
                        }
                        else
                        {
                            PrintErrorCode("SetFrameResolution Depth", errorCode);
                        }

                        break;
                    }
                    case Synexens::SYDEVICETYPE_CS40:
                    {
                        errorCode = Synexens::SetFrameResolution(pDeviceInfo[i].m_nDeviceID, Synexens::SYFRAMETYPE_DEPTH, m_cameraConfig.CS40DepthResolution);
                        if (errorCode == Synexens::SYERRORCODE_SUCCESS)
                        {
                            Synexens::SYStreamType streamType = m_cameraConfig.CS40StreamType;
                            errorCode = Synexens::StartStreaming(pDeviceInfo[i].m_nDeviceID, streamType);
                            if (errorCode == Synexens::SYERRORCODE_SUCCESS)
                            {
                                auto itStreamFind = m_mapStreamType.find(pDeviceInfo[i].m_nDeviceID);
                                if (itStreamFind != m_mapStreamType.end())
                                {
                                    itStreamFind->second = streamType;
                                }
                                else
                                {
                                    m_mapStreamType.insert(std::pair<unsigned int, Synexens::SYStreamType>(pDeviceInfo[i].m_nDeviceID, streamType));
                                }

                                pOpen[i] = true;
                            }
                            else
                            {
                                PrintErrorCode("StartStreaming", errorCode);
                            }
                        }
                        else
                        {
                            PrintErrorCode("SetFrameResolution Depth", errorCode);
                        }

                        break;
                    }

                    case Synexens::SYDEVICETYPE_CS20_P:
                    {
                        errorCode = Synexens::SetFrameResolution(pDeviceInfo[i].m_nDeviceID, Synexens::SYFRAMETYPE_DEPTH, m_cameraConfig.CS20PDepthResolution);
                        if (errorCode == Synexens::SYERRORCODE_SUCCESS)
                        {
                            Synexens::SYStreamType streamType = m_cameraConfig.CS20PStreamType;
                            errorCode = Synexens::StartStreaming(pDeviceInfo[i].m_nDeviceID, streamType);
                            if (errorCode == Synexens::SYERRORCODE_SUCCESS)
                            {
                                auto itStreamFind = m_mapStreamType.find(pDeviceInfo[i].m_nDeviceID);
                                if (itStreamFind != m_mapStreamType.end())
                                {
                                    itStreamFind->second = streamType;
                                }
                                else
                                {
                                    m_mapStreamType.insert(std::pair<unsigned int, Synexens::SYStreamType>(pDeviceInfo[i].m_nDeviceID, streamType));
                                }

                                pOpen[i] = true;
                            }
                            else
                            {
                                PrintErrorCode("StartStreaming", errorCode);
                            }
                        }
                        else
                        {
                            PrintErrorCode("SetFrameResolution Depth", errorCode);
                        }

                        break;
                    }
                    }
                }
                else
                {
                    PrintErrorCode("OpenDevice", errorCodeOpenDevice);
                }
            }

            // set optin
            // for (int nDeviceIndex = 0; nDeviceIndex < nCount; nDeviceIndex++)
            // {
            //     if (pOpen[nDeviceIndex])
            //     {
            //         SetOption(pDeviceInfo[nDeviceIndex].m_nDeviceID, pDeviceInfo[nDeviceIndex].m_deviceType);
            //     }
            // }

            RCLCPP_INFO(m_node->get_logger(), "Synexens Started");
            rclcpp::WallRate loop_rate(60);
            while (rclcpp::ok())
            {
                // RCLCPP_INFO(m_node->get_logger(), "nCount:%d", nCount);
                for (int nDeviceIndex = 0; nDeviceIndex < nCount; nDeviceIndex++)
                {

                    // RCLCPP_INFO(m_node->get_logger(), "N_Device_Index:%d", nDeviceIndex);
                    if (pOpen[nDeviceIndex])
                    {
                        // frame data
                        Synexens::SYFrameData *pLastFrameData = nullptr;
                        Synexens::SYErrorCode errorCodeLastFrame = Synexens::GetLastFrameData(pDeviceInfo[nDeviceIndex].m_nDeviceID, pLastFrameData);
                        if (errorCodeLastFrame == Synexens::SYERRORCODE_SUCCESS)
                        {
                            ProcessFrameData(pDeviceInfo[nDeviceIndex].m_nDeviceID, pLastFrameData);
                            if(m_bSetOptionStatus == true)
                            {
                                SetOption(pDeviceInfo[nDeviceIndex].m_nDeviceID, pDeviceInfo[nDeviceIndex].m_deviceType);
                                m_bSetOptionStatus = false;
                            }
                        }
                    }
                }

                rclcpp::spin_some(m_node);
                loop_rate.sleep();
            }
        }
        else
        {
            PrintErrorCode("FindDevice2", errorCode);
            return errorCode;
        }
    }
    else
    {
        PrintErrorCode("FindDevice1", errorCode);
        return Synexens::SYERRORCODE_FAILED;
    }

    errorCodeInitSDK = Synexens::UnInitSDK();
    if (errorCodeInitSDK != Synexens::SYERRORCODE_SUCCESS)
    {
        PrintErrorCode("UnInitSDK", errorCodeInitSDK);
    }
    return Synexens::SYERRORCODE_SUCCESS;
}

void SYRosDevice::ProcessFrameData(unsigned int nDeviceID, Synexens::SYFrameData *pFrameData)
{
    auto itStreamFind = m_mapStreamType.find(nDeviceID);
    if (itStreamFind == m_mapStreamType.end())
    {
        return;
    }

    // ToDo: Delete the line afterwards
    // RCLCPP_INFO(m_node->get_logger(), "SDKVersion:%d", nDeviceID);
    

    // ros::Time capture_time;
    rclcpp::Time capture_time = m_node->now();
    if (itStreamFind->second == Synexens::SYSTREAMTYPE_RGBD)
    {
        std::map<Synexens::SYFrameType, int> mapIndex;
        std::map<Synexens::SYFrameType, int> mapPos;
        int nPos = 0;
        for (int nFrameIndex = 0; nFrameIndex < pFrameData->m_nFrameCount; nFrameIndex++)
        {
            mapIndex.insert(std::pair<Synexens::SYFrameType, int>(pFrameData->m_pFrameInfo[nFrameIndex].m_frameType, nFrameIndex));
            mapPos.insert(std::pair<Synexens::SYFrameType, int>(pFrameData->m_pFrameInfo[nFrameIndex].m_frameType, nPos));
            nPos += pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight * pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth * sizeof(short);
        }

        auto itDepthIndex = mapIndex.find(Synexens::SYFRAMETYPE_DEPTH);
        auto itRGBIndex = mapIndex.find(Synexens::SYFRAMETYPE_RGB);
        int nRGBDWidth = pFrameData->m_pFrameInfo[itRGBIndex->second].m_nFrameWidth;
        int nRGBDHeight = pFrameData->m_pFrameInfo[itRGBIndex->second].m_nFrameHeight;
        unsigned short *pRGBDDepth = new unsigned short[nRGBDWidth * nRGBDHeight];
        memset(pRGBDDepth, 0, sizeof(unsigned short) * nRGBDWidth * nRGBDHeight);
        unsigned char *pRGBDRGB = new unsigned char[nRGBDWidth * nRGBDHeight * 3];
        memset(pRGBDRGB, 0, sizeof(unsigned char) * nRGBDWidth * nRGBDHeight * 3);
        if (itDepthIndex != mapIndex.end() && itRGBIndex != mapIndex.end())
        {
            if (Synexens::GetRGBD(nDeviceID, pFrameData->m_pFrameInfo[itDepthIndex->second].m_nFrameWidth, pFrameData->m_pFrameInfo[itDepthIndex->second].m_nFrameHeight, (unsigned short *)pFrameData->m_pData + mapPos[Synexens::SYFRAMETYPE_DEPTH],
                                  pFrameData->m_pFrameInfo[itRGBIndex->second].m_nFrameWidth, pFrameData->m_pFrameInfo[itRGBIndex->second].m_nFrameHeight, (unsigned char *)pFrameData->m_pData + mapPos[Synexens::SYFRAMETYPE_RGB],
                                  nRGBDWidth, nRGBDHeight, pRGBDDepth, pRGBDRGB) == Synexens::SYERRORCODE_SUCCESS)
            {
                // device resolution
                SYResolution resolution;
                SYErrorCode errorCode = Synexens::GetFrameResolution(nDeviceID, SYFRAMETYPE_DEPTH, resolution);

                // camera intrinsics
                SYIntrinsics intrinsics;
                errorCode = Synexens::GetIntric(nDeviceID, resolution, intrinsics);
                if (errorCode != SYERRORCODE_SUCCESS)
                {
                    return;
                }

                cv::Mat depth_frame_buffer_mat(nRGBDHeight, nRGBDWidth, CV_16UC1, pRGBDDepth);
                Image::SharedPtr depthImage = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO16, depth_frame_buffer_mat).toImageMsg();

                // point
                if (m_cameraParams.point_cloud_enabled)
                {
                    // get point
                    int nCount = nRGBDWidth * nRGBDHeight;
                    SYPointCloudData *pPCLData = new SYPointCloudData[nCount];
                    errorCode = Synexens::GetDepthPointCloud(nDeviceID, nRGBDWidth, nRGBDHeight, pRGBDDepth, pPCLData, false);
                    if (errorCode != SYERRORCODE_SUCCESS)
                    {
                        delete pPCLData;
                        return;
                    }
                    // point
                    PointCloud2::SharedPtr pointCloud(new msg::PointCloud2);
                    FillPointCloud(nRGBDWidth, nRGBDHeight, pPCLData, pointCloud);
                    if (nDeviceID == 1)
                    {
                        RCLCPP_INFO(m_node->get_logger(), "Right is getting triggered: %d", nDeviceID);   
                        pointCloud->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrameRight;
                        pointCloud->header.stamp = capture_time;
                        m_mapPointsPublisher[nDeviceID][POINTS]->publish(*pointCloud.get());

                        delete pPCLData;    
                    }
                    else if (nDeviceID == 2)
                    {
                        RCLCPP_INFO(m_node->get_logger(), "Left is getting triggered: %d", nDeviceID);   
                        pointCloud->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrameLeft;
                        pointCloud->header.stamp = capture_time;
                        m_mapPointsPublisher[nDeviceID][POINTS]->publish(*pointCloud.get());

                        delete pPCLData; 
                    }                    
                    // pointCloud->header.stamp = capture_time;
                    
                }

                // frame_id stamp ros publish
                depthImage->header.stamp = capture_time;
                if (nDeviceID == 1)
                {
                    depthImage->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrameRight;
                   
                    m_mapImagePublisher[nDeviceID][DEPTH].publish(depthImage);
                    m_calibrationData.getDepthCameraInfoRight(m_depthCameraInfo, &intrinsics);
                    m_mapRosPublisher[nDeviceID][DEPTH]->publish(m_depthCameraInfo);
                }
                else if(nDeviceID == 2)
                {
                    depthImage->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrameLeft;

                    m_mapImagePublisher[nDeviceID][DEPTH].publish(depthImage);
                    m_calibrationData.getDepthCameraInfoLeft(m_depthCameraInfo, &intrinsics);
                    m_mapRosPublisher[nDeviceID][DEPTH]->publish(m_depthCameraInfo);
                }
                // m_mapImagePublisher[nDeviceID][DEPTH].publish(depthImage);

                // m_calibrationData.getDepthCameraInfo(m_depthCameraInfo, &intrinsics);
                // m_mapRosPublisher[nDeviceID][DEPTH]->publish(m_depthCameraInfo);

                // rgb
                cv::Mat bgrImgRGB(nRGBDHeight, nRGBDWidth, CV_8UC3, pRGBDRGB);

                Image::SharedPtr rgbImage = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, bgrImgRGB).toImageMsg();
                m_mapImagePublisher[nDeviceID][RGB].publish(rgbImage);
            }
        }
    }
    else
    {
        int nPos = 0;
        for (int nFrameIndex = 0; nFrameIndex < pFrameData->m_nFrameCount; nFrameIndex++)
        {
            int nCount = pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth * pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight;
            switch (pFrameData->m_pFrameInfo[nFrameIndex].m_frameType)
            {
            case Synexens::SYFRAMETYPE_DEPTH:
            {
                cv::Mat depth_frame_buffer_mat(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_16UC1, (unsigned char *)pFrameData->m_pData + nPos);
                Image::SharedPtr depthImage = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_buffer_mat).toImageMsg();

                // device resolution
                SYResolution resolution;
                SYErrorCode errorCode = Synexens::GetFrameResolution(nDeviceID, SYFRAMETYPE_DEPTH, resolution);

                // camera intrinsics
                SYIntrinsics intrinsics;
                errorCode = Synexens::GetIntric(nDeviceID, resolution, intrinsics);
                if (errorCode != SYERRORCODE_SUCCESS)
                {
                    continue;
                }

                // point
                if (m_cameraParams.point_cloud_enabled)
                {
                    // get point
                    SYPointCloudData *pPCLData = new SYPointCloudData[nCount];
                    errorCode = Synexens::GetDepthPointCloud(nDeviceID, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, (unsigned short *)((unsigned char *)pFrameData->m_pData + nPos), pPCLData, false);
                    if (errorCode != SYERRORCODE_SUCCESS)
                    {
                        delete pPCLData;
                        continue;
                    }
                    // point
                    PointCloud2::SharedPtr pointCloud(new msg::PointCloud2);
                    FillPointCloud(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pPCLData, pointCloud);
                    if (nDeviceID == 1)
                    {
                        
                        pointCloud->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrameRight;
                        pointCloud->header.stamp = capture_time;
                        m_mapPointsPublisher[nDeviceID][POINTS]->publish(*pointCloud.get());
                        delete pPCLData;
                    }
                    else if (nDeviceID == 2)
                    {
                        pointCloud->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrameLeft;
                        pointCloud->header.stamp = capture_time;
                        m_mapPointsPublisher[nDeviceID][POINTS]->publish(*pointCloud.get());
                        delete pPCLData;

                    }
                    // pointCloud->header.stamp = capture_time;


                }

                // frame_id stamp ros publish
                depthImage->header.stamp = capture_time;
                if (nDeviceID == 1)
                {
                    depthImage->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrameRight;

                    m_mapImagePublisher[nDeviceID][DEPTH].publish(depthImage);

                    m_calibrationData.getDepthCameraInfoRight(m_depthCameraInfo, &intrinsics);
                    m_mapRosPublisher[nDeviceID][DEPTH]->publish(m_depthCameraInfo);
                }
                else if (nDeviceID == 2)
                {
                    depthImage->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrameLeft;

                    m_mapImagePublisher[nDeviceID][DEPTH].publish(depthImage);

                    m_calibrationData.getDepthCameraInfoLeft(m_depthCameraInfo, &intrinsics);
                    m_mapRosPublisher[nDeviceID][DEPTH]->publish(m_depthCameraInfo);
                }
                // m_mapImagePublisher[nDeviceID][DEPTH].publish(depthImage);

                // m_calibrationData.getDepthCameraInfo(m_depthCameraInfo, &intrinsics);
                // m_mapRosPublisher[nDeviceID][DEPTH]->publish(m_depthCameraInfo);

                nPos += pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight * pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth * sizeof(short);

                break;
            }
            case Synexens::SYFRAMETYPE_IR:
            {

                cv::Mat ir_buffer_mat(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_16UC1, (unsigned char *)pFrameData->m_pData + nPos);
                Image::SharedPtr irImage = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO16, ir_buffer_mat).toImageMsg();

                // device resolution
                SYResolution resolution;
                SYErrorCode errorCode = Synexens::GetFrameResolution(nDeviceID, SYFRAMETYPE_IR, resolution);

                // camera intrinsics
                SYIntrinsics intrinsics;
                errorCode = Synexens::GetIntric(nDeviceID, resolution, intrinsics);
                if (errorCode != SYERRORCODE_SUCCESS)
                    continue;

                irImage->header.stamp = capture_time;
                if (nDeviceID == 1)
                {
                    irImage->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrameRight;
                    m_mapImagePublisher[nDeviceID][IR].publish(irImage);

                    m_calibrationData.getDepthCameraInfoRight(m_irCameraInfo, &intrinsics);
                    m_mapRosPublisher[nDeviceID][IR]->publish(m_irCameraInfo);
                }
                else if (nDeviceID == 2)
                {
                    irImage->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrameLeft;
                    m_mapImagePublisher[nDeviceID][IR].publish(irImage);

                    m_calibrationData.getDepthCameraInfoLeft(m_irCameraInfo, &intrinsics);
                    m_mapRosPublisher[nDeviceID][IR]->publish(m_irCameraInfo);
                }
                // m_mapImagePublisher[nDeviceID][IR].publish(irImage);

                // m_calibrationData.getDepthCameraInfo(m_irCameraInfo, &intrinsics);
                // m_mapRosPublisher[nDeviceID][IR]->publish(m_irCameraInfo);

                nPos += pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight * pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth * sizeof(short);
                break;
            }
            case Synexens::SYFRAMETYPE_RGB:
            {
                cv::Mat yuvImg(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_8UC2, (unsigned char *)pFrameData->m_pData + nPos);
                cv::Mat rgbImg;
                cv::Mat bgrImg;
                rgbImg = cv::Mat(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_8UC3);
                bgrImg = cv::Mat(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_8UC3);
                cv::cvtColor(yuvImg, rgbImg, cv::ColorConversionCodes::COLOR_YUV2RGB_YUYV);
                cv::cvtColor(rgbImg, bgrImg, cv::ColorConversionCodes::COLOR_RGB2BGR);

                Image::SharedPtr rgbImage = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, bgrImg).toImageMsg();
                m_mapImagePublisher[nDeviceID][RGB].publish(rgbImage);

                nPos += pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight * pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth * 3 / 2;
                break;
            }
            }
        }
    }
}

void SYRosDevice::FillPointCloud(int nWidth, int nHeight, SYPointCloudData *pPCLData, PointCloud2::SharedPtr &pointCloud)
{
    pointCloud->height = nHeight;
    pointCloud->width = nWidth;
    pointCloud->is_dense = false;
    pointCloud->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*pointCloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(*pointCloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pointCloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pointCloud, "z");

    int nPointCount = nHeight * nWidth;
    pcd_modifier.resize(nPointCount);

    for (int i = 0; i < nPointCount; i++, ++iter_x, ++iter_y, ++iter_z)
    {
        if (pPCLData[i].m_fltZ <= 0.0f)
        {
            *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
            constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
            *iter_x = kMillimeterToMeter * pPCLData[i].m_fltX;
            *iter_y = kMillimeterToMeter * pPCLData[i].m_fltY;
            *iter_z = kMillimeterToMeter * pPCLData[i].m_fltZ;
        }
    }
}

void SYRosDevice::SetOption(unsigned int nDeviceID, SYDeviceType deviceType)
{
    switch (deviceType)
    {
    case SYDEVICETYPE_CS30_DUAL:
    case SYDEVICETYPE_CS30_SINGLE:
    {
        if (m_cameraParams.CS30_exposure >= m_cameraParams.CS30_exposure_range_min && m_cameraParams.CS30_exposure <= m_cameraParams.CS30_exposure_range_max)
        {
            Synexens::SetIntegralTime(nDeviceID, m_cameraParams.CS30_exposure);
        }

        // set filter
        if (m_cameraParams.CS30_depth_image_filter >= 0)
        {
            Synexens::SetFilter(nDeviceID, true);

            int nParamNumber = 1;
            float fParamValue[10] = {0};
            Synexens::SYFilterType filterType = Synexens::SYFILTERTYPE_NULL;
            if (m_cameraParams.CS30_filter_amplititud_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_AMPLITUDE;
                fParamValue[0] = m_cameraParams.CS30_filter_amplititud_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS30_filter_gauss_value_01 >= 0 && m_cameraParams.CS30_filter_median_value_02 >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_GAUSS;
                fParamValue[0] = m_cameraParams.CS30_filter_gauss_value_01;
                fParamValue[1] = m_cameraParams.CS30_filter_gauss_value_02;
                nParamNumber = 2;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS30_filter_median_value_01 >= 0 && m_cameraParams.CS30_filter_median_value_02 >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_MEDIAN;
                fParamValue[0] = m_cameraParams.CS30_filter_median_value_01;
                fParamValue[1] = m_cameraParams.CS30_filter_median_value_02;
                nParamNumber = 2;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS30_filter_speckle_value_01 >= 0 && m_cameraParams.CS30_filter_speckle_value_02 >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_SPECKLE;
                fParamValue[0] = m_cameraParams.CS30_filter_speckle_value_01;
                fParamValue[1] = m_cameraParams.CS30_filter_speckle_value_02;
                nParamNumber = 2;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS30_filter_edge_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_EDGE;
                fParamValue[0] = m_cameraParams.CS30_filter_edge_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS30_filter_edge_mad_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_EDGE_MAD;
                fParamValue[0] = m_cameraParams.CS30_filter_edge_mad_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS30_filter_okada_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_OKADA;
                fParamValue[0] = m_cameraParams.CS30_filter_okada_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }
        }

        break;
    }
    case SYDEVICETYPE_CS20_DUAL:
    case SYDEVICETYPE_CS20_SINGLE:
    {
        if (m_cameraParams.CS20_exposure >= m_cameraParams.CS20_exposure_range_min && m_cameraParams.CS20_exposure <= m_cameraParams.CS20_exposure_range_max)
        {
            Synexens::SetIntegralTime(nDeviceID, m_cameraParams.CS20_exposure);
        }

        // set filter
        if (m_cameraParams.CS20_depth_image_filter >= 0)
        {
            Synexens::SetFilter(nDeviceID, true);

            int nParamNumber = 1;
            float fParamValue[10] = {0};
            Synexens::SYFilterType filterType = Synexens::SYFILTERTYPE_NULL;
            if (m_cameraParams.CS20_filter_amplititud_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_AMPLITUDE;
                fParamValue[0] = m_cameraParams.CS20_filter_amplititud_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS20_filter_gauss_value_01 >= 0 && m_cameraParams.CS20_filter_median_value_02 >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_GAUSS;
                fParamValue[0] = m_cameraParams.CS20_filter_gauss_value_01;
                fParamValue[1] = m_cameraParams.CS20_filter_gauss_value_02;
                nParamNumber = 2;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS20_filter_median_value_01 >= 0 && m_cameraParams.CS20_filter_median_value_02 >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_MEDIAN;
                fParamValue[0] = m_cameraParams.CS20_filter_median_value_01;
                fParamValue[1] = m_cameraParams.CS20_filter_median_value_02;
                nParamNumber = 2;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS20_filter_speckle_value_01 >= 0 && m_cameraParams.CS20_filter_speckle_value_02 >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_SPECKLE;
                fParamValue[0] = m_cameraParams.CS20_filter_speckle_value_01;
                fParamValue[1] = m_cameraParams.CS20_filter_speckle_value_02;
                nParamNumber = 2;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS20_filter_edge_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_EDGE;
                fParamValue[0] = m_cameraParams.CS20_filter_edge_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS20_filter_edge_mad_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_EDGE_MAD;
                fParamValue[0] = m_cameraParams.CS20_filter_edge_mad_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS20_filter_okada_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_OKADA;
                fParamValue[0] = m_cameraParams.CS20_filter_okada_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }
        }
        break;
    }
    case SYDEVICETYPE_CS20_P:
    {
        if (m_cameraParams.CS20P_exposure >= m_cameraParams.CS20P_exposure_range_min && m_cameraParams.CS20P_exposure <= m_cameraParams.CS20P_exposure_range_max)
        {
            Synexens::SetIntegralTime(nDeviceID, m_cameraParams.CS20P_exposure);
        }

        // set filter
        if (m_cameraParams.CS20P_depth_image_filter >= 0)
        {
            Synexens::SetFilter(nDeviceID, true);

            int nParamNumber = 1;
            float fParamValue[10] = {0};
            Synexens::SYFilterType filterType = Synexens::SYFILTERTYPE_NULL;
            if (m_cameraParams.CS20P_filter_amplititud_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_AMPLITUDE;
                fParamValue[0] = m_cameraParams.CS20P_filter_amplititud_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS20P_filter_gauss_value_01 >= 0 && m_cameraParams.CS20P_filter_median_value_02 >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_GAUSS;
                fParamValue[0] = m_cameraParams.CS20P_filter_gauss_value_01;
                fParamValue[1] = m_cameraParams.CS20P_filter_gauss_value_02;
                nParamNumber = 2;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS20P_filter_median_value_01 >= 0 && m_cameraParams.CS20P_filter_median_value_02 >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_MEDIAN;
                fParamValue[0] = m_cameraParams.CS20P_filter_median_value_01;
                fParamValue[1] = m_cameraParams.CS20P_filter_median_value_02;
                nParamNumber = 2;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS20P_filter_speckle_value_01 >= 0 && m_cameraParams.CS20P_filter_speckle_value_02 >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_SPECKLE;
                fParamValue[0] = m_cameraParams.CS20P_filter_speckle_value_01;
                fParamValue[1] = m_cameraParams.CS20P_filter_speckle_value_02;
                nParamNumber = 2;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS20P_filter_edge_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_EDGE;
                fParamValue[0] = m_cameraParams.CS20P_filter_edge_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS20P_filter_edge_mad_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_EDGE_MAD;
                fParamValue[0] = m_cameraParams.CS20P_filter_edge_mad_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS20P_filter_okada_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_OKADA;
                fParamValue[0] = m_cameraParams.CS20P_filter_okada_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }
        }
        break;
    }
    case SYDEVICETYPE_CS40:
    {
        if (m_cameraParams.CS40_exposure >= m_cameraParams.CS40_exposure_range_min && m_cameraParams.CS40_exposure <= m_cameraParams.CS40_exposure_range_max)
        {
            Synexens::SetIntegralTime(nDeviceID, m_cameraParams.CS40_exposure);
        }

        // set filter
        if (m_cameraParams.CS40_depth_image_filter >= 0)
        {
            Synexens::SetFilter(nDeviceID, true);

            int nParamNumber = 1;
            float fParamValue[10] = {0};
            Synexens::SYFilterType filterType = Synexens::SYFILTERTYPE_NULL;
            if (m_cameraParams.CS40_filter_amplititud_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_AMPLITUDE;
                fParamValue[0] = m_cameraParams.CS40_filter_amplititud_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS40_filter_gauss_value_01 >= 0 && m_cameraParams.CS40_filter_median_value_02 >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_GAUSS;
                fParamValue[0] = m_cameraParams.CS40_filter_gauss_value_01;
                fParamValue[1] = m_cameraParams.CS40_filter_gauss_value_02;
                nParamNumber = 2;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS40_filter_median_value_01 >= 0 && m_cameraParams.CS40_filter_median_value_02 >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_MEDIAN;
                fParamValue[0] = m_cameraParams.CS40_filter_median_value_01;
                fParamValue[1] = m_cameraParams.CS40_filter_median_value_02;
                nParamNumber = 2;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS40_filter_speckle_value_01 >= 0 && m_cameraParams.CS40_filter_speckle_value_02 >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_SPECKLE;
                fParamValue[0] = m_cameraParams.CS40_filter_speckle_value_01;
                fParamValue[1] = m_cameraParams.CS40_filter_speckle_value_02;
                nParamNumber = 2;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS40_filter_edge_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_EDGE;
                fParamValue[0] = m_cameraParams.CS40_filter_edge_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS40_filter_edge_mad_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_EDGE_MAD;
                fParamValue[0] = m_cameraParams.CS40_filter_edge_mad_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }

            if (m_cameraParams.CS40_filter_okada_value >= 0)
            {
                filterType = Synexens::SYFILTERTYPE_OKADA;
                fParamValue[0] = m_cameraParams.CS40_filter_okada_value;
                nParamNumber = 1;
                Synexens::SetFilterParam(nDeviceID, filterType, nParamNumber, fParamValue);
            }
        }
        break;
    }
    default:
        break;
    }
}