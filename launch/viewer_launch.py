
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os.path
 
def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('synexens_ros2'), 'rviz', 'view.rviz')
    print(rviz_config_dir)
    return LaunchDescription([
        Node(
            package='synexens_ros2',
            executable='synexens_ros2_node',
            parameters=[{
                # Prefix added to tf frame IDs. It typically contains a trailing '_' unless empty.
                'tf_prefix': '',
                # The FPS of the RGB and Depth cameras. Options are: 5, 7, 15, 30
                'fps': 30,
                # Generate a point cloud from depth data. Requires depth_enabled
                'point_cloud_enabled': True,

                # +++++++++++++++++++++++++++++++++++++++++++++++++ CS30 CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++
                # Enable or disable the depth camera
                'CS30_depth_enabled' : True,
                # The resolution of the depth frame. Options are: 240P, 480P
                'CS30_depth_resolution' : '480P',
                # Enable or disable the ir camera
                'CS30_ir_enabled' : True,
                # Enable or disable the color camera
                'CS30_color_enabled' : True,
                # Resolution at which to run the color camera. Valid options: 1080P 540P 480P
                'CS30_color_resolution' : '540P',
                # True if mapped depth in color space should be enabled mapping depth resolution 480P rgb resolution 1080P 540P
                'CS30_mapping_enabled' : False,
                # The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=3000
                'CS30_exposure' : 3000,
                # The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=0
                'CS30_exposure_range_min' : 0,
                # The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=3000
                'CS30_exposure_range_max' : 3000,
                # 0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1
                'CS30_depth_image_filter' : -1,
                # AMPLITITUD value sett. Use default setting if value=-1. Min value 0 , Max value 100, Recommend value 6
                'CS30_filter_amplititud_value' : -1.0,
                # MEDIAN_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 3
                'CS30_filter_median_value_01' : -1.0,
                # MEDIAN_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
                'CS30_filter_median_value_02' : -1.0,
                # GAUSS_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 1
                'CS30_filter_gauss_value_01' : -1.0,
                # GAUSS_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
                'CS30_filter_gauss_value_02' : -1.0,
                # EDGE value sett. Use default setting if value=-1. Min value 20, Max value 200, Recommend value 50
                'CS30_filter_edge_value' : -1.0,
                # SPECKLE_01 value sett. Use default setting if value=-1. Min value 24, Max value 200, Recommend value 40
                'CS30_filter_speckle_value_01' : -1.0,
                # SPECKLE_02 value sett. Use default setting if value=-1. Min value 40, Max value 200
                'CS30_filter_speckle_value_02' : -1.0,
                # SOBEL value sett. Use default setting if value=-1. Min value 20, Max value 300, Recommend value 150
                'CS30_filter_sobel_value' : -1.0,
                # EDGE_MAD value sett. Use default setting if value=-1. Min value 5, Max value 100, Recommend value 15
                'CS30_filter_edge_mad_value' : -1.0,
                # OKADA value sett. Use default setting if value=-1. Min value 10, Max value 100, Recommend value 10
                'CS30_filter_okada_value' : -1.0,

                # +++++++++++++++++++++++++++++++++++++++++++++++++ CS20 CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++
                # Enable or disable the depth camera
                'CS20_depth_enabled' : True,
                # The resolution of the depth frame. Options are: 240P, 480P. SINGLE not supported 480P
                'CS20_depth_resolution' : '240P',
                # Enable or disable the ir camera
                'CS20_ir_enabled' : True,
                # The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=3000
                'CS20_exposure' : 3000,
                # The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=0
                'CS20_exposure_range_min' : 0,
                # The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=3000
                'CS20_exposure_range_max' : 3000,
                # 0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1
                'CS20_depth_image_filter' : -1,
                # AMPLITITUD value sett. Use default setting if value=-1. Min value 0 , Max value 100, Recommend value 6
                'CS20_filter_amplititud_value' : -1.0,
                # MEDIAN_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 3
                'CS20_filter_median_value_01' : -1.0,
                # MEDIAN_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
                'CS20_filter_median_value_02' : -1.0,
                # GAUSS_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 1
                'CS20_filter_gauss_value_01' : -1.0,
                # GAUSS_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
                'CS20_filter_gauss_value_02' : -1.0,
                # EDGE value sett. Use default setting if value=-1. Min value 20, Max value 200, Recommend value 50
                'CS20_filter_edge_value' : -1.0,
                # SPECKLE_01 value sett. Use default setting if value=-1. Min value 24, Max value 200, Recommend value 40
                'CS20_filter_speckle_value_01' : -1.0,
                # SPECKLE_02 value sett. Use default setting if value=-1. Min value 40, Max value 200
                'CS20_filter_speckle_value_02' : -1.0,
                # SOBEL value sett. Use default setting if value=-1. Min value 20, Max value 300, Recommend value 150
                'CS20_filter_sobel_value' : -1.0,
                # EDGE_MAD value sett. Use default setting if value=-1. Min value 5, Max value 100, Recommend value 15
                'CS20_filter_edge_mad_value' : -1.0,
                # OKADA value sett. Use default setting if value=-1. Min value 10, Max value 100, Recommend value 10
                'CS20_filter_okada_value' : -1.0,

                # +++++++++++++++++++++++++++++++++++++++++++++++++ CS20-P CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++
                # Enable or disable the depth camera
                'CS20P_depth_enabled' : True,
                # The resolution of the depth frame. Options are: 240P.
                'CS20P_depth_resolution' : '240P',
                # Enable or disable the ir camera
                'CS20P_ir_enabled' : True,
                # The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=3000
                'CS20P_exposure' : 3000,
                # The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=0
                'CS20P_exposure_range_min' : 0,
                # The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=3000
                'CS20P_exposure_range_max' : 3000,
                # 0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1
                'CS20P_depth_image_filter' : -1,
                # AMPLITITUD value sett. Use default setting if value=-1. Min value 0 , Max value 100, Recommend value 6
                'CS20P_filter_amplititud_value' : -1.0,
                # MEDIAN_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 3
                'CS20P_filter_median_value_01' : -1.0,
                # MEDIAN_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
                'CS20P_filter_median_value_02' : -1.0,
                # GAUSS_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 1
                'CS20P_filter_gauss_value_01' : -1.0,
                # GAUSS_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
                'CS20P_filter_gauss_value_02' : -1.0,
                # EDGE value sett. Use default setting if value=-1. Min value 20, Max value 200, Recommend value 50
                'CS20P_filter_edge_value' : -1.0,
                # SPECKLE_01 value sett. Use default setting if value=-1. Min value 24, Max value 200, Recommend value 40
                'CS20P_filter_speckle_value_01' : -1.0,
                # SPECKLE_02 value sett. Use default setting if value=-1. Min value 40, Max value 200
                'CS20P_filter_speckle_value_02' : -1.0,
                # SOBEL value sett. Use default setting if value=-1. Min value 20, Max value 300, Recommend value 150
                'CS20P_filter_sobel_value' : -1.0,
                # EDGE_MAD value sett. Use default setting if value=-1. Min value 5, Max value 100, Recommend value 15
                'CS20P_filter_edge_mad_value' : -1.0,
                # OKADA value sett. Use default setting if value=-1. Min value 10, Max value 100, Recommend value 10
                'CS20P_filter_okada_value' : -1.0,
                
                # +++++++++++++++++++++++++++++++++++++++++++++++++ CS40 CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++
                # Enable or disable the depth camera
                'CS40_depth_enabled' : True,
                # The resolution of the depth frame. Options are: 480P.
                'CS40_depth_resolution' : '480P',
                # Enable or disable the ir camera
                'CS40_ir_enabled' : True,
                # The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=3000
                'CS40_exposure' : 1510,
                # The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=0
                'CS40_exposure_range_min' : 10,
                # The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=3000
                'CS40_exposure_range_max' : 3000,
                # 0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1
                'CS40_depth_image_filter' : -1,
                # AMPLITITUD value sett. Use default setting if value=-1. Min value 0 , Max value 100, Recommend value 6
                'CS40_filter_amplititud_value' : -1.0,
                # MEDIAN_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 3
                'CS40_filter_median_value_01' : -1.0,
                # MEDIAN_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
                'CS40_filter_median_value_02' : -1.0,
                # GAUSS_01 value sett. Use default setting if value=-1. Min value 3, Max value 5, Recommend value 1
                'CS40_filter_gauss_value_01' : -1.0,
                # GAUSS_02 value sett. Use default setting if value=-1. Min value 0, Max value 5, Recommend value 1
                'CS40_filter_gauss_value_02' : -1.0,
                # EDGE value sett. Use default setting if value=-1. Min value 20, Max value 200, Recommend value 50
                'CS40_filter_edge_value' : -1.0,
                # SPECKLE_01 value sett. Use default setting if value=-1. Min value 24, Max value 200, Recommend value 40
                'CS40_filter_speckle_value_01' : -1.0,
                # SPECKLE_02 value sett. Use default setting if value=-1. Min value 40, Max value 200
                'CS40_filter_speckle_value_02' : -1.0,
                # SOBEL value sett. Use default setting if value=-1. Min value 20, Max value 300, Recommend value 150
                'CS40_filter_sobel_value' : -1.0,
                # EDGE_MAD value sett. Use default setting if value=-1. Min value 5, Max value 100, Recommend value 15
                'CS40_filter_edge_mad_value' : -1.0,
                # OKADA value sett. Use default setting if value=-1. Min value 10, Max value 100, Recommend value 10
                'CS40_filter_okada_value' : -1.0,
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir]
        )
    ])