from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.substitution import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    """
    Loads the URDF model of the synexens robot.

    This launch file assumes the URDF model is located within the `synexens_ros1/urdf` directory.
    """
    # Define the package path
    # pkgPath = launch_ros.substitutions.FindPackageShare(package="synexens_ros2").find("synexens_ros2")
    # pkgPath = launch_ros.substitutions.FindPackageShare(package="synexens_ros2").find("synexens_ros2")
    
    # Find the path for the urdf and config files
    # urdfModelPath = os.path.join(pkgPath, "urdf/synexens.urdf.xacro")
    urdfModelPath = "/home/jyo/ros2_ws/src/synexens_ros2/urdf/synexens.urdf.xacro"

    print(urdfModelPath)

    # Read the contents of the URDF
    with open(urdfModelPath, "r") as infp:
        robot_desc = infp.read()

    # Add the robot description to parameter
    params = {'robot_description': robot_desc}

    # Setup objects of the node with right parameters passed to it 
    # Robot state publisher
    robot_state_publisher_node = launch_ros.actions.Node(package='robot_state_publisher',
                                                         executable='robot_state_publisher',
                                                         output='screen',
                                                         parameters=[params],
                                                         arguments=[urdfModelPath])
    
    # Joint state publisher 
    joint_state_publisher_node = launch_ros.actions.Node(package='joint_state_publisher',
                                                         executable='joint_state_publisher',
                                                         output='screen',
                                                         parameters=[params],
                                                         arguments=[urdfModelPath])
  
    return LaunchDescription([robot_state_publisher_node,
                              joint_state_publisher_node])
