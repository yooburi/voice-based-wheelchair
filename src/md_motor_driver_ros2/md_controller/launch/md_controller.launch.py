import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set the path to this package.
  pkg_share = FindPackageShare(package='md_controller').find('md_controller')

  # Set the path to the RViz configuration settings
 

  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
 

  
  # Launch motor driver controller
  md_controller_cmd = Node(
    package='md_controller',
    executable='md_controller',
    parameters=[{
      "MDUI":184,
      "MDT":183,
      "Port":"/dev/ttyACM0",
      "Baudrate":57600,
      "ID":1,
      "GearRatio":4,
      "poles":20,
      "wheel_radius": 0.1,
      "wheel_base": 0.4
    }],
    output='screen'
  )

  # 
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options


  # Add any actions
  ld.add_action(md_controller_cmd)
  

  return ld
