import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the launch folder
    launch_dir = os.path.dirname(os.path.realpath(__file__))

    # Build the path to the XML file relative to the launch folder
    xmlScenePath = os.path.join(launch_dir, '..', 'mjcf', 'Bipedal_Robot.xml')
    xmlScenePath = os.path.realpath(xmlScenePath)

    if not os.path.exists(xmlScenePath):
        raise FileNotFoundError(f"Scene file does not exist: {xmlScenePath}.")

    mujoco_node = Node(
        package    = "biped_mujoco_ros2",
        executable = "mujoco_node",
        output     = "screen",
        parameters = [{"mujoco_xml_model": xmlScenePath}]
    )

    return LaunchDescription([mujoco_node])