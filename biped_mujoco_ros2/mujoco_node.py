import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import mujoco
import numpy as np
from mujoco.viewer import launch_passive
import os

class mujoco_ros2_node(Node):

    def __init__(self):
        super().__init__('mujoco_ros2_node')
        self.declare_parameter('mujoco_xml_model','mjcf/Bipedal_Robot.xml')

        # Get the path to the MuJoCo XML model
        # This can be set via ROS parameters or defaults to 'mjcf/Bipedal_Robot.xml'    
        self.mujoco_xml_path = self.get_parameter('mujoco_xml_model').get_parameter_value().string_value
        
        # Get the path to the launch folder
        launch_dir = os.path.dirname(os.path.realpath(__file__))

        # Build the path to the XML file relative to the launch folder
        xmlScenePath = os.path.join(launch_dir, '..', 'mjcf', 'Bipedal_Robot.xml')
        xmlScenePath = os.path.realpath(xmlScenePath)
        self.mujoco_xml_path = xmlScenePath

        self.mj_model = None
        self.mj_data = None
        self.mj_time_step = 0.001 # default time step for MuJoCo

        self.get_logger().info(f"Loading MuJoCo model from: {self.mujoco_xml_path}")

        # Try to Load the MuJoCo model
        try:
            self.mj_model = mujoco.MjModel.from_xml_path(self.mujoco_xml_path)
            self.mj_data = mujoco.MjData(self.mj_model)
            self.mj_time_step = self.mj_model.opt.timestep # Get the time step from the model

        except Exception as e:
            self.get_logger().error(f"Failed to load MuJoCo model from {self.mujoco_xml_path}: {e}")
            return 
               
        # Initialize the MuJoCo viewer
        try:
            self.get_logger().info("Launching MuJoCo viewer...")
            launch_passive(self.mj_model, self.mj_data)
        except Exception as e:
            self.get_logger().error(f"Failed to launch MuJoCo viewer: {e}")
            return
        
        # Create a publisher for the node
        self.joint_state_publisher_ = self.create_publisher(String, 'joint_state', 10)

    def update(self):
        # Update the MuJoCo simulation
        mujoco.mj_step(self.mj_model, self.mj_data)
        self.viewer.sync()
        # Publish joint states
        #joint_states = String()
        #joint_states.data = ', '.join([f"{joint.name}: {joint.qpos[0]}" for joint in self.mj_model.joint])
        #self.joint_state_publisher_.publish(joint_states)

        #self.get_logger().info(f"Published joint states: {joint_states.data}")



def main(args=None):
    rclpy.init(args=args)

    mjc_ros2_node = mujoco_ros2_node()

    #run mujoco simulator and publish joint states
    mjc_ros2_node.viewer = launch_passive(mjc_ros2_node.mj_model, mjc_ros2_node.mj_data)
    
    while(mjc_ros2_node.viewer.is_running()):
        rclpy.spin(mjc_ros2_node)
        mjc_ros2_node.update()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mujoco_ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()