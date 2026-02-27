#!/usr/bin/env python3
import genesis as gs
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from ament_index_python.packages import get_package_share_path
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

class GenesisRosBridge(Node):
    def __init__(self):
        super().__init__('genesis_ros_bridge')
        
        description_path = get_package_share_path("rover_description")
        robot_description_content = str(description_path) + "/mujoco/RRCBot.xml"

        
        # --- Genesis Setup ---
        gs.init(backend=gs.gpu)
        self.scene = gs.Scene(show_viewer=True)
        self.plane = self.scene.add_entity(gs.morphs.Plane())
        self.robot = self.scene.add_entity(gs.morphs.MJCF(
            file=robot_description_content, 
            pos=[0, 0, 1.0]))
        self.scene.build()
        
        self.jnt_names = [
            'R_wheel_diff',
            'R_wheel_rocker',
            'L_wheel_diff',
            'L_wheel_rocker',
            'arm_gantry',
            'l0',
            'l1',
        ]
        self.dofs_idx = [self.robot.get_joint(name).dof_idx_local for name in self.jnt_names]
        self.chassis_max_vel = 5.0
        self.l0_max_pos = 1.0
        self.l1_max_pos = 1.0
        self.gantry_max_pos = 1.0


        # --- Variables to store input state ---
        self.left_stick_pos = [0.0, 0.0]  # [x, y]
        self.right_stick_pos = [0.0, 0.0]  # [x, y]
        self.buttons = {'A': 0, 'B': 0}

        # --- ROS 2 Pubs/Subs ---
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscriber to the /joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg: Joy):
        """
        Standard Xbox/PS4 Mapping:
        Axes: [0] Left Stick X, [1] Left Stick Y
        Buttons: [0] A (or Cross), [1] B (or Circle)
        """
        # Store Joystick X and Y
        if len(msg.axes) >= 4:
            self.left_stick_pos = [msg.axes[0], msg.axes[1]]
            self.right_stick_pos = [msg.axes[2], msg.axes[3]]

        # Store A and B buttons
        if len(msg.buttons) >= 2:
            self.buttons['A'] = msg.buttons[0]
            self.buttons['B'] = msg.buttons[1]
        
        # Logging for verification (optional)
        self.get_logger().info(f"L_Stick: {self.left_stick_pos} | R_Stick: {self.right_stick_pos} | A: {self.buttons['A']} B: {self.buttons['B']}")

    def run(self):
        while rclpy.ok():
            # Apply the stored inputs to Genesis control here
            # Example: self.robot.set_dofs_velocity(self.stick_pos[1] * 10.0)
            
            self.scene.step()
            self.publish_states() # send genesis info out to ros2
            self.send_cmds() # send ros2 commands to genesis
            rclpy.spin_once(self, timeout_sec=0)

    def publish_states(self):
        pass
    
    def send_cmds(self):
        if self.buttons['A'] != 0:
            gantry_pos = self.left_stick_pos[0] * self.gantry_max_pos
            l0_pos = self.left_stick_pos[1] * self.l0_max_pos 
            l1_pos = self.right_stick_pos[1] * self.l1_max_pos 
            
            pos_inputs = np.array([gantry_pos, l0_pos, l1_pos])

            self.robot.control_dofs_position(
                pos_inputs,
                self.dofs_idx[4:7],
            )
        else:
            right_vel = -1 * self.right_stick_pos[1] * self.chassis_max_vel
            left_vel = self.left_stick_pos[1] * self.chassis_max_vel
            
            vels_input = np.array([right_vel, right_vel, left_vel, left_vel])
            self.robot.control_dofs_velocity(
                vels_input,
                self.dofs_idx[:4],
            )


def main():
    rclpy.init()
    bridge = GenesisRosBridge()
    try:
        bridge.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# hf = np.zeros((40, 40), dtype=np.int16)
# hf[10:30, 10:30] = 200 * np.hanning(20)[:, None] * np.hanning(20)[None, :]

# horizontal_scale = 0.25  # metres between grid points
# vertical_scale   = 0.005  # metres per height-field unit

# 4. add the terrain entity
# plane = scene.add_entity(gs.morphs.Plane())
# scene.add_entity(
#     morph=gs.morphs.Terrain(
#         height_field=hf,
#         horizontal_scale=horizontal_scale,
#         vertical_scale=vertical_scale,
#     ),
# )