import os
import sys
try:
    from franky import JointMotion, JointVelocityMotion, Robot, ReferenceType, Duration
except ImportError as e:
    print(f"Error importing franky: {e}")
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from collections import deque

class FrankyJointPositionController(Node):
    def __init__(self):
        super().__init__('FrankyJointPositionController')
        self.get_logger().info('Franky joint position control node initialized')
        
        # Declare parameters for controller gains
        self.declare_parameter('velocity_scaling', 0.5)
        self.declare_parameter('dynamics_factor', 0.01)
        self.declare_parameter('motion_duration_ms', 100)  # Increased for smoother motion
        self.declare_parameter('update_rate_hz', 20.0)      # Much slower for stability
        self.declare_parameter('robot_ip', '192.168.0.100')
        self.declare_parameter('joint_state_topic', '/joint_states')
        
        # Get parameter values
        self.velocity_scaling = self.get_parameter('velocity_scaling').get_parameter_value().double_value
        self.dynamics_factor = self.get_parameter('dynamics_factor').get_parameter_value().double_value
        self.motion_duration_ms = self.get_parameter('motion_duration_ms').get_parameter_value().integer_value
        self.update_rate_hz = self.get_parameter('update_rate_hz').get_parameter_value().double_value
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self.get_logger().info(f'Controller gains: velocity_scaling={self.velocity_scaling}, dynamics_factor={self.dynamics_factor}, duration={self.motion_duration_ms}ms, rate={self.update_rate_hz}Hz')
        
        # Store latest joint state data
        # self.latest_joint_states = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        self.joint_states_updated = False
        
        # Velocity smoothing buffers
        self.velocity_history = deque(maxlen=5)  # Keep last 5 velocity commands
        self.last_velocity_command = np.zeros(7)
        self.max_velocity_change = 1.0  # Very conservative change limit
        self.command_count = 0
        
        # Subscribe to joint states topic
        self.joint_sub = self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 1)
        
        # Create timer with configurable update rate
        self.timer = self.create_timer(1.0/self.update_rate_hz, self.timer_callback)

        self.robot = Robot(self.robot_ip)  # Replace this with your robot's IP
        self.robot.relative_dynamics_factor = self.dynamics_factor
        self.robot.recover_from_errors()

        # print(f'Moving to home position: {self.latest_joint_states}')
        # home_motion = JointMotion(self.latest_joint_states)
        # self.robot.move(home_motion)


        # Set joint limits for more aggressive movement

        print("joint velocity limit: ", self.robot.joint_velocity_limit)
        print("joint acceleration limit: ", self.robot.joint_acceleration_limit)
        print("joint jerk limit: ", self.robot.joint_jerk_limit)

    def joint_state_callback(self, msg):
        try:
            self.latest_joint_states = np.array(msg.position[:7])
            # temp = self.latest_joint_states[1]
            # self.latest_joint_states[1] = self.latest_joint_states[2]
            # self.latest_joint_states[2] = temp
            self.joint_states_updated = True
        except Exception as e:
            self.get_logger().error(f'Error in joint_velocity_callback: {e}')

    def smooth_velocity_command(self, raw_velocity):
        """Apply aggressive smoothing to completely eliminate discontinuities"""
        # Step 1: Very conservative velocity change limiting
        velocity_diff = raw_velocity - self.last_velocity_command
        max_change = np.full(7, self.max_velocity_change)
        
        # Clamp velocity changes very aggressively
        clamped_diff = np.clip(velocity_diff, -max_change, max_change)
        smoothed_velocity = self.last_velocity_command + clamped_diff
        
        # Step 2: Apply heavy smoothing with larger history
        self.velocity_history.append(smoothed_velocity)
        
        # Use simple average for maximum smoothing
        if len(self.velocity_history) >= 3:
            smoothed_velocity = np.mean(list(self.velocity_history), axis=0)
        
        smoothed_velocity = np.clip(smoothed_velocity, -self.max_velocity_change, self.max_velocity_change)
        
        # Update the last command
        self.last_velocity_command = smoothed_velocity
        return smoothed_velocity

    def timer_callback(self):
        if not self.joint_states_updated:
            return
        
        # Skip every other command to reduce frequency even more
        self.command_count += 1
        if self.command_count % 2 != 0:
            return
            
        try:
            current_joints = np.array(self.robot.current_joint_state.position[:7])
            raw_diff = (self.latest_joint_states - current_joints) * self.velocity_scaling
            
            # Apply aggressive smoothing
            smoothed_diff = self.smooth_velocity_command(raw_diff)
            
            # Skip very small movements
            if np.linalg.norm(smoothed_diff) < 0.005:  # Even smaller threshold
                self.joint_states_updated = False
                return
            
            motion = JointVelocityMotion(
                smoothed_diff, duration=Duration(self.motion_duration_ms),
            )
            self.robot.move(motion, asynchronous=True)
            self.joint_states_updated = False
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {e}')
            self.robot.recover_from_errors()


def main(args=None):
    rclpy.init(args=args)
    franky_joint_position_controller = FrankyJointPositionController()
    rclpy.spin(franky_joint_position_controller)
    franky_joint_position_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()