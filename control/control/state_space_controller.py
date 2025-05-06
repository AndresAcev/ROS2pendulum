#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from scipy import linalg
import time

class StateSpaceController(Node):
    def __init__(self):
        super().__init__('state_space_controller')
        
        # System parameters (these should be tuned based on your specific system)
        self.M = 0.5  # mass of cart
        self.m = 0.2  # mass of pendulum
        self.b = 0.1  # friction coefficient
        self.I = 0.006  # moment of inertia
        self.l = 0.3   # length to pendulum center of mass
        self.g = 9.8   # gravity

        # State-space matrices
        self.A = np.array([
            [0, 1, 0, 0],
            [0, -self.b/self.M, -self.m*self.g/self.M, 0],
            [0, 0, 0, 1],
            [0, -self.b/(self.M*self.l), -(self.M+self.m)*self.g/(self.M*self.l), 0]
        ])
        
        self.B = np.array([
            [0],
            [1/self.M],
            [0],
            [1/(self.M*self.l)]
        ])
        
        self.C = np.array([[1, 0, 0, 0],
                          [0, 0, 1, 0]])
        
        # LQR weights - much more aggressive control
        Q = np.diag([1.0, 1.0, 1000.0, 1000.0])  # Even more aggressive weights for pendulum states
        R = np.array([[0.001]])                  # Further reduced control cost for more aggressive control
        
        # Compute LQR gain matrix
        self.K = self._compute_lqr_gain(Q, R)
        
        # Initialize state vector
        self.state = np.zeros(4)
        self.current_position = 0.0
        self.target_position = 1.0  # Start with a target of 1 meter
        
        # Control parameters
        self.start_time = time.time()
        self.initial_disturbance = 2.0  # Much larger initial disturbance
        self.disturbance_duration = 3.0  # Longer duration for initial disturbance
        self.message_count = 0
        self.last_publish_time = time.time()
        
        # Create subscribers and publishers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
            
        # Create trajectory command publisher
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)
            
        self.get_logger().info('State-space controller node initialized')
        self.get_logger().info(f'LQR gain matrix: {self.K}')
        self.get_logger().info(f'System parameters: M={self.M}, m={self.m}, l={self.l}')
        
        # Print topic information
        self.get_logger().info('Subscribing to /joint_states')
        self.get_logger().info('Publishing to /joint_trajectory_controller/joint_trajectory')
        
        # Create a timer to periodically check if we're receiving joint states
        self.timer = self.create_timer(1.0, self.check_joint_states)
        
        # Send initial command
        self.send_trajectory_command()

    def check_joint_states(self):
        """Periodically check if we're receiving joint states."""
        if self.message_count == 0:
            self.get_logger().warn('No joint states received yet!')
        else:
            self.get_logger().info(f'Received {self.message_count} joint state messages')
            
        # Check if we're publishing commands
        time_since_last_publish = time.time() - self.last_publish_time
        if time_since_last_publish > 1.0:
            self.get_logger().warn(f'No commands published in {time_since_last_publish:.1f} seconds!')
            self.send_trajectory_command()  # Resend command if no recent publishes

    def send_trajectory_command(self):
        """Send a trajectory command to move the cart."""
        try:
            # Create trajectory message
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ['corredera']
            
            # Create multiple trajectory points for smooth motion
            points = []
            
            # Point 1: Start from current position
            point1 = JointTrajectoryPoint()
            point1.positions = [self.current_position]
            point1.velocities = [0.0]  # Start with zero velocity
            point1.accelerations = [0.0]
            point1.time_from_start.sec = 0
            point1.time_from_start.nanosec = 0
            points.append(point1)
            
            # Point 2: Move to target position
            point2 = JointTrajectoryPoint()
            point2.positions = [self.target_position]
            point2.velocities = [0.5]  # Constant velocity during movement
            point2.accelerations = [0.0]
            point2.time_from_start.sec = 0
            point2.time_from_start.nanosec = 500000000  # 0.5 seconds
            points.append(point2)
            
            # Point 3: End with zero velocity
            point3 = JointTrajectoryPoint()
            point3.positions = [self.target_position]
            point3.velocities = [0.0]  # End with zero velocity
            point3.accelerations = [0.0]
            point3.time_from_start.sec = 1
            point3.time_from_start.nanosec = 0  # 1 second total
            points.append(point3)
            
            trajectory_msg.points = points
            
            # Publish the message
            self.trajectory_pub.publish(trajectory_msg)
            self.last_publish_time = time.time()
            
            self.get_logger().info(f'Sent trajectory command: current={self.current_position:.3f}, target={self.target_position:.3f}')
            
            # Update target position for next command
            self.target_position = -self.target_position  # Alternate between 1.0 and -1.0
            
        except Exception as e:
            self.get_logger().error(f'Error sending trajectory command: {str(e)}')

    def _compute_lqr_gain(self, Q, R):
        """Compute the LQR gain matrix using the algebraic Riccati equation."""
        P = linalg.solve_continuous_are(self.A, self.B, Q, R)
        K = np.linalg.inv(R) @ self.B.T @ P
        return K

    def joint_state_callback(self, msg):
        """Callback function for joint state updates."""
        try:
            self.message_count += 1
            
            # Find indices for our joints
            corredera_idx = msg.name.index('corredera')
            pendulum_idx = msg.name.index('pendulum_joint')
            
            # Extract position and velocity from joint states
            cart_pos = msg.position[corredera_idx]
            cart_vel = msg.velocity[corredera_idx]
            pendulum_pos = msg.position[pendulum_idx]
            pendulum_vel = msg.velocity[pendulum_idx]
            
            # Update current position
            self.current_position = cart_pos
            
            # Log the current state
            if self.message_count % 10 == 0:  # Log every 10th message
                self.get_logger().info(
                    f'Message #{self.message_count}, '
                    f'Cart: pos={cart_pos:.6f}, vel={cart_vel:.6f}, '
                    f'Pendulum: pos={pendulum_pos:.6f}, vel={pendulum_vel:.6f}'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error in joint state callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    controller = StateSpaceController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 