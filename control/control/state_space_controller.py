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
        
        # System parameters from URDF
        self.M = 0.4015  # mass of cart (carro) from URDF
        self.m = 0.212  # mass of pendulum (pendulum_link) from URDF
        self.b = 0.7  # friction coefficient from Gazebo joint properties (increased for more damping)
        self.I = 0.0017534  # moment of inertia (ixx) of pendulum from URDF
        self.l = 0.19577  # length to pendulum center of mass from URDF (y coordinate of inertial origin)
        self.g = 9.8   # gravity

        # Rail limits from URDF with safety margins
        self.rail_min = -0.35  # Slightly less than URDF limit (-0.40204)
        self.rail_max = 0.30   # Slightly less than URDF limit (0.37696)

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
        
        # LQR weights - less aggressive for stability
        Q = np.diag([10.0, 1.0, 7000.0, 1000.0])  # Slightly less aggressive for stability
        R = np.array([[0.0001]])                  # Lower control cost for more aggressive control
        
        # Compute LQR gain matrix
        self.K = self._compute_lqr_gain(Q, R)
        
        # Initialize state vector
        self.state = np.zeros(4)
        self.current_position = 0.0
        self.target_position = 0.0  # Start at center
        self.swing_amplitude = 0.5  # Start with smaller amplitude
        self.swing_count = 0
        self.last_pendulum_position = 0.0
        self.last_pendulum_velocity = 0.0
        self.energy_threshold = 0.5  # Threshold for pendulum energy
        
        # Control parameters
        self.start_time = time.time()
        self.initial_disturbance = 3.0  # Even larger initial disturbance
        self.disturbance_duration = 2.0  # Shorter duration for faster response
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
        self.timer = self.create_timer(0.1, self.check_joint_states)  # Check every 0.1 seconds instead of 1.0
        
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
        """Send a trajectory command to move the cart based on pendulum position."""
        try:
            # Create trajectory message
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ['corredera']
            
            # Calculate target position based on pendulum state
            pendulum_energy = (self.last_pendulum_velocity ** 2) / 2 + self.g * self.l * (1 - np.cos(self.last_pendulum_position))
            
            # Determine cart movement based on pendulum state
            if abs(self.last_pendulum_position) < 0.1:  # Near vertical
                # If pendulum is near vertical, move cart to help maintain balance
                self.target_position = -0.2 * self.last_pendulum_velocity
            else:
                # If pendulum is swinging, move cart to help build energy
                # Move cart in the same direction as pendulum when it's moving away from vertical
                # This helps build momentum instead of canceling it
                if self.last_pendulum_velocity > 0:  # Pendulum moving right
                    self.target_position = 0.35  # Move cart right, larger amplitude
                else:  # Pendulum moving left
                    self.target_position = -0.35  # Move cart left, larger amplitude
            
            # Limit target position to rail limits with safety margins
            self.target_position = np.clip(self.target_position, self.rail_min, self.rail_max)
            
            # Add position margin check to prevent hitting limits
            position_margin = 0.05  # 5cm margin from limits
            if abs(self.target_position - self.rail_min) < position_margin:
                self.target_position = self.rail_min + position_margin
            elif abs(self.target_position - self.rail_max) < position_margin:
                self.target_position = self.rail_max - position_margin
            
            # Create trajectory points
            points = []
            
            # Point 1: Start from current position with high acceleration
            point1 = JointTrajectoryPoint()
            point1.positions = [self.current_position]
            point1.velocities = [0.0]
            point1.accelerations = [10.0]  # Lower initial acceleration
            point1.time_from_start.sec = 0
            point1.time_from_start.nanosec = 0
            points.append(point1)
            
            # Point 2: Move to target position with very high velocity
            point2 = JointTrajectoryPoint()
            point2.positions = [self.target_position]
            point2.velocities = [8.0]  # Lower velocity
            point2.accelerations = [0.0]
            point2.time_from_start.sec = 0
            point2.time_from_start.nanosec = 10000000  # 0.01 seconds for very fast movement
            points.append(point2)
            
            # Point 3: End with zero velocity
            point3 = JointTrajectoryPoint()
            point3.positions = [self.target_position]
            point3.velocities = [0.0]
            point3.accelerations = [-10.0]  # Lower deceleration
            point3.time_from_start.sec = 0
            point3.time_from_start.nanosec = 20000000  # 0.02 seconds total
            points.append(point3)
            
            trajectory_msg.points = points
            
            # Publish the message
            self.trajectory_pub.publish(trajectory_msg)
            self.last_publish_time = time.time()
            
            self.get_logger().info(
                f'Pendulum: pos={self.last_pendulum_position:.3f}, vel={self.last_pendulum_velocity:.3f}, '
                f'energy={pendulum_energy:.3f}, Cart: pos={self.current_position:.3f}, target={self.target_position:.3f}, '
                f'velocity=8.0, limits=[{self.rail_min:.3f}, {self.rail_max:.3f}]'
            )
            
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
            
            # Update current position and pendulum state
            self.current_position = cart_pos
            self.last_pendulum_position = pendulum_pos
            self.last_pendulum_velocity = pendulum_vel
            
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