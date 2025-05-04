#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np
import math

class PendulumSimulator(Node):
    def __init__(self):
        super().__init__('pendulum_simulator')
        
        # Physical parameters
        self.g = 9.81  # gravity
        self.l = 0.4   # pendulum length (approximate)
        self.m = 0.057921  # pendulum mass (from URDF)
        self.dt = 0.01  # simulation time step
        
        # State variables
        self.theta = 0.0  # pendulum angle
        self.theta_dot = 0.0  # pendulum angular velocity
        self.x = 0.0  # cart position
        self.x_dot = 0.0  # cart velocity
        
        # Create publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cart_pos_sub = self.create_subscription(
            Float64, 
            '/cart_position', 
            self.cart_position_callback, 
            10)
        
        # Create timer for simulation update
        self.timer = self.create_timer(self.dt, self.update_simulation)
        
        self.get_logger().info('Pendulum simulator started')

    def cart_position_callback(self, msg):
        # Update desired cart position
        self.x = msg.data

    def update_simulation(self):
        # Calculate pendulum dynamics
        theta_ddot = (self.g/self.l) * math.sin(self.theta)
        
        # Update state using Euler integration
        self.theta_dot += theta_ddot * self.dt
        self.theta += self.theta_dot * self.dt
        
        # Add some damping
        self.theta_dot *= 0.99
        
        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['corredera', 'pendulo']
        joint_state.position = [self.x, self.theta]
        joint_state.velocity = [self.x_dot, self.theta_dot]
        self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    pendulum_simulator = PendulumSimulator()
    rclpy.spin(pendulum_simulator)
    pendulum_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 