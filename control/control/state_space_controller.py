#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from scipy import linalg
import time

class StateSpaceController(Node):
    def __init__(self):
        super().__init__('state_space_controller')
        
        # System parameters from URDF
        self.M = 0.4015  # mass of cart
        self.m = 0.212   # mass of pendulum
        self.b = 0.1     # friction coefficient (from xacro)
        self.I = 0.0017534  # moment of inertia
        self.l = 0.19577  # length to pendulum center of mass
        self.g = 9.8     # gravity

        # Rail limits with safety margins (from xacro)
        self.rail_min = -0.43
        self.rail_max = 0.43

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
        
        # LQR weights - adjusted for better control
        Q = np.diag([20.0, 1.0, 20000.0, 2000.0])  # Increased position and angle weights
        R = np.array([[0.1]])  # Control effort weight
        
        # Compute LQR gain matrix
        self.K = self._compute_lqr_gain(Q, R)
        
        # Log system parameters
        self.get_logger().info(
            f'System Parameters:\n'
            f'Mass (cart): {self.M}\n'
            f'Mass (pendulum): {self.m}\n'
            f'Length: {self.l}\n'
            f'Friction: {self.b}\n'
            f'LQR Weights - Q: {Q.diagonal()}, R: {R[0,0]}'
        )
        
        # Control parameters
        self.current_position = 0.0
        self.target_position = 0.0
        self.last_pendulum_position = 0.0
        self.last_pendulum_velocity = 0.0
        self.last_direction = 1
        
        # Swing-up control parameters
        self.target_energy = self.m * self.g * self.l  # Energy needed for upright position
        self.swing_up_gain = 4.0  # Gain for energy-based swing-up
        self.swing_up_threshold = 0.15  # Threshold for switching to LQR
        self.is_swing_up = True  # Always start with swing-up control
        self.upright_position = np.pi  # Upright is at pi radians
        self.max_force = 5.0  # Allow larger cart movements
        self.pendulum_period = 2.0 * np.pi * np.sqrt(self.l / self.g)  # Natural period of pendulum
        self.movement_time = self.pendulum_period / 3.0  # Balanced movement time
        self.velocity_threshold = 3.0  # Increased velocity threshold
        self.energy_threshold = 0.03  # Balanced threshold for energy addition
        self.last_target = 0.0  # Track last target position for smooth transitions
        self.max_velocity = 0.8  # Maximum allowed velocity for smoothing
        self.timer_period = 0.05  # Timer period in seconds (matches create_timer)
        self.swing_up_start_time = time.time()  # Track swing-up start time
        self.swing_up_amplitude = 0.42  # Increased amplitude for more energy transfer (meters)
        
        # Transition parameters
        self.is_catch_phase = False
        self.catch_start_time = 0.0
        self.catch_duration = 1.0  # Increased catch duration
        self.catch_gain = 12.0  # Much more aggressive catch gain
        self.last_catch_velocity = 0.0
        self.catch_velocity_history = []
        
        # Add cart velocity tracking
        self.prev_position = 0.0
        self.last_cart_velocity = 0.0
        
        # For catch phase entry dwell
        self.catch_entry_dwell_time = 0.1  # seconds
        self.catch_entry_dwell_start = None
        
        # Create subscribers and publishers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile=rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )
            
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            qos_profile=rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )
            
        self.get_logger().info('State-space controller node initialized')
        
        # Create timer for sending commands
        self.command_timer = self.create_timer(0.05, self.send_trajectory_command)
        
        # Add to __init__:
        self.lqr_entry_time = None  # Track when LQR mode starts

    def calculate_pendulum_energy(self):
        """Calculate the total energy of the pendulum."""
        # Kinetic energy
        kinetic_energy = 0.5 * self.m * (self.l * self.last_pendulum_velocity)**2
        # Potential energy (relative to upright)
        potential_energy = self.m * self.g * self.l * (1 - np.cos(self.last_pendulum_position))
        return kinetic_energy + potential_energy

    def swing_up_control(self):
        """Energy-based swing-up control with improved stability."""
        # Calculate current energy
        current_energy = self.calculate_pendulum_energy()
        target_energy = self.m * self.g * self.l  # Energy needed for upright position
        
        # Calculate energy error
        energy_error = target_energy - current_energy
        
        # Calculate pendulum phase
        phase = np.arctan2(self.last_pendulum_velocity, self.last_pendulum_position)
        
        # Adaptive gain based on energy error
        energy_gain = 2.0 * (1.0 + abs(energy_error) / target_energy)
        
        # Calculate control input using energy-based control law
        control_input = energy_gain * np.sign(self.last_pendulum_velocity * np.cos(self.last_pendulum_position))
        
        # Calculate target position
        center = (self.rail_min + self.rail_max) / 2.0
        safety_margin = 0.05  # Increased safety margin
        
        # Adaptive amplitude based on energy and position
        base_amplitude = 0.35  # Increased base amplitude
        energy_factor = min(1.0, abs(energy_error) / target_energy)
        position_factor = 1.0 - min(1.0, abs(self.current_position - center) / (self.rail_max - center))
        
        # Calculate maximum allowed amplitude
        max_amplitude = min(0.45, (self.rail_max - self.rail_min) / 2 - safety_margin)
        
        # Combine factors for final amplitude
        amplitude = base_amplitude + (max_amplitude - base_amplitude) * (energy_factor * 0.7 + position_factor * 0.3)
        
        # Calculate target position with smooth transitions
        target = center + control_input * amplitude
        
        # Add damping when close to upright
        if abs(self.last_pendulum_position) < np.pi/4:  # 45 degrees
            damping_factor = 0.5 * (1.0 - abs(self.last_pendulum_position) / (np.pi/4))
            target = center + (target - center) * (1.0 - damping_factor)
        
        # Ensure target stays within limits with smooth constraints
        target = np.clip(target, self.rail_min + safety_margin, self.rail_max - safety_margin)
        
        # Log control parameters for debugging
        self.get_logger().info(
            f'Swing-up: Energy={current_energy:.3f}/{target_energy:.3f}, '
            f'Phase={phase:.3f}, Gain={energy_gain:.3f}, '
            f'Amplitude={amplitude:.3f}, Target={target:.3f}'
        )
        
        return target

    def lqr_control(self):
        """Implement LQR controller for stabilization."""
        # Calculate center position
        center = (self.rail_min + self.rail_max) / 2.0
        
        # Update cart velocity estimate
        self.last_cart_velocity = (self.current_position - self.prev_position) / self.timer_period
        self.prev_position = self.current_position
        
        # Create state vector with actual cart velocity
        state = np.array([
            self.current_position,
            self.last_cart_velocity,  # Use actual cart velocity
            self.last_pendulum_position,
            self.last_pendulum_velocity
        ])
        
        # Calculate control input
        max_control_step = 0.1  # Maximum position change per cycle
        control = float(np.clip(-self.K @ state, -max_control_step, max_control_step))
        
        # Calculate new target position
        new_target = float(np.clip(self.current_position + control, self.rail_min, self.rail_max))
        
        # Log LQR control details
        self.get_logger().info(
            f'LQR Control:\n'
            f'State: pos={state[0]:.3f}, vel={state[1]:.3f}, theta={state[2]:.3f}, theta_dot={state[3]:.3f}\n'
            f'Control input: {control:.3f}\n'
            f'Target position: {new_target:.3f}'
        )
        
        return new_target

    def catch_control(self):
        """Implement catch controller for transition phase."""
        # Calculate time in catch phase
        catch_time = time.time() - self.catch_start_time
        
        # Calculate target position using catch controller
        center = (self.rail_min + self.rail_max) / 2.0
        angle_error = self.last_pendulum_position
        velocity_error = self.last_pendulum_velocity
        
        # Store velocity history
        self.catch_velocity_history.append(abs(self.last_pendulum_velocity))
        if len(self.catch_velocity_history) > 10:
            self.catch_velocity_history.pop(0)
        
        # Calculate velocity trend
        velocity_trend = 0
        if len(self.catch_velocity_history) > 1:
            velocity_trend = self.catch_velocity_history[-1] - self.catch_velocity_history[0]
        
        # Bang-bang term for high velocity
        bang_bang = 0.0
        bang_bang_threshold = 4.0  # rad/s
        if abs(self.last_pendulum_velocity) > bang_bang_threshold:
            bang_bang = -np.sign(self.last_pendulum_velocity) * 0.25  # meters, adjust as needed
        
        # Stronger PD controller for catch phase
        control_input = -self.catch_gain * (angle_error + 0.5 * velocity_error)
        
        # Add velocity-based damping
        damping = -0.3 * self.last_pendulum_velocity
        
        # Add position correction
        position_correction = -0.5 * (self.current_position - center)
        
        # Combine all control terms
        target = center + control_input + damping + position_correction + bang_bang
        
        # Ensure target stays within limits
        safety_margin = 0.05
        target = max(self.rail_min + safety_margin, min(self.rail_max - safety_margin, target))
        
        # Log catch control parameters
        self.get_logger().info(
            f'Catch Control:\n'
            f'Time in catch: {catch_time:.3f}\n'
            f'Angle error: {angle_error:.3f}\n'
            f'Velocity error: {velocity_error:.3f}\n'
            f'Velocity trend: {velocity_trend:.3f}\n'
            f'Control input: {control_input:.3f}\n'
            f'Damping: {damping:.3f}\n'
            f'Bang-bang: {bang_bang:.3f}\n'
            f'Position correction: {position_correction:.3f}\n'
            f'Target: {target:.3f}'
        )
        
        self.last_catch_velocity = abs(self.last_pendulum_velocity)
        return target

    def send_trajectory_command(self):
        """Send a trajectory command to move the cart."""
        try:
            # Create trajectory message
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ['corredera']
            
            # Calculate distance from upright position
            distance_from_upright = abs(self.last_pendulum_position - self.upright_position)
            
            # Calculate current energy and target energy
            current_energy = self.calculate_pendulum_energy()
            target_energy = self.m * self.g * self.l
            
            # Choose control strategy based on pendulum state
            if self.is_swing_up:
                # Reset swing-up timer if just entered swing-up mode
                if not hasattr(self, '_was_swing_up') or not self._was_swing_up:
                    self.swing_up_start_time = time.time()
                    self.catch_velocity_history = []
                self._was_swing_up = True
                
                # Enhanced transition conditions
                energy_ratio = current_energy / target_energy
                
                # Tightened catch phase entry condition
                catch_angle_threshold = 0.8  # radians (~45 deg, increased)
                cart_position_threshold = 0.1  # meters (adjust as needed)
                catch_velocity_threshold = 8.0  # rad/s (increased)
                center = (self.rail_min + self.rail_max) / 2.0
                catch_conditions_met = (
                    distance_from_upright < catch_angle_threshold and
                    abs(self.current_position - center) < cart_position_threshold and
                    energy_ratio > 0.95 and
                    abs(self.last_pendulum_velocity) < catch_velocity_threshold
                )
                if catch_conditions_met:
                    if self.catch_entry_dwell_start is None:
                        self.catch_entry_dwell_start = time.time()
                    elif time.time() - self.catch_entry_dwell_start > self.catch_entry_dwell_time:
                        self.is_catch_phase = True
                        self.is_swing_up = False  # Ensure swing-up mode is exited
                        self.catch_start_time = time.time()
                        self.catch_velocity_history = []
                        self.catch_entry_dwell_start = None
                        self.get_logger().info(
                            f'Entering catch phase - Conditions met (with dwell):\n'
                            f'Distance from upright: {distance_from_upright:.3f} < {catch_angle_threshold}\n'
                            f'Cart position: {self.current_position:.3f} (center: {center:.3f}, threshold: {cart_position_threshold})\n'
                            f'Energy ratio: {energy_ratio:.3f} > 0.95\n'
                            f'Velocity: {self.last_pendulum_velocity:.3f} < {catch_velocity_threshold}'
                        )
                else:
                    self.catch_entry_dwell_start = None
                
                # Log all catch entry conditions every cycle with True/False
                angle_ok = distance_from_upright < catch_angle_threshold
                cart_ok = abs(self.current_position - center) < cart_position_threshold
                energy_ok = energy_ratio > 0.95
                velocity_ok = abs(self.last_pendulum_velocity) < catch_velocity_threshold
                self.get_logger().info(
                    f'[CATCH ENTRY CHECK] Angle: {distance_from_upright:.3f} < {catch_angle_threshold} ({angle_ok}), '
                    f'Cart pos: {abs(self.current_position - center):.3f} < {cart_position_threshold} ({cart_ok}), '
                    f'Energy ratio: {energy_ratio:.3f} > 0.95 ({energy_ok}), '
                    f'Velocity: {abs(self.last_pendulum_velocity):.3f} < {catch_velocity_threshold} ({velocity_ok}), '
                    f'Dwell: {self.catch_entry_dwell_start is not None}'
                )
                
                # Use swing-up control
                new_target = self.swing_up_control()
            elif self.is_catch_phase:
                # Check if catch phase is complete
                catch_time = time.time() - self.catch_start_time
                velocity_decreasing = len(self.catch_velocity_history) > 5 and self.catch_velocity_history[-1] < self.catch_velocity_history[0]
                
                # More lenient transition conditions
                max_catch_duration = 4.0  # seconds (increased)
                velocity_threshold = 4.0  # rad/s
                velocity_decreasing_threshold = 4.0  # rad/s

                # Log all relevant variables for debugging
                self.get_logger().info(
                    f'[CATCH PHASE DEBUG] catch_time={catch_time:.3f}, max_catch_duration={max_catch_duration}, '
                    f'last_pendulum_velocity={self.last_pendulum_velocity:.3f}, '
                    f'velocity_decreasing={velocity_decreasing}, '
                    f'catch_duration={self.catch_duration}, '
                    f'velocity_threshold={velocity_threshold}, '
                    f'velocity_decreasing_threshold={velocity_decreasing_threshold}'
                )
                
                if (catch_time > self.catch_duration and 
                    (abs(self.last_pendulum_velocity) < velocity_threshold or  # Increased velocity threshold
                     (velocity_decreasing and abs(self.last_pendulum_velocity) < velocity_decreasing_threshold))):
                    self.is_catch_phase = False
                    self.is_swing_up = False
                    self.get_logger().info(
                        f'[CATCH PHASE EXIT] Catch phase complete, switching to LQR control:\n'
                        f'Final velocity: {abs(self.last_pendulum_velocity):.3f}\n'
                        f'Velocity decreasing: {velocity_decreasing}\n'
                        f'Time in catch: {catch_time:.3f}'
                    )
                elif catch_time > max_catch_duration:
                    self.is_catch_phase = False
                    self.is_swing_up = False
                    self.get_logger().info(
                        f'[CATCH PHASE EXIT] Catch phase forced to LQR due to max duration ({max_catch_duration}s) reached.\n'
                        f'Final velocity: {abs(self.last_pendulum_velocity):.3f}\n'
                        f'Time in catch: {catch_time:.3f}'
                    )
                else:
                    # Add logging to clarify why not switching
                    self.get_logger().info(
                        f'[CATCH PHASE HOLD] Catch phase ongoing. Not switching to LQR yet.\n'
                        f'Current velocity: {abs(self.last_pendulum_velocity):.3f} (threshold: {velocity_threshold})\n'
                        f'Velocity decreasing: {velocity_decreasing}\n'
                        f'Time in catch: {catch_time:.3f} (max: {max_catch_duration})'
                    )
                
                # Use catch controller
                new_target = self.catch_control()
            else:
                self._was_swing_up = False
                # Use LQR control
                if self.lqr_entry_time is None:
                    self.lqr_entry_time = time.time()
                new_target = self.lqr_control()
                
                # Enhanced switch back to swing-up conditions
                min_lqr_time = 0.5  # seconds
                lqr_time = time.time() - self.lqr_entry_time if self.lqr_entry_time else 0.0
                
                # Tightened LQR entry conditions
                lqr_angle_threshold = 0.2  # radians (~11 deg)
                lqr_velocity_threshold = 2.0  # rad/s
                center = (self.rail_min + self.rail_max) / 2.0
                
                if (lqr_time > min_lqr_time and (
                    distance_from_upright > 1.57 or  # More than 90 degrees
                    abs(self.last_pendulum_velocity) > 8.0 or  # Too fast
                    current_energy / target_energy < 0.5 or  # Too little energy
                    abs(self.current_position - center) > 0.1  # Too far from center
                )):
                    self.is_swing_up = True
                    self.is_catch_phase = False
                    self.lqr_entry_time = None
                    self.get_logger().info(
                        f'Switching to swing-up control - Conditions met:\n'
                        f'Distance from upright: {distance_from_upright:.3f} > 1.57\n'
                        f'Velocity: {abs(self.last_pendulum_velocity):.3f} > 8.0\n'
                        f'Energy ratio: {current_energy/target_energy:.3f} < 0.5\n'
                        f'LQR time: {lqr_time:.3f} > {min_lqr_time}\n'
                        f'Cart position: {abs(self.current_position - center):.3f} > 0.1'
                    )
            
            # Smooth target transition
            max_step = self.max_velocity * self.movement_time
            self.target_position = self.last_target + np.clip(
                new_target - self.last_target,
                -max_step,
                max_step
            )
            self.last_target = self.target_position
            
            # Create trajectory points with smooth acceleration
            points = []
            
            # Calculate movement parameters
            total_distance = self.target_position - self.current_position
            total_time = self.movement_time
            
            # Point 1: Start from current position
            point1 = JointTrajectoryPoint()
            point1.positions = [self.current_position]
            point1.velocities = [0.0]
            point1.accelerations = [0.0]
            point1.time_from_start.sec = 0
            point1.time_from_start.nanosec = 0
            points.append(point1)
            
            # Point 2: Acceleration phase
            point2 = JointTrajectoryPoint()
            accel_time = total_time * 0.3  # 30% of time for acceleration
            point2.positions = [self.current_position + total_distance * 0.3]
            point2.velocities = [total_distance / total_time * 0.7]  # 70% of average velocity
            point2.accelerations = [total_distance / (accel_time * accel_time)]  # Constant acceleration
            point2.time_from_start.sec = 0
            point2.time_from_start.nanosec = int(accel_time * 1e9)
            points.append(point2)
            
            # Point 3: Deceleration phase
            point3 = JointTrajectoryPoint()
            decel_time = total_time * 0.3  # 30% of time for deceleration
            point3.positions = [self.current_position + total_distance * 0.7]
            point3.velocities = [total_distance / total_time * 0.7]  # 70% of average velocity
            point3.accelerations = [-total_distance / (decel_time * decel_time)]  # Constant deceleration
            point3.time_from_start.sec = 0
            point3.time_from_start.nanosec = int((total_time - decel_time) * 1e9)
            points.append(point3)
            
            # Point 4: End of movement
            point4 = JointTrajectoryPoint()
            point4.positions = [self.target_position]
            point4.velocities = [0.0]
            point4.accelerations = [0.0]
            point4.time_from_start.sec = 0
            point4.time_from_start.nanosec = int(total_time * 1e9)
            points.append(point4)
            
            trajectory_msg.points = points
            
            # Publish the message
            self.trajectory_pub.publish(trajectory_msg)
            
            self.get_logger().info(
                f'Mode: {"Swing-up" if self.is_swing_up else "Catch" if self.is_catch_phase else "LQR"}, '
                f'Pendulum: pos={self.last_pendulum_position:.3f}, vel={self.last_pendulum_velocity:.3f}, '
                f'Distance from upright: {distance_from_upright:.3f}, '
                f'Target Energy: {self.target_energy:.3f}, '
                f'Cart: pos={self.current_position:.3f}, target={self.target_position:.3f}'
            )
            
            if self.is_swing_up or self.is_catch_phase:
                self.lqr_entry_time = None
            
        except Exception as e:
            self.get_logger().error(f'Error sending trajectory command: {str(e)}')

    def _compute_lqr_gain(self, Q, R):
        """Compute the LQR gain matrix using the algebraic Riccati equation."""
        try:
            # Adjust Q matrix to be less aggressive
            Q = np.diag([1.0, 1.0, 10.0, 1.0])  # Reduced weights for position and velocity
            R = np.array([[0.1]])  # Increased control cost
            
            P = linalg.solve_continuous_are(self.A, self.B, Q, R)
            K = np.linalg.inv(R) @ self.B.T @ P
            
            # Log LQR gain matrix
            self.get_logger().info(f'LQR Gain Matrix: {K}')
            
            return K
        except Exception as e:
            self.get_logger().error(f'Error computing LQR gain: {e}')
            return np.array([[0.0, 0.0, 0.0, 0.0]])

    def joint_state_callback(self, msg):
        """Callback function for joint state updates."""
        try:
            # Find indices for our joints
            corredera_idx = msg.name.index('corredera')
            pendulum_idx = msg.name.index('pendulum_joint')
            
            # Extract position and velocity from joint states
            self.current_position = msg.position[corredera_idx]
            self.last_pendulum_position = msg.position[pendulum_idx]
            self.last_pendulum_velocity = msg.velocity[pendulum_idx]
            
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