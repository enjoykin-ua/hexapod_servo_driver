import math
import time
from Hexapod_object import Hexapod
from uart_communication import UARTCommunication
from kinematics import inverse_kinematics

class KinematicMovements:
    def __init__(self, uart_comm: UARTCommunication):
        self.uart_comm = uart_comm

    def go_to_init_position(self, hexapod: Hexapod):
        """Set each servo to its initial position based on the selected mode."""
        for leg_name, leg in hexapod.legs.items():
            for servo_name in ["coxa", "femur", "tibia"]:
                servo = getattr(leg, servo_name)
                
                # Set the target position to the initial position
                servo.set_position = servo.init_position
                
                # Calculate the pulse width for the initial position and convert to integer
                servo.calculated_pulse = int(
                    servo.min_pulse +
                    ((servo.init_position - servo.min_angle) / 
                    (servo.max_angle - servo.min_angle)) * 
                    (servo.max_pulse - servo.min_pulse)
                )
                
                # Print debug information
                print(f"{leg_name} {servo_name}: Init Position = {servo.init_position}°, Pulse = {servo.calculated_pulse} µs")

    def go_to_xyz_position(self, hexapod: Hexapod, xyz_positions: list, steps: int = 100, steps_interval_ms: int = 10):
        """Move the legs to specified XYZ coordinates using inverse kinematics."""
        # Convert XYZ coordinates to angles using inverse kinematics
        angle_array = []
        for leg_data in xyz_positions:
            leg_name, x, y, z = leg_data
            leg = hexapod.legs[leg_name]
            
            try:
                # Calculate angles using inverse kinematics
                theta1, theta2, theta3 = inverse_kinematics(leg, x, y, z)
                
                # Convert radians to degrees
                coxa_angle = math.degrees(theta1)
                femur_angle = math.degrees(theta2)
                tibia_angle = math.degrees(theta3)
                
                # Add to angle array
                angle_array.append([leg_name, coxa_angle, femur_angle, tibia_angle])
                print(f"{leg_name} IK result: Coxa={coxa_angle:.2f}°, Femur={femur_angle:.2f}°, Tibia={tibia_angle:.2f}°")
                
            except ValueError as e:
                print(f"Error calculating inverse kinematics for {leg_name}: {e}")
                continue
        
        # Move to calculated angles
        if angle_array:
            self.go_multiple_to_pose(hexapod, angle_array, steps, steps_interval_ms)
        else:
            print("No valid angles calculated, movement aborted.")

    def go_multiple_to_pose(self, hexapod: Hexapod, pose_array: list, steps: int = 100, steps_interval_ms: int = 10):
        """Move the Hexapod to a new pose with a smooth transition using sinusoidal interpolation.
        This version sends all servo commands in a single UART message."""
        for step in range(steps + 1):
            factor = 0.5 * (1 - math.cos(math.pi * step / steps))  # Sinusoidal interpolation factor
            print(f"Step {step}/{steps}, Interpolation Factor: {factor}")

            all_targets_reached = True  # Flag to check if all servos reached their targets
            max_position_diff = 0.1  # Maximum allowed difference to consider position reached

            # First, update all servo positions
            for pose in pose_array:
                leg_name, coxa_target, femur_target, tibia_target = pose
                leg = hexapod.legs[leg_name]

                # Update each servo's set position gradually
                for servo, target_angle, servo_name in zip([leg.coxa, leg.femur, leg.tibia], 
                                                         [coxa_target, femur_target, tibia_target], 
                                                         ["coxa", "femur", "tibia"]):
                    # Calculate new position
                    new_position = servo.actual_position + factor * (target_angle - servo.actual_position)
                    
                    # Only update if the position difference is significant
                    position_diff = abs(target_angle - servo.actual_position)
                    if position_diff > max_position_diff:
                        servo.set_position = new_position
                        servo.actual_position = new_position  # Update actual position
                        # Calculate the corresponding pulse width
                        servo.calculated_pulse = int(
                            servo.min_pulse +
                            ((new_position - servo.min_angle) /
                            (servo.max_angle - servo.min_angle)) *
                            (servo.max_pulse - servo.min_pulse)
                        )
                        servo.servo_reached_position = False  # Reset flag as servo needs to move
                        all_targets_reached = False
                    else:
                        # If target is reached, update actual position to target
                        servo.set_position = target_angle
                        servo.actual_position = target_angle
                        # Calculate final pulse width
                        final_pulse = int(
                            servo.min_pulse +
                            ((target_angle - servo.min_angle) /
                            (servo.max_angle - servo.min_angle)) *
                            (servo.max_pulse - servo.min_pulse)
                        )
                        # Only set flag to True if we've sent the final pulse
                        if servo.calculated_pulse == final_pulse:
                            servo.servo_reached_position = True
                        else:
                            servo.calculated_pulse = final_pulse
                            servo.servo_reached_position = False
                            all_targets_reached = False

            # Generate and send command only if there are servos that need to move
            if not all_targets_reached:
                uart_command = self.uart_comm.generate_multiple_uart_command(hexapod)
                if uart_command:  # Only send if there are servos to move
                    # Create debug output showing servo pins and their pulse values
                    debug_output = []
                    for pose in pose_array:
                        leg_name, coxa_target, femur_target, tibia_target = pose
                        leg = hexapod.legs[leg_name]

                        for servo, target_angle, servo_name in zip([leg.coxa, leg.femur, leg.tibia], 
                                                                 [coxa_target, femur_target, tibia_target], 
                                                                 ["coxa", "femur", "tibia"]):
                            if not servo.servo_reached_position:  # Only show servos that haven't reached their target
                                # Calculate desired end pulse using target_angle
                                desired_pulse = int(
                                    servo.min_pulse +
                                    ((target_angle - servo.min_angle) /
                                    (servo.max_angle - servo.min_angle)) *
                                    (servo.max_pulse - servo.min_pulse)
                                )
                                debug_output.append(f"[{servo.pin}, {servo.calculated_pulse}, {desired_pulse}]")
                    
                    print("Sending UART Command:", " ".join(debug_output))
                    self.uart_comm.send_uart_command(uart_command)

            # If all targets are reached, we can skip the remaining steps
            if all_targets_reached:
                print(f"All targets reached at step {step}, skipping remaining steps")
                break

            time.sleep(steps_interval_ms / 1000)  # Wait for next step 