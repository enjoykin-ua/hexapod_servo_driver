import struct
import serial
import time
from Hexapod_object import Hexapod

# Global UART variable
ser = None

def init():
    """Initialize the Hexapod by loading the configuration."""
    global hexapod
    hexapod = Hexapod("configuration.json", init_mode="hanging")
    print("Hexapod initialized.")

def init_uart():
    """Initialize the UART communication."""
    global ser
    SERIAL_PORT = "/dev/ttyACM0"  # USB-UART port
    BAUDRATE = 115200  # Baud rate
    TIMEOUT = 1  # UART timeout

    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
    time.sleep(2)  # Wait for connection stabilization
    print("UART initialized.")

def send_uart_command(command):
    """Sends a given command over UART."""
    global ser
    try:
        ser.write(command)
        ser.flush()
        print(f"Sent: {command}")
    except serial.SerialException as e:
        print(f"Error: {e}")


def go_to_init_position():
    """Set each servo to its initial position based on the selected mode."""
    for leg_name, leg in hexapod.legs.items():
        for servo_name in ["coxa", "femur", "tibia"]:
            servo = getattr(leg, servo_name)
            
            # Set the target position to the initial position
            servo.set_position = servo.init_position
            
            # Calculate the pulse width for the initial position and convert to integer
            servo.calculated_pulse = int(
                servo.min_pulse +
                ((servo.set_position - servo.min_angle) / 
                (servo.max_angle - servo.min_angle)) * 
                (servo.max_pulse - servo.min_pulse)
            )
            
            # Print debug information
            print(f"{leg_name} {servo_name}: Init Position = {servo.set_position}°, Pulse = {servo.calculated_pulse} µs")

def print_servo_commands():
    """Prints the pin and target pulse width for each servo."""
    for leg_name, leg in hexapod.legs.items():
        command_output = f"{leg_name}"
        for servo_name in ["coxa", "femur", "tibia"]:
            servo = getattr(leg, servo_name)
            command_output += f", {servo.pin}, {servo.calculated_pulse}"
        print(command_output)

def generate_uart_commands():
    """Generates UART command packets for setting servo pulse widths."""
    commands = []
    for leg_name, leg in hexapod.legs.items():
        for servo_name in ["coxa", "femur", "tibia"]:
            servo = getattr(leg, servo_name)
            
            # Convert pulse width to 4-byte Little-Endian format
            pulse_bytes = struct.pack("<f", servo.calculated_pulse)  # "<f" = Little-Endian Float
            
            # Construct UART command: Start (0x55), Opcode (0x06), Servo Index, Pulse Bytes, End (0xAA)
            command = bytes([0x55, 0x06, servo.pin]) + pulse_bytes + bytes([0xAA])
            commands.append(command)
    
    return commands

def generate_multiple_uart_command():
    """Generates a single UART command packet for setting multiple servo pulse widths."""
    command = bytearray()  # Use bytearray for better byte handling
    command.append(0x55)   # COMMAND_START
    command.append(0x07)   # CMD_SET_SERVO_PULSES

    # Collect all servo data
    servo_data = bytearray()
    for leg_name, leg in hexapod.legs.items():
        for servo_name in ["coxa", "femur", "tibia"]:
            servo = getattr(leg, servo_name)
            pulse_bytes = struct.pack("<f", servo.calculated_pulse)  # Convert float to bytes (Little-Endian)
            servo_data.append(servo.pin)  # Add servo pin
            servo_data.extend(pulse_bytes)  # Add 4-byte pulse width

    # Total number of servos (must be set AFTER counting them)
    total_servos = len(servo_data) // 5  # Each servo has 5 bytes (pin + 4 float)
    command.append(total_servos)  # Append servo count
    command.extend(servo_data)  # Append all servo data
    command.append(0xAA)  # COMMAND_END

    return bytes(command)

pose_hanging = [
    ["leg_1", 0, -78, 0],
    ["leg_3", 0, -75, 0],
    ["leg_4", -7, 80, 0],
    ["leg_6", 0, 80, 10]  
]

pose_1 = [
    ["leg_1", 0, 0, 30],  # leg_1: Coxa = 0°, Femur = 0°, Tibia = 30°
    ["leg_3", 0, 0, -30], # leg_3: Coxa = 0°, Femur = 0°, Tibia = -30°
    ["leg_4", 0, 0, -30], # leg_4: Coxa = 0°, Femur = 0°, Tibia = -30°
    ["leg_6", 0, 0, 30]   # leg_6: Coxa = 0°, Femur = 0°, Tibia = 30°
]

pose_01 = [
    ["leg_1", 45, 10, 90],  # leg_1: Coxa = 0°, Femur = 0°, Tibia = 30°
    ["leg_3", 30, 10, 90],  # leg_1: Coxa = 0°, Femur = 0°, Tibia = 30°
    ["leg_4", -30, 0, -90],  # leg_1: Coxa = 0°, Femur = 0°, Tibia = 30°
    ["leg_6", -45, 0, -90],  # leg_1: Coxa = 0°, Femur = 0°, Tibia = 30°
]

#this is first
pose_02 = [
    ["leg_1", 0, 10, 0],  # leg_1: Coxa = 0°, Femur = 0°, Tibia = 30°
    ["leg_3", 0, 10, 0],  # leg_1: Coxa = 0°, Femur = 0°, Tibia = 30°
    ["leg_4", -7, 0, 0],  # leg_1: Coxa = 0°, Femur = 0°, Tibia = 30°
    ["leg_6", 0, 0, 0],  # leg_1: Coxa = 0°, Femur = 0°, Tibia = 30°
]



def go_to_pose(pose_array, steps, steps_interval_ms):
    """Move the Hexapod to a new pose with a smooth transition using sinusoidal interpolation."""
    import math

    for step in range(steps + 1):
        factor = 0.5 * (1 - math.cos(math.pi * step / steps))  # Sinusoidal interpolation factor
        print(f"Step {step}/{steps}, Interpolation Factor: {factor}")

        for pose in pose_array:
            leg_name, coxa_target, femur_target, tibia_target = pose
            leg = hexapod.legs.get(leg_name)
            if not leg:
                print(f"Error: {leg_name} not found!")
                continue

            # Update each servo's set position gradually
            for servo, target_angle, servo_name in zip([leg.coxa, leg.femur, leg.tibia], 
                                                       [coxa_target, femur_target, tibia_target], 
                                                       ["coxa", "femur", "tibia"]):
                old_position = servo.actual_position
                servo.set_position = servo.actual_position + factor * (target_angle - servo.actual_position)

                # Calculate the corresponding pulse width
                servo.calculated_pulse = int(
                    servo.min_pulse +
                    ((servo.set_position - servo.min_angle) /
                    (servo.max_angle - servo.min_angle)) *
                    (servo.max_pulse - servo.min_pulse)
                )

                 # **Update actual position**
                servo.actual_position = servo.set_position




                print(f"{leg_name} {servo_name}: Old Pos = {old_position}°, New Pos = {servo.set_position}°, Pulse = {servo.calculated_pulse} µs")

        # Generate and send commands at each step
        uart_commands = generate_uart_commands()
        for cmd in uart_commands:
            print(f"Sending UART Command: {cmd}")
            send_uart_command(cmd)

        time.sleep(steps_interval_ms / 1000)  # Wait for next step

def go_multiple_to_pose(pose_array, steps, steps_interval_ms):
    """Move the Hexapod to a new pose with a smooth transition using sinusoidal interpolation.
    This version sends all servo commands in a single UART message."""
    import math

    for step in range(steps + 1):
        factor = 0.5 * (1 - math.cos(math.pi * step / steps))  # Sinusoidal interpolation factor
        print(f"Step {step}/{steps}, Interpolation Factor: {factor}")

        all_targets_reached = True  # Flag to check if all servos reached their targets
        max_position_diff = 0.1  # Maximum allowed difference to consider position reached

        for pose in pose_array:
            leg_name, coxa_target, femur_target, tibia_target = pose
            leg = hexapod.legs.get(leg_name)
            if not leg:
                print(f"Error: {leg_name} not found!")
                continue

            # Update each servo's set position gradually
            for servo, target_angle, servo_name in zip([leg.coxa, leg.femur, leg.tibia], 
                                                       [coxa_target, femur_target, tibia_target], 
                                                       ["coxa", "femur", "tibia"]):
                old_position = servo.actual_position
                new_position = servo.actual_position + factor * (target_angle - servo.actual_position)
                servo.set_position = new_position

                # Calculate the corresponding pulse width
                servo.calculated_pulse = int(
                    servo.min_pulse +
                    ((servo.set_position - servo.min_angle) /
                    (servo.max_angle - servo.min_angle)) *
                    (servo.max_pulse - servo.min_pulse)
                )

                # Update actual position
                servo.actual_position = servo.set_position

                # Check if this servo has reached its target
                position_diff = abs(target_angle - servo.actual_position)
                if position_diff > max_position_diff:
                    all_targets_reached = False

                print(f"{leg_name} {servo_name}: Old Pos = {old_position}°, New Pos = {servo.set_position}°, Pulse = {servo.calculated_pulse} µs")

        # Generate and send single command for all servos
        uart_command = generate_multiple_uart_command()
        #print(f"Sending UART Command: {uart_command}")
        print("Sending UART Command:", uart_command.hex(" "))  # Zeigt saubere Hex-Werte


        send_uart_command(uart_command)

        # If all targets are reached, we can skip the remaining steps
        if all_targets_reached:
            print(f"All targets reached at step {step}, skipping remaining steps")
            break

        time.sleep(steps_interval_ms / 1000)  # Wait for next step



if __name__ == "__main__":
    # cd pico/hexapod_servo_driver/python_tests/
    # python3 python_leg_test.py
    init()
    init_uart()
    print("################")
    go_to_init_position()
    print("################")
    print_servo_commands()

    print("################")

    uart_commands = generate_uart_commands()
    
    # Print and send generated UART commands
    # for cmd in uart_commands:
    #     #print(cmd)
    #     send_uart_command(cmd)
    #     time.sleep(1)  # 200ms delay
    
    # Test the single servo go to pose
    # go_to_pose(pose_01, steps=300, steps_interval_ms=5)
    # go_to_pose(pose_02, steps=300, steps_interval_ms=5)
    # go_to_pose(pose_hanging, steps=300, steps_interval_ms=5)

    # Test the new multiple servo command function
    print("Testing go_multiple_to_pose...")
    go_multiple_to_pose(pose_01, steps=300, steps_interval_ms=10)
    go_multiple_to_pose(pose_02, steps=300, steps_interval_ms=10)
    go_multiple_to_pose(pose_hanging, steps=300, steps_interval_ms=10)



    # Close UART connection at the end
    ser.close()
    print("UART connection closed.")