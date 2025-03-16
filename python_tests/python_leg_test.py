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
    
    go_to_pose(pose_01, steps=300, steps_interval_ms=5)
    
    go_to_pose(pose_02, steps=300, steps_interval_ms=5)

    go_to_pose(pose_hanging, steps=300, steps_interval_ms=5)
    # Close UART connection at the end
    ser.close()
    print("UART connection closed.")