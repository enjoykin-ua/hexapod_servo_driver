import time
from Hexapod_object import Hexapod
from uart_communication import UARTCommunication
from kinematic_movements import KinematicMovements

def calibrate(uart_comm: UARTCommunication, servo_pin: int):
    """
    Calibrate a single servo by sending direct pulse width values.
    
    :param uart_comm: UART communication object
    :param servo_pin: Pin number of the servo to calibrate
    """
    import struct

    def send_pulse_command(pin: int, pulse: float):
        """Helper function to send the pulse command"""
        command = bytearray()
        command.append(0x55)  # Start byte
        command.append(0x06)  # Opcode for SET_SERVO_PULSE
        command.append(pin)   # Servo pin
        command.extend(struct.pack('<f', pulse))  # Float pulse width, little endian
        command.append(0xAA)  # End byte
        print(f"Sending pulse: {pulse:.1f}")
        uart_comm.send_uart_command(command)

    print(f"\nCalibrating servo on pin {servo_pin}")
    print("Enter pulse width (500-2500) or 'q' to quit")

    while True:
        try:
            value = input("\nPulse width: ")
            
            if value.lower() == 'q':
                print("Exiting calibration mode.")
                break
            
            pulse = float(value)
            if pulse < 500 or pulse > 2500:
                print("Error: Pulse width must be between 500 and 2500 microseconds!")
                continue
            
            send_pulse_command(servo_pin, pulse)
            
        except ValueError:
            print("Error: Please enter a valid number!")
        except Exception as e:
            print(f"Error: {e}")

# Define poses (angles)
pose_hanging = [
    ["leg_1", 0, -90, 0],
    ["leg_3", 0, -90, 0],
    ["leg_4", 0, -90, 0],
    ["leg_6", 0, -90, 0]  
]

pose_init_start = [
    ["leg_1", 0, 80, 30],
    ["leg_3", 0, 80, 30],
    ["leg_4", 0, 80, 30],
    ["leg_6", 0, 80, 30]  
]

pose_1 = [
    ["leg_1", 0, 0, 30],  # leg_1: Coxa = 0°, Femur = 0°, Tibia = 30°
    ["leg_3", 0, 0, -30], # leg_3: Coxa = 0°, Femur = 0°, Tibia = -30°
    ["leg_4", 0, 0, -30], # leg_4: Coxa = 0°, Femur = 0°, Tibia = -30°
    ["leg_6", 0, 0, 30]   # leg_6: Coxa = 0°, Femur = 0°, Tibia = 30°
]

pose_01 = [
    ["leg_1", 45, 10, 90],  # leg_1: Coxa = 45°, Femur = 10°, Tibia = 90°
    ["leg_3", 30, 10, 90],  # leg_3: Coxa = 30°, Femur = 10°, Tibia = 90°
    ["leg_4", 30, 0, 90],  # leg_4: Coxa = -30°, Femur = 0°, Tibia = 90°
    ["leg_6", 45, 0, 90],  # leg_6: Coxa = -45°, Femur = 0°, Tibia = 90°
]

pose_02 = [
    ["leg_1", 0, 10, 0],  # leg_1: Coxa = 0°, Femur = 10°, Tibia = 0°
    ["leg_3", 0, 10, 0],  # leg_3: Coxa = 0°, Femur = 10°, Tibia = 0°
    ["leg_4", 0, 0, 0],  # leg_4: Coxa = -7°, Femur = 0°, Tibia = 0°
    ["leg_6", 0, 0, 0],   # leg_6: Coxa = 0°, Femur = 0°, Tibia = 0°
]

# Define XYZ coordinate poses
xyz_test_01 = [
    # leg_name, x, y, z (in mm)
    ["leg_1", 100.0, 0.0, 0.0],   # Max stretched forward
    ["leg_3", 100.0, 0.0, 0.0],   # Max stretched forward
    ["leg_4", 100.0, 0.0, 0.0],   # Max stretched forward
    ["leg_6", 100.0, 0.0, 0.0],   # Max stretched forward
]

xyz_test_02 = [
    # leg_name, x, y, z (in mm)
    ["leg_1", 70.0, 0.0, -70.0],  # Forward and down
    ["leg_3", 70.0, 0.0, -70.0],  # Forward and down
    ["leg_4", 70.0, 0.0, -70.0],  # Forward and down
    ["leg_6", 70.0, 0.0, -70.0],  # Forward and down
]

xyz_test_02_1 = [
    # leg_name, x, y, z (in mm)
    ["leg_1", 70.0, 0.0, -50.0],  # Forward and down
    ["leg_3", 70.0, 0.0, -50.0],  # Forward and down
    ["leg_4", 70.0, 0.0, -50.0],  # Forward and down
    ["leg_6", 70.0, 0.0, -50.0],  # Forward and down
]

xyz_test_03 = [
    # leg_name, x, y, z (in mm)
    ["leg_1", 50.0, 50.0, -20.0],  # Forward-right and down
    ["leg_3", 50.0, -50.0, -20.0], # Forward-left and down
    ["leg_4", 50.0, -50.0, -20.0], # Forward-left and down
    ["leg_6", 50.0, 50.0, -20.0],  # Forward-right and down
]

def main():
    # Initialize UART communication
    uart_comm = UARTCommunication()
    uart_comm.init_uart()

    # Check if we're in calibration mode
    # cal_input = input("Enter servo pin number to calibrate, or press Enter to continue normally: ")
    # if cal_input.strip():
    #     try:
    #         servo_pin = int(cal_input)
    #         calibrate(uart_comm, servo_pin)
    #         return  # Exit after calibration
    #     except ValueError:
    #         print("Invalid pin number, continuing with normal operation.")

    # Normal operation continues here
    # Initialize Hexapod
    hexapod = Hexapod("configuration.json", init_mode="ground")
    print("Hexapod initialized.")

    # Initialize Kinematic Movements
    kinematics = KinematicMovements(uart_comm)

    print("################")
    # Go to initial position
    #kinematics.go_to_init_position(hexapod)
    print("################")

    # Execute movement sequence
    print("Testing angle-based movements...")

    init_poosition = pose_init_start

    kinematics.go_multiple_to_pose(hexapod, init_poosition, steps=300, steps_interval_ms=12)
    # kinematics.go_multiple_to_pose(hexapod, pose_01, steps=300, steps_interval_ms=12)
    # kinematics.go_multiple_to_pose(hexapod, pose_02, steps=300, steps_interval_ms=12)
    
    #kinematics.go_multiple_to_pose(hexapod, pose_init_start, steps=300, steps_interval_ms=12)
    input("Press Enter for next movement...")   

    

    for i in range(1):        
        #print("\nTesting XYZ coordinate movements...")
        #kinematics.go_to_xyz_position(hexapod, xyz_test_01, steps=300, steps_interval_ms=12)
        #time.sleep(2)  # 200ms delay
        #input("Press Enter for next XYZ movement...")
        kinematics.go_to_xyz_position(hexapod, xyz_test_02, steps=300, steps_interval_ms=12)
        time.sleep(2)  # 200ms delay
        #input("Press Enter for next XYZ movement...")
        kinematics.go_to_xyz_position(hexapod, xyz_test_02_1, steps=300, steps_interval_ms=12)
        time.sleep(2)  # 200ms delay
        kinematics.go_to_xyz_position(hexapod, xyz_test_02, steps=300, steps_interval_ms=12)
        time.sleep(2)  # 200ms delay
        kinematics.go_to_xyz_position(hexapod, xyz_test_02_1, steps=300, steps_interval_ms=12)
        time.sleep(2)  # 200ms delay
        #input("Press Enter for next XYZ movement...")   

    # Return to hanging position
    kinematics.go_multiple_to_pose(hexapod, init_poosition, steps=300, steps_interval_ms=12)

    print("All movements finished!")

    # Close UART connection
    uart_comm.close_uart()

if __name__ == "__main__":
    main() 



