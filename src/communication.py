import sys
import select
import re
from machine import UART, Pin
from hardware import init_leg, apply_servo_angles  # Import functions from hardware.py

# UART setup
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), timeout=50)

# Regex patterns for parsing commands
ANGLE_PATTERN = re.compile(r"#ANGLES\[\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)\s*\]")
INIT_LEG_PATTERN = re.compile(r"#INIT_LEG\[\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\s*\]")

toggle_flag = False
raw_data_1 = b"#INIT_LEG[0,1000,1000,1000]"  # Example test command
raw_data_2 = b"#INIT_LEG[0,1800,1800,1800]"  # Example test command

def read_test_command():
    """
    Simulates reading a test command instead of UART input.
    """
    #raw_data = b"#INIT_LEG[0,1487,1455,1477]"  # Example test command
    global toggle_flag
    raw_data = raw_data_1 if toggle_flag else raw_data_2
    toggle_flag = not toggle_flag
    
    try:
        command = raw_data.decode().strip()
        print(f"[DEBUG] [COMMUNICATION] Received test command: {command}")
        return command
    except UnicodeDecodeError:
        print("[ERROR] [COMMUNICATION] Failed to decode test input.")
    return None

def read_command():
    """
    Reads commands from UART or stdin.
    """
    if uart.any():
        raw_data = uart.readline()
        if raw_data:
            try:
                command = raw_data.decode().strip()
                print(f"[DEBUG] [COMMUNICATION] Received UART command: {command}")
                return command
            except UnicodeDecodeError:
                print("[ERROR] [COMMUNICATION] Failed to decode UART input.")
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        command = sys.stdin.readline().strip()
        print(f"[DEBUG] [COMMUNICATION] Received stdin command: {command}")
        return command
    return None

def process_command(command):
    """
    Processes incoming UART commands and forwards them to hardware functions.
    """
    print(f"[DEBUG] [COMMUNICATION] Processing command: {command}")
    if command.startswith("#INIT_LEG"):
        extract_init_leg(command)
    elif command.startswith("#ANGLES"):
        angles = extract_angles(command)
        if angles:
            print(f"[DEBUG] [COMMUNICATION] Applying angles: {angles}")
            apply_servo_angles(angles)  # Move servos based on received angles

def extract_angles(command):
    """
    Extracts angle values from a valid #ANGLES command.
    """
    match = ANGLE_PATTERN.match(command)
    if match:
        try:
            angles = [int(match.group(1)), int(match.group(2)), int(match.group(3))]
            print(f"[DEBUG] [COMMUNICATION] Extracted angles: {angles}")
            return angles
        except ValueError:
            print("[ERROR] [COMMUNICATION] Invalid input. Expected integer values between -90 and 90.")
            return None
    else:
        print(f"[ERROR] [COMMUNICATION] Invalid format. Expected: #ANGLES[30,-30,45], received: {command}")
        return None

def extract_init_leg(command):
    """
    Extracts initialization values from a valid #INIT_LEG command and calls init_leg.
    """
    match = INIT_LEG_PATTERN.match(command)
    if match:
        try:
            leg_number = int(match.group(1))
            init_pwm_0 = int(match.group(2))
            init_pwm_1 = int(match.group(3))
            init_pwm_2 = int(match.group(4))

            print(f"[DEBUG] [COMMUNICATION] Extracted INIT_LEG values: Leg {leg_number}, PWM: {init_pwm_0}, {init_pwm_1}, {init_pwm_2}")
            init_leg(leg_number, init_pwm_0, init_pwm_1, init_pwm_2)
        except ValueError:
            print("[ERROR] [COMMUNICATION] Invalid input. Expected integer values for INIT_LEG.")

def send_status(switch_state, current):
    """
    Sends sensor state and current measurement via UART in the format [XXXXXX] [X.XXX].
    """
    uart_data = f"[{switch_state:06b}] [{current:.3f}]\n"
    uart.write(uart_data)
    print(f"[DEBUG] [COMMUNICATION] Sent status: {uart_data.strip()}")

##########################################################################################################
#test functions, use as needed to imitate incomming message from uart
##########################################################################################################


toggle_flag_angles = False
raw_angle_1 = b"#ANGLES[30,-30,45]"  # Example test command
raw_angle_2 = b"#ANGLES[-45,45,0]"  # Example test command

def read_test_angles_command():
    """
    Simulates reading a test ANGLES command instead of UART input.
    """
    global toggle_flag_angles
    raw_data = raw_angle_1 if toggle_flag_angles else raw_angle_2
    toggle_flag_angles = not toggle_flag_angles
    
    try:
        command = raw_data.decode().strip()
        print(f"[DEBUG] [COMMUNICATION] Received test ANGLES command: {command}")
        return command
    except UnicodeDecodeError:
        print("[ERROR] [COMMUNICATION] Failed to decode test input.")
    return None

################################################################################################

toggle_flag_angles = False
raw_angle_1 = b"#ANGLES[70,140,20]"  # Example test command
raw_angle_2 = raw_angle_1
raw_angle_2 = b"#ANGLES[0,140,-20]"  # Example test command

def read_test_angles_command():
    """
    Simulates reading a test ANGLES command instead of UART input.
    """
    global toggle_flag_angles
    raw_data = raw_angle_1 if toggle_flag_angles else raw_angle_2
    toggle_flag_angles = not toggle_flag_angles
    
    try:
        command = raw_data.decode().strip()
        print(f"[DEBUG] [COMMUNICATION] Received test ANGLES command: {command}")
        return command
    except UnicodeDecodeError:
        print("[ERROR] [COMMUNICATION] Failed to decode test input.")
    return None

    
