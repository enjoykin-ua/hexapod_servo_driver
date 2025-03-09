import sys
import time
import communication
import hardware
from pimoroni import Button
from servo import servo2040
import test  # Import test.py

# User button for system shutdown
user_sw = Button(servo2040.USER_SW)

# Flag to ensure test function is executed only once per button press
test_executed = False

# Custom PWM values for each servo (3 per leg × 6 legs)
initial_pwm_values_hanging2 = [
    1500, 850, 1500,  #VL Leg 1 [0, 1, 2]  (Coxa, Femur, Tibia)
    1500, 850, 1500,  # Leg 2 [3, 4, 5]
    1500, 2100, 1500,  # Leg 3 [6, 7, 8]
    1500, 2100, 1500,  # Leg 4
    1500, 850, 1500,  # Leg 5
    1500, 850, 1500   # Leg 6
]

initial_pwm_values_hanging = [
    1500, 850, 1500,  # Leg 1 [0, 1, 2]  (Coxa, Femur, Tibia)
    1500, 850, 1500,  # Leg 2 [3, 4, 5]
    1500, 2100, 1500,  # Leg 3 [6, 7, 8]
    1500, 2100, 1500,  # Leg 4
    1500, 850, 1500,  # Leg 5
    1500, 850, 1500   # Leg 6
]

hardware.initialize_pwm_values(initial_pwm_values_hanging) ##

# # Debug output for test cases
# hardware.debug_log("******************************************************", level=2)
# 
# Test Case 1: Normal command with 3 servos


test_command = "#SET_PWM[0,1000,1,1900,2,1000,3,1000,4,1900,5,1000,6,2000,7,850,8,2000,9,1800,10,850,11,2000]"
# communication.process_command(test_command)
# 
#test_command = "#SET_PWM[0,2000,1,800,2,2000,3,2000,4,800,5,2000,6,850,7,2000,8,850,9,850,10,2000,11,850]"
communication.process_command(test_command)



#set all servos to hangig-stand positon to avoid fast movements
test_command = "#SET_PWM[0,1500,1,850,2,1500,3,1500,4,850,5,1500,6,1500,7,2100,8,1500,9,1500,10,2100,11,1500]"
communication.process_command(test_command)



def main():
    """
    Main function to initialize the hardware and process incoming UART commands.
    """
    global test_executed

    hardware.debug_log("", level=2)
    hardware.debug_log("##########################################################################", level=2)
    hardware.debug_log("[INFO] System starting...", level=2)

    # Initialize hardware (LEDs, sensors, general setup)
    hardware.init_hardware()
    #hardware.init_servos()
    #test.test_servo()  

    hardware.debug_log("[INFO] System ready. Waiting for UART commands...", level=2)

    # Main loop
    while not user_sw.raw():
        # Read sensor state and total current
        switch_state, total_current = hardware.get_sensor_data()
        command = ""
        
        # Read incoming command from UART or stdin
        command = communication.read_command()
        if command:
            communication.process_command(command)  # Process angles and init commands
        
        # Send sensor data via UART
        #communication.send_status(switch_state, total_current)

        # Update LEDs based on sensor state
        hardware.update_leds(switch_state)

        # Ensure proper timing for stable operation (matching original implementation)
        time.sleep(1.0 / 50)  # 50 Hz update rate

    # Shutdown sequence
    hardware.shutdown()
    hardware.debug_log("[INFO] System stopped.", level=2)

# Run the main loop if executed directly
if __name__ == "__main__":
    main()


