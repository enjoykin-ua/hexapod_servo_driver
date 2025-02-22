import time
import communication
import hardware
from pimoroni import Button
from servo import servo2040

# User button for system shutdown
user_sw = Button(servo2040.USER_SW)

# Flag to ensure test function is executed only once per button press
test_executed = False

def main():
    """
    Main function to initialize the hardware and process incoming UART commands.
    """
    global test_executed
    
    print("[INFO] System starting...")

    # Initialize hardware (LEDs, sensors, general setup)
    hardware.init_hardware()
    hardware.init_servos()

    print("[INFO] System ready. Waiting for UART commands...")

    # Main loop
    while not user_sw.raw():
        # Read sensor state and total current
        switch_state, total_current = hardware.get_sensor_data()
        command = ""
        
        # Check if any button is pressed and ensure test is executed only once per press
        if switch_state and not test_executed:
            print("[INFO] Test button pressed. Executing test function...")
            command = communication.read_test_angles_command()
            if command:
                communication.process_command(command)
                test_executed = True  # Ensure it runs only once until button is released

        # Reset the flag when button is released
        if not switch_state:
            test_executed = False
        else:
            # Skip normal command reading when test function has run
            continue

        # Read incoming command from UART or stdin
        #command = communication.read_command()
        if command:
            communication.process_command(command)  # Process angles and init commands
        
        # Send sensor data via UART
        #communication.send_status(switch_state, total_current)

        # Update LEDs based on sensor state
        #hardware.update_leds(switch_state)

        # Ensure proper timing for stable operation (matching original implementation)
        time.sleep(1.0 / 50)  # 50 Hz update rate

    # Shutdown sequence
    hardware.shutdown()
    print("[INFO] System stopped.")

# Run the main loop if executed directly
if __name__ == "__main__":
    main()
