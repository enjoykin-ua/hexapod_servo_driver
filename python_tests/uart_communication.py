import struct
import serial
import time

class UARTCommunication:
    def __init__(self):
        self.ser = None

    def init_uart(self):
        """Initialize the UART communication."""
        SERIAL_PORT = "/dev/ttyACM0"  # USB-UART port
        BAUDRATE = 115200  # Baud rate
        TIMEOUT = 1  # UART timeout

        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
        time.sleep(2)  # Wait for connection stabilization
        print("UART initialized.")

    def send_uart_command(self, command):
        """Sends a given command over UART."""
        try:
            self.ser.write(command)
            self.ser.flush()
            print(f"Sent: {command}")
        except serial.SerialException as e:
            print(f"Error: {e}")

    def generate_multiple_uart_command(self, hexapod):
        """Generates a single UART command packet for setting multiple servo pulse widths.
        Only includes servos that still need to move (position difference > threshold)."""
        command = bytearray()  # Use bytearray for better byte handling
        command.append(0x55)   # COMMAND_START
        command.append(0x07)   # CMD_SET_SERVO_PULSES

        # Collect servo data only for servos that need to move
        servo_data = bytearray()

        for leg_name, leg in hexapod.legs.items():
            for servo_name in ["coxa", "femur", "tibia"]:
                servo = getattr(leg, servo_name)
                
                # Only include servo if it hasn't reached its target position yet
                if not servo.servo_reached_position:
                    pulse_bytes = struct.pack("<f", servo.calculated_pulse)  # Convert float to bytes (Little-Endian)
                    servo_data.append(servo.pin)  # Add servo pin
                    servo_data.extend(pulse_bytes)  # Add 4-byte pulse width

        # Total number of servos (must be set AFTER counting them)
        total_servos = len(servo_data) // 5  # Each servo has 5 bytes (pin + 4 float)
        command.append(total_servos)  # Append servo count
        command.extend(servo_data)  # Append all servo data
        command.append(0xAA)  # COMMAND_END

        return bytes(command)

    def close_uart(self):
        """Close the UART connection."""
        if self.ser:
            self.ser.close()
            print("UART connection closed.") 