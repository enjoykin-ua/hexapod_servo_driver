import serial
import time
import struct

# UART configuration
SERIAL_PORT = "/dev/ttyACM0"  # USB-UART port
BAUDRATE = 115200  # Baud rate
TIMEOUT = 1  # UART timeout

# Initialize UART connection once
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
time.sleep(2)  # Wait for connection stabilization

def set_led(led_index, r, g, b):
    """
    Sends a command over UART to control a WS2812 LED.

    :param led_index: LED number (0-5)
    :param r: Red value (0-255)
    :param g: Green value (0-255)
    :param b: Blue value (0-255)
    """
    if not (0 <= led_index <= 5):
        print("Error: LED index must be between 0 and 5!")
        return

    try:
        # Construct data packet (Start, Command, LED Index, RGB, End Byte)
        command = bytes([0x55, 0x04, led_index, r, g, b, 0xAA])
        
        # Send command
        ser.write(command)
        ser.flush()

        print(f"Sent: LED {led_index} → RGB({r}, {g}, {b})")

    except serial.SerialException as e:
        print(f"Error: {e}")


def set_servo_pulse(servo_index, pulse_width):
    """
    Sends a command over UART to set the pulse width of a servo.

    :param servo_index: Servo number (0-18)
    :param pulse_width: Pulse width in microseconds (float)
    """
    if not (0 <= servo_index <= 18):
        print("Error: Servo index must be between 0 and 18!")
        return

    try:
        # Convert pulse width (float) to 4-byte Little-Endian format
        pulse_bytes = struct.pack("<f", pulse_width)  # "<f" = Little-Endian Float

        # Construct data packet: Start (0x55), Opcode (0x06), Servo, Pulse Bytes, End (0xAA)
        command = bytes([0x55, 0x06, servo_index]) + pulse_bytes + bytes([0xAA])
        
        print(f"send command:[{command}]")

        # Send command
        ser.write(command)
        ser.flush()

        print(f"Sent: Servo {servo_index} → Pulse {pulse_width} µs")

    except serial.SerialException as e:
        print(f"Error: {e}")

###############################################################

def test_leds():
    set_led(0, 0, 0, 0)  # Red
    set_led(1, 0, 0, 0)  # Green
    set_led(2, 0, 0, 0)  # Blue
    set_led(3, 0, 0, 0)  # Yellow
    set_led(4, 0, 0, 0)  # Cyan
    set_led(5, 0, 0, 0)  # Magenta

    set_led(0, 255, 0, 0)  # Red
    time.sleep(0.5)  # 200ms delay
    set_led(1, 0, 255, 0)  # Green
    time.sleep(0.5)
    set_led(2, 0, 0, 255)  # Blue
    time.sleep(0.5)
    set_led(3, 255, 255, 0)  # Yellow
    time.sleep(0.5)
    set_led(4, 0, 255, 255)  # Cyan
    time.sleep(0.5)
    set_led(5, 255, 0, 255)  # Magenta

if __name__ == "__main__":
    # Test: Set LED 0 to Red, LED 1 to Green, LED 2 to Blue, etc.
    test_leds()
    set_servo_pulse(0, 800.0)
    time.sleep(1)  # 200ms delay
    set_servo_pulse(0, 1700.0)

    # Close UART connection at the end
    ser.close()
    print("UART connection closed.")
