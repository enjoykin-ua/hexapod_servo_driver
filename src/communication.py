import sys
import select
import re
from machine import UART, Pin
from hardware import set_multiple_servo_pwm, debug_log  # Import functions from hardware.py

# UART setup
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), timeout=50)

# Regex patterns for parsing commands
SET_PWM_PATTERN = re.compile(r"#SET_PWM\[\s*((?:\d+\s*,\s*\d+\s*,?\s*)+)\]")


def read_command():
    """
    Reads commands from UART or stdin and ensures that they are complete before processing.
    """
    command = read_uart_input() or read_stdin_input()
    if command:
        debug_log(f"[COMMUNICATION] Processing command: {command}", level=2)
        return command
    return None


def read_uart_input_original():
    """
    Reads a full UART command, removes unnecessary characters, 
    and extracts only the valid command inside #SET_PWM[...] brackets.
    """
    # Versuche, den UART-Puffer zu leeren, bevor neue Daten kommen
    while uart.any():
        uart.read()

    raw_data = uart.readline()  # Lese eine komplette Zeile von UART
    print(f"Received: {raw_data}")

    if raw_data:
        try:
            command = raw_data.decode(errors='ignore').strip()
            
            # Falls der Befehl zu kurz ist oder nur Schrott enthält → Verwerfen
            if len(command) < 10:
                debug_log("[COMMUNICATION] Ignored short/broken UART input.", level=2)
                return None
            
            debug_log(f"[COMMUNICATION] Raw UART input: {command}", level=2)

            # Extrahiere nur den Befehl innerhalb von #SET_PWM[...]
            start = command.find("#SET_PWM[")
            end = command.find("]", start)  # Schließende Klammer suchen

            if start != -1 and end != -1:
                cleaned_command = command[start:end + 1]  # Nur den gültigen Teil behalten
                debug_log(f"[COMMUNICATION] Cleaned UART command: {cleaned_command}", level=2)
                return cleaned_command
            else:
                debug_log("[COMMUNICATION] No valid #SET_PWM command found.", level=2)
                return None
        except UnicodeDecodeError:
            debug_log("[COMMUNICATION] Failed to decode UART input.", level=2)
    return None

uart_buffer = ""
def read_uart_input():
    """
    Reads UART input, splits commands by newline (\n), and processes them immediately.
    """
    global uart_buffer  

    if uart.any():
        new_data = uart.read().decode(errors='ignore')
        uart_buffer += new_data  # Append new data to buffer
        led_flash_other()

    if "\n" in uart_buffer:
        # Split buffer into commands
        commands = uart_buffer.split("\n")
        
        # Process only the complete command(s)
        for cmd in commands[:-1]:  
            if cmd.startswith("#SET_PWM[") and cmd.endswith("]"):
                debug_log(f"[COMMUNICATION] Processing command: {cmd}", level=2)
                uart.write(f"[DEBUG] Processed command: {cmd}\n".encode())
                return cmd

        # Keep only the last (possibly incomplete) command in buffer
        uart_buffer = commands[-1]

    return None




def read_stdin_input():
    """
    Reads input from stdin without blocking.
    """
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None


def process_command(command):
    """
    Processes incoming UART commands and forwards them to the corresponding hardware functions.
    """
    debug_log(f"[COMMUNICATION] Processing command: {command}", level=2)

    if command.startswith("#SET_PWM"):  # Handle multiple servo PWM settings
        pwm_dict = extract_multiple_pwm(command)
        if pwm_dict:
            debug_log(f"[COMMUNICATION] Sending multiple PWM values to hardware: {pwm_dict}", level=2)
            set_multiple_servo_pwm(pwm_dict)  # Pass the whole dictionary at once


def extract_multiple_pwm(command):
    """
    Extracts multiple servo PWM values from a valid #SET_PWM command.
    Ensures that only properly formatted values are used.
    """
    # Use regex to extract only the values inside the square brackets
    match = SET_PWM_PATTERN.match(command)
    if match:
        try:
            raw_values = match.group(1).replace(" ", "")  # Remove unnecessary spaces
            debug_log(f"[COMMUNICATION] Extracted raw PWM values: {raw_values}", level=2)

            # Split values into a list of integers
            values = list(map(int, raw_values.split(",")))
            if len(values) % 2 != 0:
                debug_log("[COMMUNICATION] Invalid format: Uneven number of arguments in #SET_PWM command.", level=2)
                return None
            
            # Create a dictionary where keys are pin numbers and values are PWM values
            pwm_dict = {values[i]: values[i + 1] for i in range(0, len(values), 2)}
            debug_log(f"[COMMUNICATION] Parsed PWM values: {pwm_dict}", level=2)
            return pwm_dict
        
        except ValueError:
            debug_log("[COMMUNICATION] Invalid numeric values in #SET_PWM command.", level=2)
            return None
    else:
        debug_log(f"[COMMUNICATION] Invalid format. Expected: #SET_PWM[n1,pwm1,n2,pwm2,...], received: {command}", level=2)
        return None


def send_status(switch_state, current):
    """
    Sends sensor state and current measurement via UART in the format [XXXXXX] [X.XXX].
    """
    uart_data = f"[{switch_state:06b}] [{current:.3f}]\n"
    uart.write(uart_data)
    debug_log(f"[COMMUNICATION] Sent status: {uart_data.strip()}", level=2)


##############################################################################################################


def test_set_pwm():
    """
    Simulates a UART command to set a single servo's PWM.
    """
    test_command = "#SET_PWM[1,2300]"  # Example command to set Servo 1 to 1500 µs PWM

    debug_log(f"[TEST] Sending test command: {test_command}", level=2)
    
    # Call the process_command function with the test command
    return test_command

