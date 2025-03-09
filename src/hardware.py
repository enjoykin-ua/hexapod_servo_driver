import sys
import math
import time
from machine import Pin
from pimoroni import Analog, AnalogMux
from plasma import WS2812
from servo import Servo, ServoCluster, servo2040

# Constants
NUM_SERVOS = 18  # Total servos on the hexapod (set to 3 for testing)
SAMPLES = 50
UPDATES = 1000 / 50  # 50Hz update rate

# Initialize hardware components
mux = AnalogMux(servo2040.ADC_ADDR_0, servo2040.ADC_ADDR_1, servo2040.ADC_ADDR_2, muxed_pin=Pin(servo2040.SHARED_ADC))
sensor_adc = Analog(servo2040.SHARED_ADC)
cur_adc = Analog(servo2040.SHARED_ADC, servo2040.CURRENT_GAIN, servo2040.SHUNT_RESISTOR, servo2040.CURRENT_OFFSET)
led_bar = WS2812(servo2040.NUM_LEDS, 1, 0, servo2040.LED_DATA)

START_PIN = servo2040.SERVO_1
END_PIN = servo2040.SERVO_18
servos = [Servo(i) for i in range(START_PIN, END_PIN + 1)]


# Set debug level (change as needed: 0 = all, 5 = silent)
DEBUG_LEVEL = 8  # 0 = Alle, 1 = Wichtige Logs, 5 = Keine Logs

def debug_log(message, level=2):
    """
    Logs debug messages based on the defined DEBUG_LEVEL.
    
    :param message: The debug message to print.
    :param level: The priority of the message (lower = more important).
    """
    if level >= DEBUG_LEVEL:
        print(f"[DEBUG] {message}", file=sys.stderr)  # Output to stderr (not UART)




# Debug output to check servo pin assignment
debug_log("[HARDWARE] Servo pins will be initialized in init_servos()", level=2)

# Sensor addresses
sensor_addrs = list(range(servo2040.SENSOR_1_ADDR, servo2040.SENSOR_6_ADDR + 1))

def init_hardware():
    """
    Initializes the hardware components such as LEDs and sensors.
    """
    debug_log("[HARDWARE] Starting hardware initialization...", level=2)
    
    # Configure sensor pull-downs
    for addr in sensor_addrs:
        mux.configure_pull(addr, Pin.PULL_DOWN)
    
    # LED startup sequence
    led_bar.start()
    for i in range(servo2040.NUM_LEDS):
        led_bar.set_rgb(i, 255, 255, 255)  # White for startup indication
    time.sleep(0.5)
    led_bar.clear()
    
    debug_log("[HARDWARE] Hardware initialization complete.", level=2)

def get_sensor_data():
    """
    Reads sensor and current measurements and returns them.
    """
    switch_state = 0b000000
    for i, addr in enumerate(sensor_addrs):
        mux.select(addr)
        voltage = sensor_adc.read_voltage()
        if voltage > 1.0:
            switch_state |= (1 << i)

    mux.select(servo2040.CURRENT_SENSE_ADDR)
    total_current = sum(cur_adc.read_current() for _ in range(SAMPLES)) / SAMPLES

    return switch_state, total_current

def update_leds(switch_state):
    """
    Updates LEDs based on sensor states.
    """
    for i in range(len(sensor_addrs)):
        if switch_state & (1 << i):
            led_bar.set_rgb(i, 0, 255, 0)  # Green for active
        else:
            led_bar.set_rgb(i, 0, 0, 0)

def shutdown():
    """
    Shuts down the hardware components.
    """
    debug_log("[HARDWARE] Shutting down hardware...", level=2)
    servos.disable_all()
    led_bar.clear()
    debug_log("[HARDWARE] Hardware shut down.", level=2)

# Dictionary to store last known PWM values for each servo
last_pwm_values = {}

def initialize_pwm_values(pwm_array):
    """
    Initializes the last known PWM values for all servos.
    
    :param pwm_array: List of 18 PWM values to set as the initial state.
    """
    global last_pwm_values

    if len(pwm_array) != 18:
        debug_log("[HARDWARE] Invalid initialization array. Expected 18 values.", level=2)
        return

    # Store the initial values in last_pwm_values
    last_pwm_values = {i: pwm_array[i] for i in range(18)}

    debug_log(f"[HARDWARE] PWM values initialized: {last_pwm_values}", level=2)

# Motion parameters
STEPS = 100             # More steps for smoother motion
STEPS_INTERVAL = 0.02   # Time delay between steps (adjust for speed)

def set_multiple_servo_pwm(pwm_dict):
    """
    Smoothly moves multiple servos to their target PWM values using a sine wave pattern.
    
    :param pwm_dict: Dictionary where keys are servo pin numbers and values are target PWM values.
    """
    global last_pwm_values

    # Retrieve last known PWM values
    start_pwm_values = {pin: last_pwm_values[pin] for pin in pwm_dict.keys()}

    debug_log(f"[HARDWARE] Moving Servos smoothly from {start_pwm_values} to {pwm_dict}", level=2)

    for step in range(STEPS):
        sine_factor = math.sin(math.radians(step * (90 / STEPS)))  # Half sine wave (0 → 1)
        all_servos_reached_target = True  # Flag to check if all servos are done

        for pin_number, target_pwm in pwm_dict.items():
            interpolated_pwm = int(start_pwm_values[pin_number] + (target_pwm - start_pwm_values[pin_number]) * sine_factor)

            # Ensure no unnecessary updates
            if interpolated_pwm != target_pwm:
                all_servos_reached_target = False  # At least one servo still moving

            # Apply PWM to the specified servo
            servos[pin_number].pulse(interpolated_pwm)

        time.sleep(STEPS_INTERVAL)  # Ensure smooth movement timing

        # **Exit loop early if all servos have reached their target values**
        if all_servos_reached_target:
            debug_log("[HARDWARE] All servos have reached their target PWM. Exiting loop early.", level=2)
            break

    # Store the new PWM values
    for pin_number, target_pwm in pwm_dict.items():
        last_pwm_values[pin_number] = target_pwm

    debug_log(f"[HARDWARE] Servos reached final PWM positions: {last_pwm_values}", level=2)



