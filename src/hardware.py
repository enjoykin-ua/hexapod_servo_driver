import math
import time
from machine import Pin
from pimoroni import Analog, AnalogMux
from plasma import WS2812
from servo import ServoCluster, servo2040

# Constants
NUM_SERVOS = 3  # Total servos on the hexapod (set to 3 for testing)
SAMPLES = 50
UPDATES = 1000 / 50  # 50Hz update rate

# Initialize hardware components
mux = AnalogMux(servo2040.ADC_ADDR_0, servo2040.ADC_ADDR_1, servo2040.ADC_ADDR_2, muxed_pin=Pin(servo2040.SHARED_ADC))
sensor_adc = Analog(servo2040.SHARED_ADC)
cur_adc = Analog(servo2040.SHARED_ADC, servo2040.CURRENT_GAIN, servo2040.SHUNT_RESISTOR, servo2040.CURRENT_OFFSET)
led_bar = WS2812(servo2040.NUM_LEDS, 1, 0, servo2040.LED_DATA)
servos = None  # ServoCluster will be initialized in init_servos()

# Debug output to check servo pin assignment
print(f"[DEBUG] [HARDWARE] Servo pins will be initialized in init_servos()")

# Sensor addresses
sensor_addrs = list(range(servo2040.SENSOR_1_ADDR, servo2040.SENSOR_6_ADDR + 1))

def init_hardware():
    """
    Initializes the hardware components such as LEDs and sensors.
    """
    print("[DEBUG] [HARDWARE] Starting hardware initialization...")
    
    # Configure sensor pull-downs
    for addr in sensor_addrs:
        mux.configure_pull(addr, Pin.PULL_DOWN)
    
    # LED startup sequence
    led_bar.start()
    for i in range(servo2040.NUM_LEDS):
        led_bar.set_rgb(i, 255, 255, 255)  # White for startup indication
    time.sleep(0.5)
    led_bar.clear()
    
    print("[DEBUG] [HARDWARE] Hardware initialization complete.")

def init_servos():
    """
    Initializes the servo cluster once at startup and enables servos one by one.
    """
    global servos
    print("[DEBUG] [HARDWARE] Initializing ServoCluster...")
    servos = ServoCluster(pio=0, sm=0, pins=list(range(servo2040.SERVO_1, servo2040.SERVO_1 + NUM_SERVOS)))
    time.sleep(0.06)  # Allow some time for initialization
    
    # Enable each servo separately with a small delay
    for i in range(NUM_SERVOS):
        servos.enable(i)
        print(f"[DEBUG] [HARDWARE] Enabled Servo {i}")
        time.sleep(1)  # Wait a bit before enabling the next one
    
    print("[DEBUG] [HARDWARE] ServoCluster initialized and all servos enabled one by one.")

def init_leg(leg_id, pwm_0, pwm_1, pwm_2):
    """
    Initializes a specific leg by setting the servos to the given PWM values.
    """
    print(f"[DEBUG] [HARDWARE] Initializing leg {leg_id} with PWM values: {pwm_0}, {pwm_1}, {pwm_2}")
    
    # Calculate servo indices
    servo_1 = leg_id * 3
    servo_2 = leg_id * 3 + 1
    servo_3 = leg_id * 3 + 2
    
    print(f"[DEBUG] [HARDWARE] Calculated Servo IDs: {servo_1}, {servo_2}, {servo_3}")
    
    # Check if servo indices are in a valid range
    if not (0 <= servo_1 < NUM_SERVOS and 0 <= servo_2 < NUM_SERVOS and 0 <= servo_3 < NUM_SERVOS):
        print("[ERROR] [HARDWARE] Invalid servo indices! Aborting init_leg.")
        return
    
    # Enable each servo individually with a short delay

    print("[DEBUG] [HARDWARE] Enabled servos individually.")
    
    # Directly set servo PWM values
    servos.pulse(servo_1, pwm_0)
    time.sleep(0.01)
    print(f"[DEBUG] [HARDWARE] Set Servo {servo_1} to PWM {pwm_0}")
    
    servos.pulse(servo_2, pwm_1)
    time.sleep(0.01)
    print(f"[DEBUG] [HARDWARE] Set Servo {servo_2} to PWM {pwm_1}")
    
    servos.pulse(servo_3, pwm_2)
    time.sleep(0.01)
    print(f"[DEBUG] [HARDWARE] Set Servo {servo_3} to PWM {pwm_2}")
    
    print(f"[DEBUG] [HARDWARE] Leg {leg_id} initialized successfully.")





def get_sensor_data():
    """
    Reads sensor and current measurements and returns them.
    """
    switch_state = 0b000000
    for i, addr in enumerate(sensor_addrs):
        mux.select(addr)
        voltage = sensor_adc.read_voltage()
        #print(f"[DEBUG] Sensor {i} Voltage: {voltage:.3f}V")
        if voltage > 1.0:
            switch_state |= (1 << i)

    mux.select(servo2040.CURRENT_SENSE_ADDR)
    total_current = sum(cur_adc.read_current() for _ in range(SAMPLES)) / SAMPLES
    #print(f"[DEBUG] Total Current: {total_current:.3f} A")

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
    print("[DEBUG] Shutting down hardware...")
    servos.disable_all()
    led_bar.clear()
    print("[DEBUG] Hardware shut down.")

def apply_servo_angles_original(angles):
    """
    Moves the servos to the specified angles.
    """
    print(f"[DEBUG] Moving servos to angles: {angles}")
    
    for i, angle in enumerate(angles):
        servos.value(i, angle)  # Directly set servo angle
        print(f"[DEBUG] Servo {i} set to angle {angle}")
    
    print("[DEBUG] Servos moved to new angles.")
    
    
# Motion parameters
STEPS = 360              # More steps for smoother motion (doubled from 10 to 20)
STEPS_INTERVAL = 0.01   # Shorter delay for faster motion (halved from 0.5)
PWM_MIN = 1000          # Minimum pulse width (adjust if necessary)
PWM_MAX = 2000          # Maximum pulse width (adjust if necessary)

# Dictionary to store last servo positions
last_angles = {}

def angle_to_pwm(angle):
    """ Converts an angle (-90 to 90) to a PWM pulse width. """
    return int(PWM_MIN + ((angle + 90) / 180) * (PWM_MAX - PWM_MIN))

def apply_servo_angles(angles):
    """
    Moves all servos smoothly using a sine wave pattern.
    Stores last positions to ensure fluid transitions on consecutive runs.
    
    :param angles: List of target angles in degrees (-90 to 90).
    """
    print(f"[DEBUG] Received angles: {angles}")

    if len(angles) != NUM_SERVOS:
        print("[ERROR] Invalid number of angles received! Expected:", NUM_SERVOS)
        return

    # Retrieve last known positions, default to 0 if unknown
    global last_angles
    start_angles = [last_angles.get(i, 0) for i in range(NUM_SERVOS)]  

    print(f"[DEBUG] Moving all {NUM_SERVOS} servos smoothly from {start_angles} to {angles}")

    # Smooth motion for all servos using a sine wave
    for step in range(STEPS):
        sine_factor = math.sin(math.radians(step * (90 / STEPS)))  # Half sine wave

        for i in range(NUM_SERVOS):
            # Calculate interpolated position
            interpolated_angle = start_angles[i] + (angles[i] - start_angles[i]) * sine_factor
            pwm_value = angle_to_pwm(interpolated_angle)  # Convert angle to PWM

            # Apply PWM to servo
            servos.pulse(i, pwm_value)

            print(f"[DEBUG] Step {step+1}/{STEPS}: Servo {i} -> {interpolated_angle:.2f}° | PWM: {pwm_value}")

        time.sleep(STEPS_INTERVAL)  # Wait for smooth movement

    # Store new positions
    last_angles = {i: angles[i] for i in range(NUM_SERVOS)}
    print(f"[DEBUG] Stored new positions: {last_angles}")
