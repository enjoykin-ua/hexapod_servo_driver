import json
import math
from kinematics import inverse_kinematics
from Hexapod_object import Hexapod  # Import the Hexapod class

# Load the hexapod configuration
with open("configuration.json", "r") as file:
    config = json.load(file)

# Initialize the Hexapod object
hexapod = Hexapod("configuration.json", init_mode="ground")

# Select a test leg (e.g., "leg_1")
test_leg_name = "leg_6"
test_leg = hexapod.legs[test_leg_name]

# Define multiple test target positions (x, y, z)
test_positions = [
    (100.0, 0.0, 0.0),   # Max stretched
    (70.0, 0.0, -30.0),  # Slightly bent
    (50.0, 0.0, -50.0),  # 45° down
    (80.0, 0.0, 20.0)    # Slightly above neutral
]

print(f"\nTesting inverse kinematics for {test_leg_name}")

# Iterate through all test positions
for i, (x, y, z) in enumerate(test_positions):
    print(f"\nTest {i + 1}: Target position → x={x} mm, y={y} mm, z={z} mm")

    try:
        # Compute inverse kinematics
        theta1, theta2, theta3 = inverse_kinematics(test_leg, x, y, z)

        # Convert radians to degrees for readability
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)
        theta3_deg = math.degrees(theta3)

        print(f"✅ SUCCESS: Computed joint angles (degrees):")
        print(f"  Coxa:  {theta1_deg:.2f}°")
        print(f"  Femur: {theta2_deg:.2f}°")
        print(f"  Tibia: {theta3_deg:.2f}°")

    except ValueError as e:
        print(f"❌ ERROR: {e}")
