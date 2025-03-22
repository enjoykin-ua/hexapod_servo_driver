import math
from Hexapod_object import Leg  # Import the Leg class

def inverse_kinematics(leg: Leg, x: float, y: float, z: float) -> tuple:
    """
    Computes the inverse kinematics (IK) for a single leg of the hexapod.

    :param leg: `Leg` object containing all relevant leg and joint data.
    :param x: Target position of the foot in mm (relative to the Coxa joint).
    :param y: Target position of the foot in mm (relative to the Coxa joint).
    :param z: Target height of the foot in mm (relative to the Coxa joint).

    :return: Tuple with (Coxa angle, Femur angle, Tibia angle) in radians.
    """
    # 1️⃣ Compute the Coxa angle (rotation in the XY plane)
    theta1 = math.atan2(y, x)  # Coxa rotates the leg towards the target position
    theta1 += math.radians(leg.coxa.offset_angle)  # Apply Coxa offset correction

    # 2️⃣ Compute the projected 2D distance for Femur & Tibia calculations
    d_xy = math.sqrt(x**2 + y**2)  # Distance in the XY plane
    D = math.sqrt(d_xy**2 + z**2)  # Total distance from the Coxa joint

    # 3️⃣ Check if the target position is within reachable range
    if D > (leg.femur.length_mm + leg.tibia.length_mm) or D < abs(leg.femur.length_mm - leg.tibia.length_mm):
        raise ValueError(f"Target ({x}, {y}, {z}) is out of reach for leg {leg.name}!")

    # 4️⃣ Compute Tibia angle (θ3) using the Law of Cosines
    theta3 = math.acos((leg.femur.length_mm**2 + leg.tibia.length_mm**2 - D**2) / 
                        (2 * leg.femur.length_mm * leg.tibia.length_mm))
    theta3 = math.pi - theta3  # Adjust orientation
    theta3 += math.radians(leg.tibia.offset_angle)  # Apply Tibia offset correction

    # 5️⃣ Compute Femur angle (θ2)
    theta2 = math.atan2(z, d_xy) + math.acos((leg.femur.length_mm**2 + D**2 - leg.tibia.length_mm**2) / 
                                             (2 * leg.femur.length_mm * D))
    theta2 += math.radians(leg.femur.offset_angle)  # Apply Femur offset correction

    # 6️⃣ Check joint angle limits and warn if out of range
    angles = {"coxa": theta1, "femur": theta2, "tibia": theta3}
    for joint, angle in angles.items():
        min_radians = math.radians(getattr(leg, joint).min_angle)
        max_radians = math.radians(getattr(leg, joint).max_angle)

        if angle < min_radians or angle > max_radians:
            print(f"⚠ WARNING: {leg.name} {joint} angle {math.degrees(angle):.2f}° exceeds limits ({getattr(leg, joint).min_angle}° to {getattr(leg, joint).max_angle}°)!")

    return theta1, theta2, theta3  # Return angles in radians
