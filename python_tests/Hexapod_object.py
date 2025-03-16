import json

class Servo:
    """Class representing a servo motor for a Hexapod joint."""
    def __init__(self, pin, length_mm, min_angle, mid_angle, max_angle, min_pulse, middle_pulse, max_pulse, init_position):
        self.pin = pin
        self.length_mm = length_mm
        self.min_angle = min_angle
        self.mid_angle = mid_angle  # Neutral position angle
        self.max_angle = max_angle
        self.min_pulse = min_pulse
        self.middle_pulse = middle_pulse  # Center pulse width
        self.max_pulse = max_pulse
        self.init_position = init_position  # {"hanging": x, "ground": y}
        
        # Runtime variables
        self.actual_position = init_position  # Set to initial position at start
        self.set_position = None  # Target position in radians
        self.calculated_pulse = None  # Calculated pulse width for movement
    
    def __repr__(self):
        return f"Servo(pin={self.pin}, length={self.length_mm}mm, angle_range=[{self.min_angle}, {self.max_angle}], actual_position={self.actual_position})"

class Leg:
    """Class representing a single leg of a Hexapod."""
    def __init__(self, name, position, orientation, segments):
        self.name = name
        self.position = position
        self.orientation = orientation
        self.coxa = Servo(**segments["coxa"])
        self.femur = Servo(**segments["femur"])
        self.tibia = Servo(**segments["tibia"])

    def __repr__(self):
        return f"Leg(name={self.name}, pos={self.position}, orientation={self.orientation})"

class Hexapod:
    """Class representing the Hexapod robot."""
    def __init__(self, config_file, init_mode="ground"):
        # Load JSON configuration file
        with open(config_file, "r") as file:
            config = json.load(file)

        # Store Hexapod body data
        self.position = config["hexapod"]["body"]["frame"]["position"]
        self.orientation = config["hexapod"]["body"]["frame"]["orientation"]
        self.legs = {}

        # Load legs from configuration
        for leg_name, leg_data in config["hexapod"]["legs"].items():
            self.legs[leg_name] = Leg(
                name=leg_name,
                position=leg_data["frame"]["position"],
                orientation=leg_data["frame"]["orientation"],
                segments={
                    "coxa": {
                        **leg_data["segments"]["coxa"],
                        "init_position": leg_data["segments"]["coxa"]["init_position"][init_mode]
                    },
                    "femur": {
                        **leg_data["segments"]["femur"],
                        "init_position": leg_data["segments"]["femur"]["init_position"][init_mode]
                    },
                    "tibia": {
                        **leg_data["segments"]["tibia"],
                        "init_position": leg_data["segments"]["tibia"]["init_position"][init_mode]
                    }
                }
            )

    def __repr__(self):
        return f"Hexapod(position={self.position}, orientation={self.orientation}, legs={list(self.legs.keys())})"


# Example: Load Hexapod object from JSON configuration
if __name__ == "__main__":
    hexapod = Hexapod("configuration.json", init_mode="hanging")

    # Print loaded Hexapod data
    print(hexapod)
    for leg_name, leg in hexapod.legs.items():
        print(leg)
        print(f"  Coxa:  {leg.coxa}")
        print(f"  Femur: {leg.femur}")
        print(f"  Tibia: {leg.tibia}")
