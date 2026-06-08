import argparse
from time import sleep
from gpiozero import DistanceSensor

class DistanceColor:
    FAR = '\033[92m'
    OK = '\033[93m'
    NEAR = '\033[91m'
    END = '\033[0m'


class RobotSensor:
    """Class encapsulating the hardware logic of the ultrasonic sensor."""
    
    def __init__(self, trigger_pin=23, echo_pin=24, max_distance_m=2.0):
        """Initializes the ultrasonic sensor with specified GPIO pins and maximum distance.
        
        Args:
            trigger_pin (int, optional): GPIO pin for Trigger. Defaults to 23.
            echo_pin (int, optional): GPIO pin for Echo. Defaults to 24.
            max_distance_m (float, optional): Maximum detection distance in meters. Defaults to 2.0.
        """
        self.device = DistanceSensor(echo=echo_pin, trigger=trigger_pin, max_distance=max_distance_m)
        self.max_distance_mm = max_distance_m * 1000

    def read_distance_mm(self):
        """Returns the current distance converted to millimeters.
        Returns:
            float: Distance in millimeters.
        """
        return self.device.distance * 1000


def parse_arguments():
    """Handles command-line argument parsing.
    Returns:
        argparse.Namespace: Parsed command-line arguments.
    """
    parser = argparse.ArgumentParser(
        description="Calculate an object's position relative to the robot using an Ultrasonic sensor."
    )
    parser.add_argument('--trigger', type=int, default=23, help="GPIO pin for Trigger (default: 23)")
    parser.add_argument('--echo', type=int, default=24, help="GPIO pin for Echo (default: 24)")
    parser.add_argument('--max-dist', type=float, default=2.0, help="Maximum detection distance in meters (default: 2.0)")
    parser.add_argument('--level1', type=float, default=70.0, help="Level 1 threshold (Red alert) in mm (default: 70.0)")
    parser.add_argument('--level2', type=float, default=100.0, help="Level 2 threshold (Yellow alert) in mm (default: 100.0)")
    parser.add_argument('--interval', type=float, default=0.05, help="Measurement interval in seconds (default: 0.05)")
    
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    sensor = RobotSensor(
        trigger_pin=args.trigger, 
        echo_pin=args.echo, 
        max_distance_m=args.max_dist
    )

    # External Main Loop
    try:
        print("Starting continuous measurements... (Press Ctrl+C to stop)")
        while True:
            # Calling the class method outside
            distance = sensor.read_distance_mm()
            
            # Display logic based on the read distance
            if distance > sensor.max_distance_mm:
                print(f"{DistanceColor.FAR}No object detected within {sensor.max_distance_mm:.2f} mm{DistanceColor.END}")
            elif distance > args.level2:
                print(f"{DistanceColor.FAR}Object located at: {distance:.2f} mm{DistanceColor.END}")
            elif distance > args.level1:
                print(f"{DistanceColor.OK}Object located at: {distance:.2f} mm{DistanceColor.END}")
            else:
                print(f"{DistanceColor.NEAR}Object located at: {distance:.2f} mm{DistanceColor.END}")
                
            sleep(args.interval)
            
    except KeyboardInterrupt:
        print("\nProgram terminated. Goodbye!")
        print("Program developed by Team C - MasterCamp SE 2026.")
