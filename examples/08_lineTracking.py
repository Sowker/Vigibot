import time
import argparse
from gpiozero import InputDevice

class Color:
    BLACK_LINE = '\033[92m'  
    FLOOR = '\033[90m'
    ACTION_GO = '\033[92m'
    ACTION_TURN = '\033[93m'
    ACTION_WARN = '\033[91m'
    END = '\033[0m'


class LineTrackingModule:
    def __init__(self, pin_left=22, pin_middle=27, pin_right=17):
        """Initializes the line tracking module with specified GPIO pins.
        0 means black line detected, 1 means white/floor background.
        
        Args:
            pin_left (int, optional): GPIO pin for Left sensor. Defaults to 22.
            pin_middle (int, optional): GPIO pin for Middle sensor. Defaults to 27.
            pin_right (int, optional): GPIO pin for Right sensor. Defaults to 17.
        """
        self.left = InputDevice(pin=pin_left)
        self.middle = InputDevice(pin=pin_middle)
        self.right = InputDevice(pin=pin_right)

    def read(self) -> tuple[int, int, int]:
        """Reads the current status of the line tracking sensors.
        
        Returns:
            tuple: (left, middle, right) sensor statuses.
        """
        return self.left.value, self.middle.value, self.right.value

    def get_visual_bar(self) -> str:
        """Generates a colored, easy-to-read UI bar representing the sensors.
        Returns:
            str: A string visualizing the sensor states with colors.
        """
        def format_sensor(val):
            # 0 = Black Line (Represented by a bright green block)
            # 1 = White Floor (Represented by a dim gray block)
            return f"{Color.BLACK_LINE}█{Color.END}" if val == 0 else f"{Color.FLOOR}░{Color.END}"
        
        return f"[ L:{format_sensor(self.left.value)}  M:{format_sensor(self.middle.value)}  R:{format_sensor(self.right.value)} ]"


def parse_arguments() -> argparse.Namespace:
    """Handles command-line argument parsing for Line Tracking.
    
    Returns:
        argparse.Namespace: Parsed command-line arguments.
    """
    parser = argparse.ArgumentParser(
        description="Line tracking automation script for infrared sensor array (0 = Black Line)."
    )
    parser.add_argument('--left', type=int, default=22, help="GPIO pin for Left sensor (default: 22)")
    parser.add_argument('--middle', type=int, default=27, help="GPIO pin for Middle sensor (default: 27)")
    parser.add_argument('--right', type=int, default=17, help="GPIO pin for Right sensor (default: 17)")
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_arguments()
    line_tracking = LineTrackingModule(pin_left=args.left, pin_middle=args.middle, pin_right=args.right)
    
    print("Starting line tracking system... Press Ctrl+C to stop.\n")
    
    try:
        while True:
            status_left, status_middle, status_right = line_tracking.read()
            sensor_visual = line_tracking.get_visual_bar()
            
            # Determine action based on sensor readings
            if status_left == 1 and status_middle == 0 and status_right == 1:
                action = f"{Color.ACTION_GO}Go Straight{Color.END}"
                
            elif status_left == 0 and status_middle == 1 and status_right == 1:
                action = f"{Color.ACTION_TURN}Turn Left ←{Color.END}"
                
            elif status_left == 1 and status_middle == 1 and status_right == 0:
                action = f"{Color.ACTION_TURN}Turn Right →{Color.END}"
                
            elif status_left == 0 and status_middle == 0 and status_right == 1:
                action = f"{Color.ACTION_TURN}Slight Left (Left + Middle){Color.END}"
                
            elif status_left == 1 and status_middle == 0 and status_right == 0:
                action = f"{Color.ACTION_TURN}Slight Right (Middle + Right){Color.END}"
                
            elif status_left == 0 and status_middle == 0 and status_right == 0:
                action = f"{Color.ACTION_GO}Intersection / Crossroad{Color.END}"
                
            else:            
                action = f"{Color.ACTION_WARN}Waiting for line / Lost...{Color.END}"
            
            # Print everything on a single line
            raw_data = f"(L:{status_left} M:{status_middle} R:{status_right})"
            print(f"{sensor_visual} {raw_data:<14} -> {action}")
            
            time.sleep(0.1)
        
    except KeyboardInterrupt:
        print(f"\nProgram terminated. Goodbye!")
        print("Program developed by Team C - MasterCamp SE 2026.")
