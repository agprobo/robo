import time
from inputs import get_gamepad, UnpluggedError
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Define joystick mappings
JOYSTICK_CODE_VALUE_MAP = {
    'Microsoft X-Box 360 pad': {
        'ABS_Y': ('linear.x', -32768, 32767),   # ABS_Y controls forward/backward
        'ABS_RX': ('angular.z', -32768, 32767)  # ABS_RX controls rotation
    },
    # Add other joysticks if needed
}

class JoystickRos2(Node):
    def __init__(self):
        super().__init__('joystick_ros2')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_vel = Twist()

    def normalize_value(self, min_val, max_val, value):
        """Normalize the joystick value to [-1, 1]"""
        normalized = (2 * (value - min_val) / (max_val - min_val)) - 1
        return normalized if abs(normalized) > 0.05 else 0.0

    def publish_cmd_vel(self):
        self.publisher_.publish(self.cmd_vel)
        print(f"Published cmd_vel: linear.x={self.cmd_vel.linear.x}, angular.z={self.cmd_vel.angular.z}")

    def run(self):
        print("Starting joystick input loop...")
        while rclpy.ok():
            try:
                events = get_gamepad()
                if not events:
                    time.sleep(0.1)
                    continue

                for event in events:
                    print(f"Event received: {event.code} ({event.ev_type}) with value {event.state}")

                    if event.ev_type == 'Absolute':
                        if event.code in JOYSTICK_CODE_VALUE_MAP['Microsoft X-Box 360 pad']:
                            axis, min_val, max_val = JOYSTICK_CODE_VALUE_MAP['Microsoft X-Box 360 pad'][event.code]
                            normalized_value = self.normalize_value(min_val, max_val, event.state)

                            if axis.startswith('linear'):
                                setattr(self.cmd_vel.linear, axis.split('.')[1], normalized_value)
                            elif axis.startswith('angular'):
                                setattr(self.cmd_vel.angular, axis.split('.')[1], normalized_value)

                            print(f"Updated cmd_vel: {axis}={normalized_value}")
                            self.publish_cmd_vel()

            except UnpluggedError:
                print("Joystick was unplugged. Waiting for replug...")
                time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    joystick_ros2 = JoystickRos2()
    try:
        joystick_ros2.run()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        joystick_ros2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
