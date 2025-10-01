#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

STATE_LIST = [
    'IDLE',
    'RUNNING',
    'ERROR',
    'LOW_BATTERY',
    'CHARGING',
    'CHARGER_PLUGGED',  # Quick white flash, then CHARGING
    'CHARGER_UNPLUGGED', # Quick white flash, then SLEEP
    'CHARGING_COMPLETE', # Green sweep, then SLEEP
    'SHUTDOWN',
    'STARTUP',
    'SLEEP',
    'MOVING_FORWARD',
    'MOVING_BACKWARD',
    'ROTATE_LEFT',
    'ROTATE_RIGHT',
    'FORWARD_LEFT',
    'FORWARD_RIGHT',
    'BACKWARD_LEFT',
    'BACKWARD_RIGHT'
]

class StateController(Node):
    def __init__(self):
        super().__init__('state_controller')
        self.state_publisher = self.create_publisher(String, '/OUTLANDER_state', 10)
        self.get_logger().info('State Controller started')
        self.print_help()

    def print_help(self):
        print("\n=== Robot State Controller ===")
        print("States:")
        for state in STATE_LIST:
            print(f"  {state}")
        print("  help         - Show this help")
        print("  exit         - Exit program")
        print()

    def send_state(self, state):
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)
        self.get_logger().info(f'Sent state: {state}')

    def process_command(self, user_input):
        cmd = user_input.strip().upper()
        if cmd == 'HELP':
            self.print_help()
        elif cmd == 'EXIT':
            return False
        elif cmd in STATE_LIST:
            self.send_state(cmd)
        else:
            self.get_logger().error(f'Unknown state: {cmd}. Type \"help\" for available states.')
        return True

def main(args=None):
    rclpy.init(args=args)
    controller = StateController()
    try:
        while True:
            user_input = input("\nRobot state (help for options): ")
            if not controller.process_command(user_input):
                break
            rclpy.spin_once(controller, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\nShutting down...")
    except EOFError:
        print("\nShutting down...")
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()