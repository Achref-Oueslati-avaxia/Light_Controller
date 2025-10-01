#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200

STATE_TO_COMMAND = {
    'IDLE': 'off',
    'RUNNING': 'green_solid',
    'ERROR': 'red_blink',
    'LOW_BATTERY': 'yellow_blink',
    'CHARGING': 'blue_chase',
    'CHARGER_PLUGGED': 'white_flash',  # Quick white flash, then CHARGING
    'CHARGER_UNPLUGGED': 'white_flash', # Quick white flash, then SLEEP
    'CHARGING_COMPLETE': 'green_sweep', # Green sweep, then SLEEP
    'SHUTDOWN': 'white_solid',
    'STARTUP': 'rainbow',
    'SLEEP': 'breathing_blue',
    'MOVING_FORWARD': 'forward',
    'MOVING_BACKWARD': 'backward',
    'ROTATE_LEFT': 'rotate_left',
    'ROTATE_RIGHT': 'rotate_right',
    'FORWARD_LEFT': 'forward_left',
    'FORWARD_RIGHT': 'forward_right',
    'BACKWARD_LEFT': 'backward_left',
    'BACKWARD_RIGHT': 'backward_right'
}

class StateLedBridge(Node):
    def __init__(self):
        super().__init__('state_led_bridge')
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        self.subscription = self.create_subscription(
            String,
            '/OUTLANDER_state',
            self.listener_callback,
            10)
        self.get_logger().info(f'State LED Bridge started on {SERIAL_PORT}, listening to /OUTLANDER_state')

    def listener_callback(self, msg):
        state = msg.data.strip().upper()
        command = STATE_TO_COMMAND.get(state)
        if command:
            self.get_logger().info(f'State: {state} → Command: {command}')
            self.ser.write((command + '\n').encode('utf-8'))
        else:
            self.get_logger().warn(f'Unknown state: {state} (no LED pattern mapped)')

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StateLedBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()