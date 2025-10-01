# Robot State LED Controller

This repository provides visual feedback for robot system states using four WS2812B LED strips (8 LEDs each), controlled by an STM32F103 (Blue Pill) and ROS2 Python scripts. The project includes state-based LED animations and a serial bridge for ROS2 integration.

## Features
- LED patterns for various robot states: IDLE, RUNNING, ERROR, LOW_BATTERY, CHARGING, CHARGER_PLUGGED, CHARGER_UNPLUGGED, CHARGING_COMPLETE, SHUTDOWN, STARTUP, SLEEP, MOVING_FORWARD, MOVING_BACKWARD, ROTATE_LEFT, ROTATE_RIGHT, FORWARD_LEFT, FORWARD_RIGHT, BACKWARD_LEFT, BACKWARD_RIGHT
- ROS2 node (`state_led_bridge.py`) listens to `/OUTLANDER_state` and sends serial commands to the STM32
- STM32 parses serial commands and updates LED strips
- Interactive CLI (`state_controller.py`) for sending robot states

## Hardware Setup
- **Board:** STM32F103C8T6 (Blue Pill)
- **LEDs:** Four WS2812B strips (8 LEDs each)
- **Power:** 5V for LEDs, 3.3V for STM32
- **Serial:** USB-to-Serial (FTDI) adapter

### Wiring
| STM32 Pin | LED Strip | Description         |
|-----------|-----------|---------------------|
| PA0       | Strip 0   | Data line           |
| PA1       | Strip 1   | Data line           |
| PA2       | Strip 2   | Data line           |
| PA3       | Strip 3   | Data line           |
| GND       | All       | Common ground       |
| 5V        | All       | LED power (not STM32)|

**Note:** Connect all grounds together. Do not connect 5V to STM32, only to LEDs.

## STM32 Firmware
- Location: `src/robot_led_control/firmware/stm32_led_controller/stm32_led_controller.ino`
- Use Arduino IDE
- Install FastLED library
- Board: Generic STM32F1 series (BluePill F103C8)
- Upload method: Serial
- Port: `/dev/ttyUSB0` (or `/dev/ttyACM0`)
- Upload using FTDI adapter

## ROS2 Python Scripts
- `state_led_bridge.py`: Listens to `/OUTLANDER_state` and sends mapped serial commands to STM32
- `state_controller.py`: Interactive CLI to send robot states to `/OUTLANDER_state`

## Building
### ROS2 Package
```bash
colcon build --packages-select robot_led_control
source install/setup.bash
```

### STM32 Firmware
- Open `src/robot_led_control/firmware/stm32_led_controller/stm32_led_controller.ino` in Arduino IDE
- Build and upload to the STM32 board

## Running
### 1. Start the Serial Bridge
```bash
cd src/robot_led_control/scripts

python3 state_led_bridge.py --port /dev/ttyUSB0
```

### 2. Control LED States via CLI
```bash
python3 state_controller.py
```
Supported states:
- IDLE
- RUNNING
- ERROR
- LOW_BATTERY
- CHARGING
- CHARGER_PLUGGED
- CHARGER_UNPLUGGED
- CHARGING_COMPLETE
- SHUTDOWN
- STARTUP
- SLEEP
- MOVING_FORWARD
- MOVING_BACKWARD
- ROTATE_LEFT
- ROTATE_RIGHT
- FORWARD_LEFT
- FORWARD_RIGHT
- BACKWARD_LEFT
- BACKWARD_RIGHT
- help
- exit

### 3. (Optional) Publish State Manually
```bash
ros2 topic pub /OUTLANDER_state std_msgs/String "data: 'ERROR'"
ros2 topic pub /OUTLANDER_state std_msgs/String "data: 'RUNNING'"
# ...etc
```

## LED Pattern Mapping
| State              | LED Pattern                        |
|--------------------|------------------------------------|
| IDLE               | All LEDs off                       |
| RUNNING            | All LEDs solid green               |
| ERROR              | All LEDs blinking red              |
| LOW_BATTERY        | All LEDs blinking yellow           |
| CHARGING           | Blue chase animation               |
| CHARGER_PLUGGED    | Quick white flash, then CHARGING   |
| CHARGER_UNPLUGGED  | Quick white flash, then SLEEP      |
| CHARGING_COMPLETE  | Green sweep, then SLEEP            |
| SHUTDOWN           | All LEDs solid white, then off     |
| STARTUP            | Rainbow animation, then green      |
| SLEEP              | Breathing blue animation           |
| MOVING_FORWARD     | Green sweep (front strips)         |
| MOVING_BACKWARD    | Green sweep (back strips)          |
| ROTATE_LEFT        | Yellow blink (left strips)         |
| ROTATE_RIGHT       | Yellow blink (right strips)        |
| FORWARD_LEFT       | Green + yellow blink (front left)  |
| FORWARD_RIGHT      | Green + yellow blink (front right) |
| BACKWARD_LEFT      | Green + yellow blink (back left)   |
| BACKWARD_RIGHT     | Green + yellow blink (back right)  |

## Troubleshooting
- Only one program can access the serial port at a time
- If LED states do not update, check the serial connection and STM32 debug output
- For ROS2 issues, verify the environment is sourced and dependencies are installed

## File Structure
- `src/robot_led_control/firmware/stm32_led_controller/stm32_led_controller.ino` — STM32 firmware
- `src/robot_led_control/scripts/state_led_bridge.py` — ROS2 state bridge
- `src/robot_led_control/scripts/state_controller.py` — Interactive ROS2 state controller

## License
MIT
