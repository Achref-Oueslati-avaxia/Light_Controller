/*
 * STM32F103 Robot State LED Controller (Serial commands via pyserial)
 * 
 * Hardware Setup:
 * - STM32F103C8T6 (Blue Pill board)
 * - 4x WS2812B LED strips (8 LEDs each = 32 total LEDs)
 * - 5V power supply for the LEDs (STM32 runs on 3.3V)
 * - USB-to-Serial adapter for communication with PC
 * 
 * Wiring:
 * PA0 → Back right wheel (Strip 0)
 * PA1 → Back left wheel  (Strip 1)
 * PA2 → Front right wheel (Strip 2)
 * PA3 → Front left wheel  (Strip 3)
 * GND → All LED strip grounds + power supply ground
 * 5V  → All LED strip power (NOT to STM32 - it has its own 3.3V)
 */

// ============================================================================
// INCLUDES
// ============================================================================
#define FASTLED_ALLOW_INTERRUPTS 0
#define FASTLED_FORCE_SOFTWARE_SPI 1
#define FASTLED_STM32 1
#include <FastLED.h>
#include <stdio.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================
#define NUM_STRIPS 4
#define LEDS_PER_STRIP 8
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define LED_BRIGHTNESS 100

#define LED_PIN_0 PA0 // Back right wheel
#define LED_PIN_1 PA1 // Back left wheel
#define LED_PIN_2 PA2 // Front right wheel
#define LED_PIN_3 PA3 // Front left wheel

// ============================================================================
// LED ARRAYS
// ============================================================================
CRGB leds_0[LEDS_PER_STRIP];
CRGB leds_1[LEDS_PER_STRIP];
CRGB leds_2[LEDS_PER_STRIP];
CRGB leds_3[LEDS_PER_STRIP];
CRGB* led_strips[NUM_STRIPS] = {leds_0, leds_1, leds_2, leds_3};

// ============================================================================
// STATE VARIABLES
// ============================================================================
uint8_t current_state = 0;
unsigned long last_update = 0;
uint8_t animation_step = 0;

#define STATE_OFF 0
#define STATE_LOW_BATTERY 1
#define STATE_RUNNING 2
#define STATE_CHARGING 3
#define STATE_ERROR 4
#define STATE_CHARGER_PLUGGED 5
#define STATE_CHARGER_UNPLUGGED 6
#define STATE_CHARGING_COMPLETE 7
#define STATE_SLEEP 8
#define STATE_MOVING_FORWARD 9
#define STATE_MOVING_BACKWARD 10
#define STATE_ROTATE_LEFT 11
#define STATE_ROTATE_RIGHT 12
#define STATE_FORWARD_LEFT 13
#define STATE_FORWARD_RIGHT 14
#define STATE_BACKWARD_LEFT 15
#define STATE_BACKWARD_RIGHT 16
#define STATE_SHUTDOWN 17
#define STATE_STARTUP 18

// ============================================================================
// ERROR INDICATION FUNCTION
// ============================================================================
void error_loop() {
  while(1) {
    for(int i = 0; i < NUM_STRIPS; i++) fill_solid(led_strips[i], LEDS_PER_STRIP, CRGB::Red);
    FastLED.show();
    delay(100);
    for(int i = 0; i < NUM_STRIPS; i++) fill_solid(led_strips[i], LEDS_PER_STRIP, CRGB::Black);
    FastLED.show();
    delay(100);
  }
}

// ============================================================================
// LED INITIALIZATION
// ============================================================================
void init_leds() {
  FastLED.addLeds<LED_TYPE, LED_PIN_0, COLOR_ORDER>(leds_0, LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, LED_PIN_1, COLOR_ORDER>(leds_1, LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, LED_PIN_2, COLOR_ORDER>(leds_2, LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, LED_PIN_3, COLOR_ORDER>(leds_3, LEDS_PER_STRIP).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(LED_BRIGHTNESS);
  startup_animation();
}

// ============================================================================
// STARTUP ANIMATION
// ============================================================================
void startup_animation() {
  for(int j = 0; j < 256; j++) {
    for(int strip = 0; strip < NUM_STRIPS; strip++) {
      for(int i = 0; i < LEDS_PER_STRIP; i++) {
        led_strips[strip][i] = CHSV(j + (i * 32), 255, 255);
      }
    }
    FastLED.show();
    delay(10);
  }
  FastLED.clear();
  FastLED.show();
}

// ============================================================================
// SERIAL COMMAND PARSER
// ============================================================================
void parse_serial_command(char* command) {
  if(strcmp(command, "low_battery") == 0 || strcmp(command, "yellow_blink") == 0) current_state = STATE_LOW_BATTERY;
  else if(strcmp(command, "green_solid") == 0 || strcmp(command, "running") == 0) current_state = STATE_RUNNING;
  else if(strcmp(command, "blue_chase") == 0 || strcmp(command, "charging") == 0) current_state = STATE_CHARGING;
  else if(strcmp(command, "off") == 0 || strcmp(command, "idle") == 0) current_state = STATE_OFF;
  else if(strcmp(command, "red_blink") == 0 || strcmp(command, "error") == 0) current_state = STATE_ERROR;
  else if(strcmp(command, "white_solid") == 0 || strcmp(command, "shutdown") == 0) current_state = STATE_SHUTDOWN;
  else if(strcmp(command, "rainbow") == 0 || strcmp(command, "startup") == 0) current_state = STATE_STARTUP;
  else if(strcmp(command, "blue_pulse") == 0 || strcmp(command, "charger_plugged") == 0) current_state = STATE_CHARGER_PLUGGED;
  else if(strcmp(command, "white_flash") == 0 || strcmp(command, "charger_unplugged") == 0) current_state = STATE_CHARGER_UNPLUGGED;
  else if(strcmp(command, "green_sweep") == 0 || strcmp(command, "charging_complete") == 0) current_state = STATE_CHARGING_COMPLETE;
  else if(strcmp(command, "breathing_blue") == 0 || strcmp(command, "sleep") == 0) current_state = STATE_SLEEP;
  else if(strcmp(command, "forward") == 0 || strcmp(command, "moving_forward") == 0) current_state = STATE_MOVING_FORWARD;
  else if(strcmp(command, "backward") == 0 || strcmp(command, "moving_backward") == 0) current_state = STATE_MOVING_BACKWARD;
  else if(strcmp(command, "rotate_left") == 0) current_state = STATE_ROTATE_LEFT;
  else if(strcmp(command, "rotate_right") == 0) current_state = STATE_ROTATE_RIGHT;
  else if(strcmp(command, "forward_left") == 0) current_state = STATE_FORWARD_LEFT;
  else if(strcmp(command, "forward_right") == 0) current_state = STATE_FORWARD_RIGHT;
  else if(strcmp(command, "backward_left") == 0) current_state = STATE_BACKWARD_LEFT;
  else if(strcmp(command, "backward_right") == 0) current_state = STATE_BACKWARD_RIGHT;
}

// ============================================================================
// DISPLAY LOGIC
// ============================================================================
void display_state() {
  switch(current_state) {
    case STATE_OFF:
      for(int i=0;i<NUM_STRIPS;i++) fill_solid(led_strips[i], LEDS_PER_STRIP, CRGB::Black);
      break;
    case STATE_LOW_BATTERY:
      blink_all_strips(CRGB::Yellow, 300);
      break;
    case STATE_ERROR:
      blink_all_strips(CRGB::Red, 300);
      break;
    case STATE_RUNNING:
      for(int i=0;i<NUM_STRIPS;i++) fill_solid(led_strips[i], LEDS_PER_STRIP, CRGB::Green);
      break;
    case STATE_CHARGING:
      chase_all_strips(CRGB::Blue, 150);
      break;
    case STATE_CHARGER_PLUGGED:
      white_flash_animation();
      current_state = STATE_CHARGING;
      display_state(); // Immediately start blue chase animation
      break;
    case STATE_CHARGER_UNPLUGGED:
      white_flash_animation();
      current_state = STATE_SLEEP;
      break;
    case STATE_CHARGING_COMPLETE:
      green_sweep_animation();
      current_state = STATE_SLEEP;
      break;
    case STATE_SLEEP:
      breathing_blue_animation(); break;
    case STATE_MOVING_FORWARD:
      sweep_strips(2, 3, CRGB::Green); break; // PA2, PA3 (front)
    case STATE_MOVING_BACKWARD:
      sweep_strips(0, 1, CRGB::Green); break; // PA0, PA1 (back)
    case STATE_ROTATE_LEFT:
      blink_strips(3, 1, CRGB::Yellow); fill_solid(led_strips[2], LEDS_PER_STRIP, CRGB::Black); fill_solid(led_strips[0], LEDS_PER_STRIP, CRGB::Black); break;
    case STATE_ROTATE_RIGHT:
      blink_strips(2, 0, CRGB::Yellow); fill_solid(led_strips[3], LEDS_PER_STRIP, CRGB::Black); fill_solid(led_strips[1], LEDS_PER_STRIP, CRGB::Black); break;
    case STATE_FORWARD_LEFT:
      fill_solid(led_strips[3], LEDS_PER_STRIP, CRGB::Green); fill_solid(led_strips[2], LEDS_PER_STRIP, CRGB::Green); blink_strip(1, CRGB::Yellow); fill_solid(led_strips[0], LEDS_PER_STRIP, CRGB::Black); break;
    case STATE_FORWARD_RIGHT:
      fill_solid(led_strips[3], LEDS_PER_STRIP, CRGB::Green); fill_solid(led_strips[2], LEDS_PER_STRIP, CRGB::Green); blink_strip(0, CRGB::Yellow); fill_solid(led_strips[1], LEDS_PER_STRIP, CRGB::Black); break;
    case STATE_BACKWARD_LEFT:
      fill_solid(led_strips[1], LEDS_PER_STRIP, CRGB::Green); fill_solid(led_strips[0], LEDS_PER_STRIP, CRGB::Green); blink_strip(3, CRGB::Yellow); fill_solid(led_strips[2], LEDS_PER_STRIP, CRGB::Black); break;
    case STATE_BACKWARD_RIGHT:
      fill_solid(led_strips[1], LEDS_PER_STRIP, CRGB::Green); fill_solid(led_strips[0], LEDS_PER_STRIP, CRGB::Green); blink_strip(2, CRGB::Yellow); fill_solid(led_strips[3], LEDS_PER_STRIP, CRGB::Black); break;
    case STATE_SHUTDOWN:
      for(int i=0;i<NUM_STRIPS;i++) fill_solid(led_strips[i], LEDS_PER_STRIP, CRGB::White);
      FastLED.show(); delay(1000);
      for(int i=0;i<NUM_STRIPS;i++) fill_solid(led_strips[i], LEDS_PER_STRIP, CRGB::Black);
      current_state = STATE_OFF;
      break;
    case STATE_STARTUP:
      startup_animation();
      for(int i=0;i<NUM_STRIPS;i++) fill_solid(led_strips[i], LEDS_PER_STRIP, CRGB::Green);
      current_state = STATE_RUNNING;
      break;
  }
  FastLED.show();
}

// ============================================================================
// SIMPLE ANIMATION PATTERNS
// ============================================================================
void blue_pulse_animation() {
  static uint8_t pulse = 0;
  static bool up = true;
  uint8_t brightness = pulse;
  for(int strip = 0; strip < NUM_STRIPS; strip++) fill_solid(led_strips[strip], LEDS_PER_STRIP, CRGB(0,0,brightness));
  FastLED.show();
  delay(20);
  if(up) pulse += 10; else pulse -= 10;
  if(pulse >= 200) up = false;
  if(pulse <= 10) up = true;
}

void white_flash_animation() {
  for(int strip = 0; strip < NUM_STRIPS; strip++) fill_solid(led_strips[strip], LEDS_PER_STRIP, CRGB::White);
  FastLED.show();
  delay(200);
  for(int strip = 0; strip < NUM_STRIPS; strip++) fill_solid(led_strips[strip], LEDS_PER_STRIP, CRGB::Black);
  FastLED.show();
}

void green_sweep_animation() {
  for(int i = 0; i < LEDS_PER_STRIP; i++) {
    for(int strip = 0; strip < NUM_STRIPS; strip++) {
      fill_solid(led_strips[strip], LEDS_PER_STRIP, CRGB::Black);
      led_strips[strip][i] = CRGB::Green;
    }
    FastLED.show();
    delay(60);
  }
  delay(500);
  for(int strip = 0; strip < NUM_STRIPS; strip++) fill_solid(led_strips[strip], LEDS_PER_STRIP, CRGB::Black);
  FastLED.show();
}

void breathing_blue_animation() {
  static uint8_t breath = 0;
  static bool up = true;
  uint8_t brightness = breath;
  for(int strip = 0; strip < NUM_STRIPS; strip++) fill_solid(led_strips[strip], LEDS_PER_STRIP, CRGB(0,0,brightness));
  FastLED.show();
  delay(30);
  if(up) breath += 5; else breath -= 5;
  if(breath >= 100) up = false;
  if(breath <= 10) up = true;
}

// Helper sweep and blink functions
void sweep_strips(int s1, int s2, CRGB color) {
  for(int i = 0; i < LEDS_PER_STRIP; i++) {
    fill_solid(led_strips[s1], LEDS_PER_STRIP, CRGB::Black);
    fill_solid(led_strips[s2], LEDS_PER_STRIP, CRGB::Black);
    led_strips[s1][i] = color;
    led_strips[s2][i] = color;
    FastLED.show();
    delay(60);
  }
  for(int strip = 0; strip < NUM_STRIPS; strip++) fill_solid(led_strips[strip], LEDS_PER_STRIP, CRGB::Black);
  FastLED.show();
}

void blink_strips(int s1, int s2, CRGB color) {
  static bool on = true;
  if(on) {
    fill_solid(led_strips[s1], LEDS_PER_STRIP, color);
    fill_solid(led_strips[s2], LEDS_PER_STRIP, color);
  } else {
    fill_solid(led_strips[s1], LEDS_PER_STRIP, CRGB::Black);
    fill_solid(led_strips[s2], LEDS_PER_STRIP, CRGB::Black);
  }
  FastLED.show();
  delay(150);
  on = !on;
}

void blink_strip(int s, CRGB color) {
  static bool on = true;
  if(on) fill_solid(led_strips[s], LEDS_PER_STRIP, color);
  else fill_solid(led_strips[s], LEDS_PER_STRIP, CRGB::Black);
  FastLED.show();
  delay(150);
  on = !on;
}

void blink_all_strips(CRGB color, uint16_t speed) {
  if(millis() - last_update > speed) {
    for(int strip = 0; strip < NUM_STRIPS; strip++) {
      if(animation_step % 2 == 0) fill_solid(led_strips[strip], LEDS_PER_STRIP, color);
      else fill_solid(led_strips[strip], LEDS_PER_STRIP, CRGB::Black);
    }
    animation_step++;
    last_update = millis();
  }
}

void chase_all_strips(CRGB color, uint16_t speed) {
  if(millis() - last_update > speed) {
    for(int strip = 0; strip < NUM_STRIPS; strip++) {
      fill_solid(led_strips[strip], LEDS_PER_STRIP, CRGB::Black);
      led_strips[strip][animation_step % LEDS_PER_STRIP] = color;
    }
    animation_step++;
    last_update = millis();
  }
}

// ============================================================================
// ARDUINO SETUP FUNCTION
// ============================================================================
void setup() {
  Serial.begin(115200);
  init_leds();
  Serial.println("STM32 LED Controller Ready. Waiting for serial commands...");
}

// ============================================================================
// ARDUINO LOOP FUNCTION
// ============================================================================
void loop() {
  static char serial_buffer[64];
  static uint8_t buffer_pos = 0;
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (buffer_pos > 0) {
        serial_buffer[buffer_pos] = '\0';
        parse_serial_command(serial_buffer);
        buffer_pos = 0;
      }
    } else {
      if (buffer_pos < sizeof(serial_buffer) - 1) serial_buffer[buffer_pos++] = inChar;
    }
  }
  display_state();
  delay(50);
}