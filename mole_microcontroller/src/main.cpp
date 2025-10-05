#include <Arduino.h>
#include "hardware_msgs.hpp"

#define LED_BUILTIN 2  // Pin 2

// --- Serial Communication Definitions ---
#define SERIAL_BAUDRATE 115200
#define SERIAL_TIMEOUT_MS 100

#define HEADER1 0xAA
#define HEADER2 0xBB
#define TYPE_VELOCITY 0x00
#define TYPE_ENCODER 0x01
#define TYPE_ENCODER_REQUEST 0x02
#define TYPE_PID 0x03

#define VELOCITY_PAYLOAD_SIZE 8
#define ENCODER_PAYLOAD_SIZE 8
#define ENCODER_REQUEST_PAYLOAD_SIZE 0
#define PID_PAYLOAD_SIZE 16

// --- Robot Pins --- Examples for now until I get actual motors hooked up
#define MOTOR_LEFT_PWM_PIN 4
#define MOTOR_LEFT_DIR_PIN 5
#define MOTOR_RIGHT_PWM_PIN 6
#define MOTOR_RIGHT_DIR_PIN 7
#define ENCODER_LEFT_PIN_A 2
#define ENCODER_LEFT_PIN_B 3
#define ENCODER_RIGHT_PIN_A 8
#define ENCODER_RIGHT_PIN_B 9

// --- Encoder
volatile int32_t left_encoder_count = 0;
volatile int32_t right_encoder_count = 0;

void IRAM_ATTR left_encoder_isr() {
  if (digitalRead(ENCODER_LEFT_PIN_B) == HIGH) {
    left_encoder_count++;
  } else {
    left_encoder_count--;
  }
}

void IRAM_ATTR right_encoder_isr() {
  if (digitalRead(ENCODER_RIGHT_PIN_B) == HIGH) {
    right_encoder_count++;
  } else {
    right_encoder_count--;
  }
}

// --- Control Variables ---
#define LOOP_RATE_HZ 50
const float LOOP_PERIOD = 1.0 / LOOP_RATE_HZ;

// --- PID Control Variables --- Will get set via serial
float pid_p = 1.0;
float pid_i = 0.0;
float pid_d = 0.0;
float pid_o = 100.0;

// --- State ---
float left_integral = 0.0, right_integral = 0.0;
float left_prev_error = 0.0, right_prev_error = 0.0;
float target_ticks_left = 0.0, target_ticks_right = 0.0;

// --- Serial Functions ---

bool read_packet(Packet& pkt);
void handle_packet(const Packet& pkt);
void send_encoder_data(int32_t left_encoder, int32_t right_encoder);

// --- Control Functions ---

void set_pid_settings(int32_t p, int32_t i, int32_t d, int32_t o);
void set_motor(int pin_pwm, int pin_dir, float value);
float compute_pid(float target, float current, float &integral, float &prev_error);

// --- Timer and control loop ---
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR control_loop();

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(SERIAL_TIMEOUT_MS);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);

  ledcAttachPin(MOTOR_LEFT_PWM_PIN, 0);
  ledcSetup(0, 5000, 8); // Channel 0,
  ledcAttachPin(MOTOR_RIGHT_PWM_PIN, 1);
  ledcSetup(1, 5000, 8); // Channel 1,
  pinMode(ENCODER_LEFT_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN_A), left_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN_A), right_encoder_isr, RISING);

  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1 MHz), count up, default so 1 tick = 1 µs
  timerAttachInterrupt(timer, &control_loop, true);
  timerAlarmWrite(timer, LOOP_PERIOD * 1000000, true); // Convert s to µs
  timerAlarmEnable(timer);

  for (int i = 0; i < 3; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void loop() {
  Packet pkt;
  if (read_packet(pkt)) {
    handle_packet(pkt);
  }
}

bool read_packet(Packet& pkt) {
  static enum { WAIT_HEADER1, WAIT_HEADER2, WAIT_TYPE, WAIT_PAYLOAD, WAIT_CHECKSUM } state = WAIT_HEADER1;
  static size_t index = 0;
  static size_t payload_length = 0;

  while (Serial.available()) {
    uint8_t byte = Serial.read();
    switch (state) {
      case WAIT_HEADER1:
        if (byte == HEADER1) state = WAIT_HEADER2;
        break;
      case WAIT_HEADER2:
        if (byte == HEADER2) state = WAIT_TYPE;
        else state = WAIT_HEADER1;
        break;
      case WAIT_TYPE:
        pkt.type = byte;
        state = WAIT_PAYLOAD;
        index = 0;
        payload_length = 0;
        switch (pkt.type) {
          case TYPE_VELOCITY:
            payload_length = VELOCITY_PAYLOAD_SIZE;
            break;
          case TYPE_ENCODER:
            payload_length = ENCODER_PAYLOAD_SIZE;
            break;
          case TYPE_ENCODER_REQUEST:
            payload_length = ENCODER_REQUEST_PAYLOAD_SIZE;
            break;
          case TYPE_PID:
            payload_length = PID_PAYLOAD_SIZE;
            break;
        }
        break;
      case WAIT_PAYLOAD:
        if (index < payload_length) {
          pkt.payload[index++] = byte;
        } else {
          state = WAIT_CHECKSUM;
        }
        break;
      case WAIT_CHECKSUM:
        pkt.checksum = byte;
        // Validate checksum
        if (calculate_checksum((uint8_t *)&pkt, 2 + 1 + payload_length) == pkt.checksum) {
          state = WAIT_HEADER1;
          return true;
        } else {
          state = WAIT_HEADER1; // Invalid checksum, reset state
        }
        break;
    }
  }
  return false;
}

void handle_packet(const Packet& pkt) {
  switch (pkt.type) {
    case TYPE_VELOCITY: {
      // Set target velocities
      if (sizeof(VelocityPacket) - 1 != 2 + 1 + VELOCITY_PAYLOAD_SIZE) return; // Size mismatch guard for accidental struct change
      VelocityPacket v_pkt;
      memcpy(&v_pkt, pkt.payload, VELOCITY_PAYLOAD_SIZE);
      setVelocity(v_pkt.left_velocity, v_pkt.right_velocity);
      break;
    }
    case TYPE_ENCODER_REQUEST: {
      // Send current encoder values
      send_encoder_data(left_encoder_count, right_encoder_count);
      break;
    }
    case TYPE_PID: {
      // Set PID settings
      if (sizeof(PIDPacket) - 1 != 2 + 1 + PID_PAYLOAD_SIZE) return; // Size mismatch guard for accidental struct change
      PIDPacket p_pkt;
      memcpy(&p_pkt, pkt.payload, PID_PAYLOAD_SIZE);
      set_pid_settings(p_pkt.p, p_pkt.i, p_pkt.d, p_pkt.o);
      break;
    }
  }
}

void send_encoder_data(int32_t left_encoder, int32_t right_encoder) {
  EncoderPacket e_pkt;
  e_pkt.header1 = HEADER1;
  e_pkt.header2 = HEADER2;
  e_pkt.type = TYPE_ENCODER;
  e_pkt.left_encoder = left_encoder;
  e_pkt.right_encoder = right_encoder;
  e_pkt.checksum = calculate_checksum((uint8_t *)&e_pkt, sizeof(EncoderPacket) - 1);
  Serial.write((uint8_t *)&e_pkt, sizeof(EncoderPacket));
}

void set_pid_settings(int32_t p, int32_t i, int32_t d, int32_t o) {
  pid_p = static_cast<float>(p);
  pid_i = static_cast<float>(i);
  pid_d = static_cast<float>(d);
  pid_o = static_cast<float>(o);
}

void set_motor(int pin_pwm, int pin_dir, float value) {
  if (value >= 0) {
    digitalWrite(pin_dir, HIGH);
    ledcWrite(pin_pwm, static_cast<int>(value));
  } else {
    digitalWrite(pin_dir, LOW);
    ledcWrite(pin_pwm, static_cast<int>(-value));
  }
}

float compute_pid(float target, float current, float &integral, float &prev_error) {
  float error = target - current;
  integral += error * LOOP_PERIOD;
  integral = constrain(integral, -pid_o, pid_o); // Anti-windup
  float derivative = (error - prev_error) * LOOP_RATE_HZ;
  prev_error = error;
  float output = pid_o + pid_p * error + pid_i * integral + pid_d * derivative;
  return output;
}

void IRAM_ATTR control_loop() {
  portENTER_CRITICAL(&timerMux);
  // Read current encoder values
  int32_t current_ticks_left = left_encoder_count;
  int32_t current_ticks_right = right_encoder_count;

  // Compute PID outputs
  float left_output = compute_pid(target_ticks_left, current_ticks_left, left_integral, left_prev_error);
  float right_output = compute_pid(target_ticks_right, current_ticks_right, right_integral, right_prev_error);

  // Set motor speeds
  set_motor(MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN, left_output);
  set_motor(MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN, right_output);

  portEXIT_CRITICAL(&timerMux);
}