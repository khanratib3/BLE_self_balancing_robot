#include <Arduino_BMI270_BMM150.h>  // IMU sensor library
#include <ArduinoBLE.h>             // BLE communication library
#include <string.h>                 // For string manipulation

#define BUFFER_SIZE 20  // Max BLE characteristic buffer size

// BLE service and characteristic setup (for PID and angle control)
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

// Motor driver pins
int A_IN1 = 2;
int A_IN2 = 3;
int B_IN1 = 5;
int B_IN2 = 4;

// Variables for PID control and timing
float delta_T;
float previous_time;
float current_time;
float response;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
float k_p = 3.3;
float k_i = 57.5;
float k_d = 0.38;
float err = 0.0;
float previous_err = 0;
float pwm;

float k = 0.95;  // Complementary filter constant
float x_accel, y_accel, z_accel, theta_accel;
float x_gyro, y_gyro, z_gyro, theta_gyro = 0;
float desired_angle = 0.85;
float tuned_desired_angle = 1.04;
float forward_desired_angle = 1.05;
float backward_desired_angle = 0.05;
float theta_comp = 0.0;
float right_wheel_adjustment = 1.105;

// Movement flags
int forward_flag = 0;
int backward_flag = 0;
int left_flag = 0;
int right_flag = 0;

// Timing for direction state machines
float forward_previous_time = 0.0;
float backward_previous_time = 0.0;
float left_previous_time = 0.0;
float right_previous_time = 0.0;

float forward_current_time;
float backward_current_time;
float left_current_time;
float right_current_time;

float forward_runtime = 0.0;
float backward_runtime = 0.0;
float left_runtime = 0.0;
float right_runtime = 0.0;

// Function declarations
void go_forward();
void go_backward();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("BLE-GROUP10");
  BLE.setDeviceName("BLE-GROUP10");
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);
  customCharacteristic.writeValue("Waiting for data");
  BLE.advertise();
  Serial.println("Bluetooth® device active, waiting for connections...");

  // Motor pins setup
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);

  // Initialize IMU
  if (!IMU.begin()) {
    while (1);
  }

  current_time = millis();
}

void loop() {
  previous_time = current_time;
  current_time = millis();
  delta_T = (current_time - previous_time) / 1000.0;

  // Read accelerometer and gyroscope, update complementary filter
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x_accel, y_accel, z_accel);
    theta_accel = (-1) * atan2(y_accel, z_accel) * 180 / PI;
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x_gyro, y_gyro, z_gyro);
    if (abs(x_gyro) < 0.7) x_gyro = 0;
    theta_gyro += x_gyro * delta_T;
  }

  theta_comp = k * (theta_comp + x_gyro * delta_T) + (1 - k) * theta_accel;
  err = theta_comp - desired_angle;

  // PID control calculation
  pid_p = k_p * err;
  pid_i += k_i * err * delta_T;
  pid_i = constrain(pid_i, -50, 50);
  pid_d = k_d * ((err - previous_err) / delta_T);
  response = pid_p + pid_i + pid_d;
  response = constrain(response, -255, 255);
  pwm = abs(response);
  previous_err = err;

  // Motor PWM output based on PID response
  if (response > 0) {
    analogWrite(A_IN1, 255);
    analogWrite(A_IN2, 255 - pwm);
    analogWrite(B_IN1, 255);
    analogWrite(B_IN2, 255 - pwm * right_wheel_adjustment);
  } else if (response < -0) {
    analogWrite(A_IN1, 255 - pwm);
    analogWrite(A_IN2, 255);
    analogWrite(B_IN1, 255 - pwm * right_wheel_adjustment);
    analogWrite(B_IN2, 255);
  } else {
    analogWrite(A_IN1, 0);
    analogWrite(A_IN2, 0);
    analogWrite(B_IN1, 0);
    analogWrite(B_IN2, 0);
  }

  // Debug output
  Serial.print(response);
  Serial.print('\t');
  Serial.print(pwm);
  Serial.print('\t');
  Serial.print(theta_comp);
  Serial.print('\n');

  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    // BLE command parsing loop
    while (central.connected()) {
      if (customCharacteristic.written()) {
        int length = customCharacteristic.valueLength();
        const unsigned char* receivedData = customCharacteristic.value();
        char receivedString[length + 1];
        memcpy(receivedString, receivedData, length);
        receivedString[length] = '\0';
        Serial.print("Received data: ");
        Serial.println(receivedString);
        customCharacteristic.writeValue("Data received");

        // Handle movement and PID tuning commands
        String cmd = String(receivedString);

        if (cmd == "FORWARD") {
          forward_flag = 1; backward_flag = left_flag = right_flag = 0; forward_runtime = 0;
        } else if (cmd == "BACKWARD") {
          backward_flag = 1; forward_flag = left_flag = right_flag = 0; backward_runtime = 0;
        } else if (cmd == "LEFT") {
          left_flag = 1; forward_flag = backward_flag = right_flag = 0;
        } else if (cmd == "RIGHT") {
          right_flag = 1; forward_flag = backward_flag = left_flag = 0;
        } else if (cmd == "PAUSE") {
          forward_flag = backward_flag = left_flag = right_flag = 0;
          // Smoothly adjust to tuned angle
        } else if (cmd == "UP") {
          desired_angle += 0.01;
        } else if (cmd == "DOWN") {
          desired_angle -= 0.01;
        } else if (cmd == "P UP") {
          k_p += 0.1;
        } else if (cmd == "P DOWN") {
          k_p -= 0.1;
        } else if (cmd == "I UP") {
          k_i += 0.5;
        } else if (cmd == "I DOWN") {
          k_i -= 0.5;
        } else if (cmd == "D UP") {
          k_d += 0.01;
        } else if (cmd == "D DOWN") {
          k_d -= 0.01;
        }

        // Feedback PID parameters over BLE
        char pidBuffer[64];
        snprintf(pidBuffer, sizeof(pidBuffer), "P%.2f I%.2f D%.3f A%.3f", k_p, k_i, k_d, desired_angle);
        customCharacteristic.writeValue((const unsigned char*)pidBuffer, strlen(pidBuffer));
        delay(500);
      }

      // Re-run sensor reading and PID updates while connected
      // (Repeated similar logic to maintain balancing)
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x_accel, y_accel, z_accel);
        theta_accel = (-1) * atan2(y_accel, z_accel) * 180 / PI;
      }
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x_gyro, y_gyro, z_gyro);
        if (abs(x_gyro) < 0.7) x_gyro = 0;
        theta_gyro += x_gyro * delta_T;
      }

      theta_comp = k * (theta_comp + x_gyro * delta_T) + (1 - k) * theta_accel;
      err = theta_comp - desired_angle;
      pid_p = k_p * err;
      pid_i += k_i * err * delta_T;
      pid_i = constrain(pid_i, -55, 55);
      pid_d = k_d * ((err - previous_err) / delta_T);
      response = pid_p + pid_i + pid_d;
      response = constrain(response, -255, 255);
      pwm = abs(response);
      previous_err = err;

      snprintf(pidBuffer, sizeof(pidBuffer), "P:%.2f I:%.2f D:%.2f A:%.2f", k_p, k_i, k_d, desired_angle);
      customCharacteristic.writeValue((const unsigned char*)pidBuffer, strlen(pidBuffer));
      delay(500);  // Throttle BLE updates
      
      // Handle movement flags (forward, backward, turning)
      if (!left_flag && !right_flag) {
        // Balance
      }
      if (forward_flag != 0) go_forward();
      if (backward_flag != 0) go_backward();
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Disconnected from central.");
  }
}

// Periodic toggling of angles for forward/backward movement
void go_forward() {
  if (forward_previous_time == 0) {
    forward_previous_time = millis();
    return;
  }
  forward_current_time = millis();
  forward_runtime += (forward_current_time - forward_previous_time);
  forward_previous_time = forward_current_time;

  if (forward_flag == 1 && forward_runtime >= 100) {
    forward_flag = 2;
    forward_runtime = 0;
  } else if (forward_flag == 2 && forward_runtime >= 50) {
    forward_flag = 1;
    forward_runtime = 0;
  }

  desired_angle = (forward_flag == 1) ? forward_desired_angle : tuned_desired_angle;
}

// Similar logic for backward
void go_backward() {
  if (backward_previous_time == 0) {
    backward_previous_time = millis();
    return;
  }
  backward_current_time = millis();
  backward_runtime += (backward_current_time - backward_previous_time);
  backward_previous_time = backward_current_time;

  if (backward_flag == 1 && backward_runtime >= 100) {
    backward_flag = 2;
    backward_runtime = 0;
  } else if (backward_flag == 2 && backward_runtime >= 50) {
    backward_flag = 1;
    backward_runtime = 0;
  }

  desired_angle = (backward_flag == 1) ? backward_desired_angle : tuned_desired_angle;
}

