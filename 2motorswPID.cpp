#include <Arduino.h>
#include <PID_v1.h>

// Encoder Position Variables
volatile long encoderPosition1 = 0;  // Position for encoder 1
volatile long encoderPosition2 = 0;  // Position for encoder 2
volatile long encoderPosition3 = 0;  // Position for encoder 3
volatile long encoderPosition4 = 0;  // Position for encoder 4

// Motor Control Pins
const int motorPins[4][3] = {
  {2, 3, 4},   // Motor 1: Enable, IN1, IN2
  {5, 6, 7},   // Motor 2: Enable, IN1, IN2
  {8, 9, 10},  // Motor 3: Enable, IN1, IN2
  {11, 12, 13} // Motor 4: Enable, IN1, IN2
};

// Encoder Pins
const int encoderPins[4] = {14, 15, 16, 17}; // Encoder pins (Channel A)

// PID Variables
double setpoint[4], input[4], output[4];
double kp = 1.0, ki = 0.5, kd = 0.1;  // Tune these values
PID pids[4] = {
  PID(&input[0], &output[0], &setpoint[0], kp, ki, kd, DIRECT),
  PID(&input[1], &output[1], &setpoint[1], kp, ki, kd, DIRECT),
  PID(&input[2], &output[2], &setpoint[2], kp, ki, kd, DIRECT),
  PID(&input[3], &output[3], &setpoint[3], kp, ki, kd, DIRECT)
};

// Velocity variables
float vx = 0, vy = 0, omega = 0;  // Linear velocities and angular velocity

void setup() {
  Serial.begin(115200); // UART Communication

  // Set pin modes for motors and encoders
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i][0], OUTPUT); // Enable pin
    pinMode(motorPins[i][1], OUTPUT); // IN1
    pinMode(motorPins[i][2], OUTPUT); // IN2
    pinMode(encoderPins[i], INPUT);    // Encoder pins
    pids[i].SetMode(AUTOMATIC);
    pids[i].SetOutputLimits(0, 255); // PWM limits
  }

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(encoderPins[0]), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPins[1]), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPins[2]), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPins[3]), encoder4ISR, CHANGE);
}

void loop() {
  // Check if data is available from Raspberry Pi
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    parseWheelVelocities(data);
    
    // Read encoders and update PID control
    for (int i = 0; i < 4; i++) {
      input[i] = calculateCurrentSpeed(i);
      pids[i].Compute(); // Update PID control
      setMotorSpeed(i, output[i]);
    }
  }

  // Optional: Print the current positions of all encoders
  Serial.print("Encoder 1 Position: ");
  Serial.println(encoderPosition1);
  Serial.print("Encoder 2 Position: ");
  Serial.println(encoderPosition2);
  Serial.print("Encoder 3 Position: ");
  Serial.println(encoderPosition3);
  Serial.print("Encoder 4 Position: ");
  Serial.println(encoderPosition4);
  delay(200); // Slow down the output for readability
}

// Parse the wheel velocities from Raspberry Pi
void parseWheelVelocities(String data) {
  // Expected format: "wheel1_velocity,wheel2_velocity,wheel3_velocity,wheel4_velocity"
  sscanf(data.c_str(), "%lf,%lf,%lf,%lf", &setpoint[0], &setpoint[1], &setpoint[2], &setpoint[3]);
}

// Calculate current speed of the wheel using encoder feedback
float calculateCurrentSpeed(int wheelIndex) {
  // Use a simple conversion; adjust based on your encoder specs (PUT FORMULA HERE)
  return encoderPosition1; // Placeholder; implement your actual speed calculation
}

// Set motor speed using PWM on the Enable pin and control direction with IN1 and IN2
void setMotorSpeed(int motorIndex, double pwmValue) {
  if (pwmValue > 0) {
    digitalWrite(motorPins[motorIndex][1], HIGH); // IN1
    digitalWrite(motorPins[motorIndex][2], LOW);  // IN2
    analogWrite(motorPins[motorIndex][0], pwmValue); // Enable pin controls speed
  } else if (pwmValue < 0) {
    digitalWrite(motorPins[motorIndex][1], LOW);  // IN1
    digitalWrite(motorPins[motorIndex][2], HIGH); // IN2
    analogWrite(motorPins[motorIndex][0], -pwmValue); // Enable pin controls speed
  } else {
    digitalWrite(motorPins[motorIndex][0], LOW);  // Disable motor (no PWM signal)
    digitalWrite(motorPins[motorIndex][1], LOW);  // Stop motor
    digitalWrite(motorPins[motorIndex][2], LOW);  // Stop motor
  }
}

// Interrupt service routine for encoder 1
void encoder1ISR() {
  encoderPosition1 += (digitalRead(encoderPins[0]) == HIGH) ? 1 : -1;
}

// Interrupt service routine for encoder 2
void encoder2ISR() {
  encoderPosition2 += (digitalRead(encoderPins[1]) == HIGH) ? 1 : -1;
}

// Interrupt service routine for encoder 3
void encoder3ISR() {
  encoderPosition3 += (digitalRead(encoderPins[2]) == HIGH) ? 1 : -1;
}

// Interrupt service routine for encoder 4
void encoder4ISR() {
  encoderPosition4 += (digitalRead(encoderPins[3]) == HIGH) ? 1 : -1;
}
