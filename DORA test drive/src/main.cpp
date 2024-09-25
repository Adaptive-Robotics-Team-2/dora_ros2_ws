#include <Arduino.h>
#include <PID_v1.h>

#define CPR 1440 // Encoder counts per revolution;

// Encoder Position Variables
volatile long encoderPosition1 = 0;
volatile long encoderPosition2 = 0;
volatile long encoderPosition3 = 0;
volatile long encoderPosition4 = 0;

long lastEncoderPosition[4] = {0, 0, 0, 0};
unsigned long lastTime[4] = {0, 0, 0, 0};

// Motor Control Pins
const int motorPins[4][3] = {
    {6, 26, 27}, // Motor 1: Enable, IN1, IN2 (Top Left)
    {7, 28, 29}, // Motor 2: Enable, IN1, IN2 (Top Right)
    {4, 22, 23}, // Motor 3: Enable, IN1, IN2 (Bottom Left)
    {5, 24, 25}, // Motor 4: Enable, IN1, IN2 (Bottom Right)
};

// Encoder Pins (Channels A and B for each encoder)
const int encoderPinsA[4] = {3, 2, 18, 19};   // Encoder Channel A pins
const int encoderPinsB[4] = {53, 52, 51, 50}; // Encoder Channel B pins

// PID Variables
double setpoint[4], input[4], output[4];
double kp = 1.0, ki = 0.5, kd = 0.1; // Tune these values
PID pids[4] = {
    PID(&input[0], &output[0], &setpoint[0], kp, ki, kd, DIRECT),
    PID(&input[1], &output[1], &setpoint[1], kp, ki, kd, DIRECT),
    PID(&input[2], &output[2], &setpoint[2], kp, ki, kd, DIRECT),
    PID(&input[3], &output[3], &setpoint[3], kp, ki, kd, DIRECT)};

// Function to handle changes on both channels of the encoder
void handleEncoderChange(int encoderIndex, volatile long &encoderPosition)
{
  // Read both channels
  int a = digitalRead(encoderPinsA[encoderIndex]);
  int b = digitalRead(encoderPinsB[encoderIndex]);

  // Update encoder position based on direction
  if (a == b)
  {
    encoderPosition++; // Forward
  }
  else
  {
    encoderPosition--; // Reverse
  }
}

// Interrupt service routine for encoder 1
void encoder1ISR()
{
  handleEncoderChange(0, encoderPosition1);
}

// Interrupt service routine for encoder 2
void encoder2ISR()
{
  handleEncoderChange(1, encoderPosition2);
}

// Interrupt service routine for encoder 3
void encoder3ISR()
{
  handleEncoderChange(2, encoderPosition3);
}

// Interrupt service routine for encoder 4
void encoder4ISR()
{
  handleEncoderChange(3, encoderPosition4);
}

// Set motor speed using PWM on the Enable pin and control direction with IN1 and IN2
void setMotorSpeed(int motorIndex, double pwmValue)
{
  // Ensure the PWM value is within bounds
  int pwm = constrain(pwmValue, -255, 255);

  if (pwm > 0)
  {
    // Forward direction
    digitalWrite(motorPins[motorIndex][1], LOW);  // IN1
    digitalWrite(motorPins[motorIndex][2], HIGH); // IN2
    analogWrite(motorPins[motorIndex][0], pwm);   // Enable pin controls speed
  }
  else if (pwm < 0)
  {
    // Reverse direction
    digitalWrite(motorPins[motorIndex][1], HIGH); // IN1
    digitalWrite(motorPins[motorIndex][2], LOW);  // IN2
    analogWrite(motorPins[motorIndex][0], -pwm);  // Enable pin controls speed (use absolute value)
  }
  else
  {
    // Stop the motor
    digitalWrite(motorPins[motorIndex][0], LOW); // Disable motor (no PWM signal)
    digitalWrite(motorPins[motorIndex][1], LOW); // Stop motor
    digitalWrite(motorPins[motorIndex][2], LOW); // Stop motor
  }
}

void parseWheelVelocities(const char * data)
{
  // Expected format: "wheel1_velocity,wheel2_velocity,wheel3_velocity,wheel4_velocity"
  // sscanf(data.c_str(), "%lf,%lf,%lf,%lf", &setpoint[0], &setpoint[1], &setpoint[2], &setpoint[3]);
  // use strtok() to split the string into tokens based on the delimiter ','
  setpoint[0] = atof(strtok(data, ","));
  setpoint[1] = atof(strtok(NULL, ","));
  setpoint[2] = atof(strtok(NULL, ","));
  setpoint[3] = atof(strtok(NULL, ","));

  Serial.println(setpoint[0]);
  Serial.println(setpoint[1]);
  Serial.println(setpoint[2]);
  Serial.println(setpoint[3]);
}

// Calculate current speed of the wheel using encoder feedback in rad/s
float calculateCurrentSpeed(int wheelIndex)
{
  // Get the current time and encoder position
  unsigned long currentTime = millis(); // Time in milliseconds
  long currentEncoderPosition = 0;

  // Select the correct encoder position based on the wheel index
  switch (wheelIndex)
  {
  case 0:
    currentEncoderPosition = encoderPosition1;
    break;
  case 1:
    currentEncoderPosition = encoderPosition2;
    break;
  case 2:
    currentEncoderPosition = encoderPosition3;
    break;
  case 3:
    currentEncoderPosition = encoderPosition4;
    break;
  default:
    return 0;
  }

  // Calculate the time difference (delta t) in seconds
  float deltaTime = (currentTime - lastTime[wheelIndex]) / 1000.0; // Convert to seconds

  // Prevent division by zero
  if (deltaTime <= 0)
  {
    deltaTime = 0.001; // If no time has passed, assume a small value
  }

  // Calculate the change in encoder counts (delta counts)
  long deltaCounts = currentEncoderPosition - lastEncoderPosition[wheelIndex];

  // Store the current encoder position and time for the next calculation
  lastEncoderPosition[wheelIndex] = currentEncoderPosition;
  lastTime[wheelIndex] = currentTime;

  // Calculate the speed in revolutions per second (RPS)
  float speedRPS = deltaCounts / (float)CPR / deltaTime;

  // Convert speed from RPS to rad/s (1 revolution = 2 * pi radians)
  float speedRadPerSec = speedRPS * 2 * PI;

  return speedRadPerSec; // Return the speed in rad/s
}

// Function to send raw encoder positions back to Raspberry Pi
void sendEncoderData()
{
  Serial.print(encoderPosition1);
  Serial.print(",");
  Serial.print(encoderPosition2);
  Serial.print(",");
  Serial.print(encoderPosition3);
  Serial.print(",");
  Serial.println(encoderPosition4);
}

void setup()
{
  Serial.begin(9600); // UART Communication

  for (int i = 0; i < 4; i++)
  {
    pinMode(motorPins[i][0], OUTPUT); // Enable pin
    pinMode(motorPins[i][1], OUTPUT); // IN1
    pinMode(motorPins[i][2], OUTPUT); // IN2
    pinMode(encoderPinsA[i], INPUT);  // Encoder Channel A
    pinMode(encoderPinsB[i], INPUT);  // Encoder Channel B
    pids[i].SetMode(AUTOMATIC);
    pids[i].SetOutputLimits(-255, 255); // PWM limits
  }

  // Attach interrupts for encoder channels
  attachInterrupt(digitalPinToInterrupt(encoderPinsA[0]), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinsA[1]), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinsA[2]), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinsA[3]), encoder4ISR, CHANGE);
}

void loop()
{
  // Check if data is available from Raspberry Pi
  if (Serial.available() > 0)
  {
    String readLine = Serial.readStringUntil('\n');
    const char* data = readLine.c_str();
    Serial.println(data); // Echo back the received data for debugging
    parseWheelVelocities(data);

    // Read encoders and update PID control
    for (int i = 0; i < 4; i++)
    {
      input[i] = calculateCurrentSpeed(i); // Calculate speed based on encoder position
      pids[i].Compute();                   // Update PID control
      setMotorSpeed(i, output[i]);         // Apply PID output to motor
    }

    // Send raw encoder data back to Raspberry Pi
    // sendEncoderData();
  }

  delay(200); // Slow down the output for readability
}
