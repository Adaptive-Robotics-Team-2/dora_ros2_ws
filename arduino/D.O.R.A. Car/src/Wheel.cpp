#include "Wheel.h"

#define CPR 1440 // Encoder counts per revolution;

// Initialize the static instance pointer
Wheel *Wheel::instance = nullptr;

Wheel::Wheel(int enablePin, int in1Pin, int in2Pin, int encPinA, int encPinB,
             double kp, double ki, double kd)
    : motorSpeedEnablePin(enablePin), motorIn1Pin(in1Pin), motorIn2Pin(in2Pin),
      interruptEncoderPinA(encPinA), confermationEncoderPinB(encPinB),
      pid(&PIDinput, &PIDoutput, &setpoint, kp, ki, kd, DIRECT)
{
    // Set the instance pointer to this instance
    instance = this;

    Serial.println("Wheel constructor");
    Serial.println("Motor Speed Enable Pin: " + String(motorSpeedEnablePin));
    Serial.println("Motor In1 Pin: " + String(motorIn1Pin));
    Serial.println("Motor In2 Pin: " + String(motorIn2Pin));
    Serial.println("Interrupt Encoder Pin A: " + String(interruptEncoderPinA));
    Serial.println("Confermation Encoder Pin B: " + String(confermationEncoderPinB));

    // Initialize the motor pins as outputs
    pinMode(motorSpeedEnablePin, OUTPUT);
    pinMode(motorIn1Pin, OUTPUT);
    pinMode(motorIn2Pin, OUTPUT);

    // Initialize the encoder pins as inputs
    pinMode(interruptEncoderPinA, INPUT);
    pinMode(confermationEncoderPinB, INPUT);

    // Attach an interrupt to the encoder channel A
    // attachInterrupt(digitalPinToInterrupt(interruptEncoderPinA), isr, CHANGE);

    pid.SetSampleTime(100);
    pid.SetOutputLimits(-255, 255);
    pid.SetMode(AUTOMATIC);

    Serial.println("Wheel constructor done\n");
}

void Wheel::SetSetpoint(double sp)
{
    // if (sp != 0)
    // {
    //     Serial.println(sp);
    // }

    // Serial.println("Setting setpoint..." + String(sp));
    setpoint = sp;
}

PID *Wheel::GetPID()
{
    return &pid;
}

long Wheel::GetEncoderPosition()
{
    return encoderPosition;
}

void Wheel::HandleEncoderChange()
{
    int a = digitalRead(interruptEncoderPinA);
    // Serial.println("Encoder A: " + String(interruptEncoderPinA));
    int b = digitalRead(confermationEncoderPinB);

    // Check if the pins match the specified conditions
    bool reverseDirection = (interruptEncoderPinA == 2 || interruptEncoderPinA == 19) && 
                            (confermationEncoderPinB == 53 || confermationEncoderPinB == 50);

    // Update encoder position based on the state of the channels and the direction condition
    if (reverseDirection)
    {
        encoderPosition += (a == b) ? -1 : 1;
    }
    else
    {
        encoderPosition += (a == b) ? 1 : -1;
    }
}
void Wheel::SetServo()
{
    int pwm = 0;
    pwm = PIDoutput;
    // int pwm = setpoint;

    pwm = constrain(pwm, -255, 255);

    // Serial.println("Setting servo..." + String(pwm));

    // Serial.println("In1 pin:" + String(motorIn1Pin));
    // Serial.println("In2 pin:" + String(motorIn2Pin));
    // Serial.println("Speed pin: " + String(motorSpeedEnablePin));

    if (pwm > 0)
    {
        // Forward direction
        digitalWrite(motorIn1Pin, LOW);
        digitalWrite(motorIn2Pin, HIGH);
        analogWrite(motorSpeedEnablePin, pwm);

        // Serial.println("pwm > 0");
    }
    else if (pwm < 0)
    {
        // Reverse direction
        digitalWrite(motorIn1Pin, HIGH);        // IN1
        digitalWrite(motorIn2Pin, LOW);         // IN2
        analogWrite(motorSpeedEnablePin, -pwm); // Enable pin controls speed (use absolute value)

        // Serial.println("pwm < 0");
    }
    else
    {
        // Stop the motor
        digitalWrite(motorSpeedEnablePin, LOW); // Disable motor (no PWM signal)
        digitalWrite(motorIn1Pin, LOW);         // Stop motor
        digitalWrite(motorIn2Pin, LOW);         // Stop motor

        // Serial.println("stop motor");
    }
}

void Wheel::CalculateCurrentSpeed()
{
    // Serial.println("Calculating current speed...");
    // Get the current time and encoder position
    unsigned long currentTime = millis(); // Time in milliseconds

    // Calculate the time difference (delta t) in seconds
    float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds

    // Prevent division by zero
    if (deltaTime <= 0)
    {
        deltaTime = 0.001; // If no time has passed, assume a small value
    }

    // Calculate the change in encoder counts (delta counts)
    long deltaCounts = encoderPosition - lastEncoderPosition;

    // Store the current encoder position and time for the next calculation
    lastEncoderPosition = encoderPosition;
    lastTime = currentTime;

    // Calculate the speed in revolutions per second (RPS)
    float speedRPS = deltaCounts / (float)CPR / deltaTime;

    // Convert speed from RPS to rad/s (1 revolution = 2 * pi radians)
    float speedRadPerSec = speedRPS * 2 * PI;
    PIDinput = speedRadPerSec;
    // Serial.println("PIDINPUT: " + String(PIDinput));
}

// Static function to handle the interrupt
// void Wheel::isr()
// {
//     Serial.println("ISR called");
//     if (instance != nullptr)
//     {
//         instance->HandleEncoderChange();
//     }
// }