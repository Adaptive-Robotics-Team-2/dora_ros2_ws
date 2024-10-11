#include "Car.h"

Car::Car()
    : topLeftWheel(nullptr), topRightWheel(nullptr), bottomLeftWheel(nullptr), bottomRightWheel(nullptr)
{
}

void Car::init()
{
    topLeftWheel = new Wheel(topLeftWheelSpeedControlPin, topLeftWheelMoterPin1, topLeftWheelMoterPin2,
                             topLeftWheelInterruptEncoderPinA, topLeftWheelConfermationEncoderPinB,
                             topLeftWheelKp, topLeftWheelKi, topLeftWheelKd);
    topRightWheel = new Wheel(topRightWheelSpeedControlPin, topRightWheelMoterPin1, topRightWheelMoterPin2,
                              topRightWheelInterruptEncoderPinA, topRightWheelConfermationEncoderPinB,
                              topRightWheelKp, topRightWheelKi, topRightWheelKd);
    bottomLeftWheel = new Wheel(bottomLeftWheelSpeedControlPin, bottomLeftWheelMoterPin1, bottomLeftWheelMoterPin2,
                                bottomLeftWheelInterruptEncoderPinA, bottomLeftWheelConfermationEncoderPinB,
                                bottomLeftWheelKp, bottomLeftWheelKi, bottomLeftWheelKd);
    bottomRightWheel = new Wheel(bottomRightWheelSpeedControlPin, bottomRightWheelMoterPin1, bottomRightWheelMoterPin2,
                                 bottomRightWheelInterruptEncoderPinA, bottomRightWheelConfermationEncoderPinB,
                                 bottomRightWheelKp, bottomRightWheelKi, bottomRightWheelKd);

    Serial.println("Car initialized");
}

void Car::Drive()
{
    // Serial.println("Driving...");
    calculateCurrentSpeed();
    PIDscompute();
    topLeftWheel->SetServo();
    topRightWheel->SetServo();
    bottomLeftWheel->SetServo();
    bottomRightWheel->SetServo();
    // Serial.println("Driving done.");
}

void Car::calculateCurrentSpeed()
{
    topLeftWheel->CalculateCurrentSpeed();
    topRightWheel->CalculateCurrentSpeed();
    bottomLeftWheel->CalculateCurrentSpeed();
    bottomRightWheel->CalculateCurrentSpeed();
}

void Car::PIDscompute()
{
    topLeftWheel->GetPID()->Compute();
    topRightWheel->GetPID()->Compute();
    bottomLeftWheel->GetPID()->Compute();
    bottomRightWheel->GetPID()->Compute();
}

void Car::SetSetpoints(const WheelVelocities &wheelVelocities)
{
    topLeftWheel->SetSetpoint(wheelVelocities.topLeft);
    topRightWheel->SetSetpoint(wheelVelocities.topRight);
    bottomLeftWheel->SetSetpoint(wheelVelocities.bottomLeft);
    bottomRightWheel->SetSetpoint(wheelVelocities.bottomRight);
}

WheelVelocities Car::GetEncoders()
{
    WheelVelocities encoders;
    encoders.topLeft = topLeftWheel->GetEncoderPosition();
    encoders.topRight = topRightWheel->GetEncoderPosition();
    encoders.bottomLeft = bottomLeftWheel->GetEncoderPosition();
    encoders.bottomRight = bottomRightWheel->GetEncoderPosition();

    return encoders;
}

Wheel *Car::GetTopLeftWheel()
{
    return topLeftWheel;
}

Wheel *Car::GetTopRightWheel()
{
    return topRightWheel;
}

Wheel *Car::GetBottomLeftWheel()
{
    return bottomLeftWheel;
}

Wheel *Car::GetBottomRightWheel()
{
    return bottomRightWheel;
}

#pragma region CarMethodsToImplement
// TODO: Method to drive the car forward
void Car::ForwardDrive()
{
}

// TODO: Implement the ReverseDrive method
void Car::ReverseDrive()
{
}

// TODO: Implement the TurnLeft method
void Car::TurnLeft()
{
}

// TODO: Implement the TurnRight method
void Car::TurnRight()
{
}

// TODO: Implement the Stop method
void Car::Stop()
{
}
#pragma endregion

Car::~Car()
{
}