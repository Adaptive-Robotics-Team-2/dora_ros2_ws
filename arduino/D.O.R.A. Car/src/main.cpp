#include "Car.h"
#include "Communicator.h"
#include "WheelVelocities.h"

#include <TaskScheduler.h>
#define BAUDRATE 9600

Communicator *doraComunicator;
Car *doraCar;

// TaskScheduler
Scheduler runner;

// Task declarations
void PIDControlTask();
void SerialTask();

Task t1(100, TASK_FOREVER, &PIDControlTask, &runner, true);
Task t2(100, TASK_FOREVER, &SerialTask, &runner, true);

// ISR functions for each wheel
void topLeftWheelISR()
{
    if (doraCar != nullptr && doraCar->GetTopLeftWheel() != nullptr)
    {
        doraCar->GetTopLeftWheel()->HandleEncoderChange();
    }
}

void topRightWheelISR()
{
    if (doraCar != nullptr && doraCar->GetTopRightWheel() != nullptr)
    {
        doraCar->GetTopRightWheel()->HandleEncoderChange();
    }
}

void bottomLeftWheelISR()
{
    if (doraCar != nullptr && doraCar->GetBottomLeftWheel() != nullptr)
    {
        doraCar->GetBottomLeftWheel()->HandleEncoderChange();
    }
}

void bottomRightWheelISR()
{
    if (doraCar != nullptr && doraCar->GetBottomRightWheel() != nullptr)
    {
        doraCar->GetBottomRightWheel()->HandleEncoderChange();
    }
}

// PID Control Task
void PIDControlTask()
{
    WheelVelocities wheelVelocities;

    wheelVelocities.topLeft = 0;
    wheelVelocities.topRight = 0;
    wheelVelocities.bottomLeft = 0;
    wheelVelocities.bottomRight = 0;

    wheelVelocities = doraComunicator->ReceiveValues();

    // Serial.println("TL: " + String(wheelVelocities.topLeft) + ", TR: " + String(wheelVelocities.topRight) + ", BL: " + String(wheelVelocities.bottomLeft) + ", BR: " + String(wheelVelocities.bottomRight));
    // Serial.println(String(wheelVelocities.topLeft) + "wddfxdsd," + String(wheelVelocities.topRight) + "," + String(wheelVelocities.bottomLeft) + "," + String(wheelVelocities.bottomRight));

    doraCar->SetSetpoints(wheelVelocities);
    doraCar->Drive();
}

// Serial Communication Task
void SerialTask()
{
    WheelVelocities encoderData = doraCar->GetEncoders();

    doraComunicator->SendEncoderData(encoderData);
}

void setup()
{
    // Serial.begin(BAUDRATE);
    doraComunicator = new Communicator();
    doraComunicator->init(BAUDRATE);

    doraCar = new Car();
    doraCar->init();

    attachInterrupt(digitalPinToInterrupt(19), topLeftWheelISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(18), topRightWheelISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(2), bottomLeftWheelISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), bottomRightWheelISR, CHANGE);

    delay(2000);

    // Start the scheduler
    runner.startNow();
}

void loop()
{
    runner.execute();
}

/*
 void handleEvent()
 {
     switch (currentState)
     {
     case Idle:
         if (carComunicator.getEvent() == carComunicator.KeyUpPressed)
         {
             transitionTo(DrivingForward);
         }
         else if (carComunicator.getEvent() == carComunicator.KeyDownPressed)
         {
             transitionTo(DrivingReverse);
         }
         else if (carComunicator.getEvent() == carComunicator.KeyLeftPressed || carComunicator.getEvent() == carComunicator.KeyRightPressed)
         {
             transitionTo(DrivingSideways);
         }
         break;

     case DrivingForward:
         if (carComunicator.getEvent() == carComunicator.KeysReleased)
         {
             transitionTo(Idle);
         }
         break;

     case DrivingReverse:
         if (carComunicator.getEvent() == carComunicator.KeysReleased)
         {
             transitionTo(Idle);
         }
         break;

     case DrivingSideways:
         if (carComunicator.getEvent() == carComunicator.KeysReleased)
         {
             transitionTo(Idle);
         }
         break;
     }
 }

 void transitionTo(Car::CarState newState)
 {
     switch (newState)
     {
     case Idle:
         car.StopCar();
         break;

     case DrivingForward:
         car.ForwardDrive();
         break;

     case DrivingReverse:
         car.ReverseDrive();
         break;

     case DrivingSideways:
         car.SidewaysDrive();
         break;
     }
     currentState = newState;
 }*/