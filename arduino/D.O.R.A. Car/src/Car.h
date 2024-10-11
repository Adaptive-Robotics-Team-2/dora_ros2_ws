#ifndef CAR_H
#define CAR_H

#include "Wheel.h"
#include "WheelVelocities.h"

/**
 * @class Car
 * @brief Represents a car with multiple wheels and various driving functionalities.
 *
 * This class provides functionalities to control a car with four wheels, including
 * setting setpoints for each wheel, driving in different directions, and reading encoder values.
 */
class Car
{
public:
    /**
     * @brief Constructs a Car object.
     */
    Car();

    /**
     * @brief Initializes the Car object.
     */
    void init();

    /**
     * @brief Destructs the Car object.
     */
    ~Car();

    /**
     * @brief Drives the car based on the setpoints.
     */
    void Drive();

    /**
     * @brief Sets the setpoints for the wheel velocities.
     *
     * This function updates the setpoints for the wheel velocities of the car.
     *
     * @param wheelVelocities A reference to a WheelVelocities object containing
     *                        the desired velocities for each wheel.
     */
    void SetSetpoints(const WheelVelocities &wheelVelocities);

    /**
     * @brief Gets the top left wheel.
     *
     * @return A pointer to the top left wheel.
     */
    Wheel *GetTopLeftWheel();

    /**
     * @brief Gets the top right wheel.
     *
     * @return A pointer to the top right wheel.
     */
    Wheel *GetTopRightWheel();

    /**
     * @brief Gets the bottom left wheel.
     *
     * @return A pointer to the bottom left wheel.
     */
    Wheel *GetBottomLeftWheel();

    /**
     * @brief Gets the bottom right wheel.
     *
     * @return A pointer to the bottom right wheel.
     */
    Wheel *GetBottomRightWheel();

    /**
     * @brief Gets the encoder values for all four wheels.
     *
     * @return A WheelVelocities struct containing the encoder values.
     */
    WheelVelocities GetEncoders();

    // TODO: Add methods for driving directions

    /**
     * @brief Drives the car forward.
     */
    void ForwardDrive();

    /**
     * @brief Drives the car in reverse.
     */
    void ReverseDrive();

    /**
     * @brief Turns the car left.
     */
    void TurnLeft();

    /**
     * @brief Turns the car right.
     */
    void TurnRight();

    /**
     * @brief Drives the car diagonally.
     */
    void DiagonalDrive();

    /**
     * @brief Stops the car.
     */
    void Stop();

    // TODO
    /**
     * @enum CarState
     * @brief Represents the state of the car.
     */
    enum CarState
    {
        STATIONARY,       ///< The car is stationary.
        DRIVING_FORWARD,  ///< The car is driving forward.
        REVERSE_DRIVING,  ///< The car is driving in reverse.
        SIDEWAYS_DRIVING, ///< The car is driving sideways.
        DIAGONAL_DRIVING  ///< The car is driving diagonally.
    };

private:
    Wheel *topLeftWheel;     ///< Pointer to the top left wheel.
    Wheel *topRightWheel;    ///< Pointer to the top right wheel.
    Wheel *bottomLeftWheel;  ///< Pointer to the bottom left wheel.
    Wheel *bottomRightWheel; ///< Pointer to the bottom right wheel.
    WheelVelocities wheelVelocities;

    static constexpr int topLeftWheelSpeedControlPin = 6;          ///< Speed control pin for the top left wheel.
    static constexpr int topLeftWheelMoterPin1 = 26;               ///< Motor control pin 1 for the top left wheel.
    static constexpr int topLeftWheelMoterPin2 = 27;               ///< Motor control pin 2 for the top left wheel.
    static constexpr int topLeftWheelInterruptEncoderPinA = 19;    ///< Encoder pin A for the top left wheel.
    static constexpr int topLeftWheelConfermationEncoderPinB = 50; ///< Encoder pin B for the top left wheel.
    static constexpr double topLeftWheelKp = 1.0;                  ///< Proportional gain for the top left wheel's PID controller.
    static constexpr double topLeftWheelKi = 0.0;                  ///< Integral gain for the top left wheel's PID controller.
    static constexpr double topLeftWheelKd = 0.0;                  ///< Derivative gain for the top left wheel's PID controller.

    static constexpr int topRightWheelSpeedControlPin = 7;          ///< Speed control pin for the top right wheel.
    static constexpr int topRightWheelMoterPin1 = 28;               ///< Motor control pin 1 for the top right wheel.
    static constexpr int topRightWheelMoterPin2 = 29;               ///< Motor control pin 2 for the top right wheel.
    static constexpr int topRightWheelInterruptEncoderPinA = 18;    ///< Encoder pin A for the top right wheel.
    static constexpr int topRightWheelConfermationEncoderPinB = 51; ///< Encoder pin B for the top right wheel.
    static constexpr double topRightWheelKp = 1.0;                  ///< Proportional gain for the top right wheel's PID controller.
    static constexpr double topRightWheelKi = 0.0;                  ///< Integral gain for the top right wheel's PID controller.
    static constexpr double topRightWheelKd = 0.0;                  ///< Derivative gain for the top right wheel's PID controller.

    static constexpr int bottomLeftWheelSpeedControlPin = 4;          ///< Speed control pin for the bottom left wheel.
    static constexpr int bottomLeftWheelMoterPin1 = 22;               ///< Motor control pin 1 for the bottom left wheel.
    static constexpr int bottomLeftWheelMoterPin2 = 23;               ///< Motor control pin 2 for the bottom left wheel.
    static constexpr int bottomLeftWheelInterruptEncoderPinA = 2;     ///< Encoder pin A for the bottom left wheel.
    static constexpr int bottomLeftWheelConfermationEncoderPinB = 53; ///< Encoder pin B for the bottom left wheel.
    static constexpr double bottomLeftWheelKp = 1.0;                  ///< Proportional gain for the bottom left wheel's PID controller.
    static constexpr double bottomLeftWheelKi = 0.0;                  ///< Integral gain for the bottom left wheel's PID controller.
    static constexpr double bottomLeftWheelKd = 0.0;                  ///< Derivative gain for the bottom left wheel's PID controller.

    static constexpr int bottomRightWheelSpeedControlPin = 5;          ///< Speed control pin for the bottom right wheel.
    static constexpr int bottomRightWheelMoterPin1 = 24;               ///< Motor control pin 1 for the bottom right wheel.
    static constexpr int bottomRightWheelMoterPin2 = 25;               ///< Motor control pin 2 for the bottom right wheel.
    static constexpr int bottomRightWheelInterruptEncoderPinA = 3;     ///< Encoder pin A for the bottom right wheel.
    static constexpr int bottomRightWheelConfermationEncoderPinB = 52; ///< Encoder pin B for the bottom right wheel.
    static constexpr double bottomRightWheelKp = 1.0;                  ///< Proportional gain for the bottom right wheel's PID controller.
    static constexpr double bottomRightWheelKi = 0.0;                  ///< Integral gain for the bottom right wheel's PID controller.
    static constexpr double bottomRightWheelKd = 0.0;                  ///< Derivative gain for the bottom right wheel's PID controller.
    /**
     * @brief Calculates the current speed of the car.
     */
    void calculateCurrentSpeed();

    /**
     * @brief Computes the PID values for all wheels.
     */
    void PIDscompute();
};

#endif // CAR_H