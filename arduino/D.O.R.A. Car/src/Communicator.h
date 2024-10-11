#ifndef COMUNICATOR_H
#define COMUNICATOR_H

#include <Arduino.h>
#include "WheelVelocities.h"

/**
 * @class Communicator
 * @brief Handles communication for sending and receiving data.
 *
 * This class provides functionalities to initialize communication, send data,
 * and receive data, specifically for wheel velocities.
 */
class Communicator
{
public:
    /**
     * @brief Constructs a Communicator object.
     */
    Communicator();

    /**
     * @brief Initializes the communication with the specified baud rate.
     *
     * @param baudRate The baud rate for communication.
     */
    void init(int baudRate);

    /**
     * @brief Sends a string of data.
     *
     * @param data The data to send.
     */
    void SendData(const String &data);

    /**
     * @brief Sends the encoder data for all four wheels.
     *
     * @param data The wheel velocities to send.
     */
    void SendEncoderData(WheelVelocities data);

    /**
     * @brief Receives the wheel velocities from incoming data.
     *
     * @return The received wheel velocities.
     */
    WheelVelocities ReceiveValues();

private:
    const char startSendingCharacter = '{';
    const char endSendingCharacter = '}';

    const char startReceivingCharacter = '[';
    const char endReceivingCharacter = ']';

    /**
     * @brief Parses a message to extract wheel velocities.
     *
     * @param data The data string to parse.
     * @return The parsed wheel velocities.
     */
    WheelVelocities parseMessage(const char *data);

    /**
     * @brief Calculates a simple checksum for a given data string.
     *
     * @param data The data string to calculate the checksum for.
     * @return The calculated checksum.
     */
    int calculateChecksum(const String &data);

    void clearSerialBuffers();

    WheelVelocities wheelVelocities;
};

#endif // COMUNICATOR_H