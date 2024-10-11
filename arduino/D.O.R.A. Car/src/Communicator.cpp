#include "Communicator.h"

Communicator::Communicator()
{
    // Default constructor
}

void Communicator::init(int baudRate)
{
    Serial.begin(baudRate);
    Serial2.begin(115200);
    while (!Serial)
    {
        ; // Wait for serial port to connect. Needed for native USB port only
    }
    while (!Serial2)
    {
        ; // Wait for serial port to connect. Needed for native USB port only
    }

    Serial.flush();  // Flush the serial buffer
    Serial2.flush(); // Flush the serial buffer
    Serial.println("Communicator initialized.");
}

void Communicator::SendData(const String &data)
{
    static String previousData = "";

    if (data != previousData)
    {
        // Serial.println("Sending Data: " + data);
        previousData = data;
    }

    Serial2.println(data);
}
void Communicator::SendEncoderData(WheelVelocities data)
{
    String payload = String(data.topLeft) + "," + String(data.topRight) +
                     "," + String(data.bottomLeft) + "," + String(data.bottomRight);
    String message = startSendingCharacter + payload + endSendingCharacter;
    SendData(message);
}

WheelVelocities Communicator::ReceiveValues()
{
    // Clear both Serial and Serial2 buffers before starting to read new data

    if (Serial2.available() > 0)
    {
        // Read until endReceivingCharacter is encountered (end character will be removed)
        String readLine = Serial2.readStringUntil(endReceivingCharacter);

        // Echo back the received data for debugging
        Serial.println("Received Data: " + readLine);
        // Serial2.flush(); // Flush the serial2 buffer to ensure it's cleared after reading
        clearSerialBuffers();

        // Parse the received message (pass the string directly, no need to look for the end character)
        return parseMessage(readLine.c_str());
    }

    // Return default velocities if nothing is received
    return {0, 0, 0, 0};
}

void Communicator::clearSerialBuffers()
{
    // Clear any leftover data from the Serial buffer
    while (Serial.available())
    {
        Serial.read(); // Read and discard any leftover data
    }

    // Clear any leftover data from the Serial2 buffer
    while (Serial2.available())
    {
        Serial2.read(); // Read and discard any leftover data
    }

    // Optionally, add some delay to ensure buffers are fully cleared
    // delay(10);
}

WheelVelocities Communicator::parseMessage(const char *data)
{
    WheelVelocities velocities = {0, 0, 0, 0};

    // Find the start of the message
    const char *start = strchr(data, startReceivingCharacter);

    // Check if the start character exists
    if (start == NULL)
    {
        Serial.println("Invalid data received: no start character found"); // Debugging message for invalid data
        return velocities;                                                 // Invalid message
    }

    // Skip the startReceivingCharacter
    start++;

    // Parse the payload (since the end character is already removed)
    char payload[strlen(start) + 1]; // Allocate buffer for the remaining string
    strcpy(payload, start);          // Copy remaining data after start character

    // Debugging message to show the extracted payload
    Serial.println("Payload: " + String(payload));

    // Parse the payload
    char *token = strtok(payload, ",");
    if (token != NULL)
    {
        velocities.topLeft = atof(token);
        if (velocities.topLeft == 0)
        {
            Serial.println("Warning: topLeft value is 0 or null");
        }
    }

    token = strtok(NULL, ",");
    if (token != NULL)
    {
        velocities.topRight = atof(token);
        if (velocities.topRight == 0)
        {
            Serial.println("Warning: topRight value is 0 or null");
        }
    }

    token = strtok(NULL, ",");
    if (token != NULL)
    {
        velocities.bottomLeft = atof(token);
        if (velocities.bottomLeft == 0)
        {
            Serial.println("Warning: bottomLeft value is 0 or null");
        }
    }

    token = strtok(NULL, ",");
    if (token != NULL)
    {
        velocities.bottomRight = atof(token);
        if (velocities.bottomRight == 0)
        {
            Serial.println("Warning: bottomRight value is 0 or null");
        }
    }

    Serial.println("Top Left: " + String(velocities.topLeft) + " Top Right: " + String(velocities.topRight) +
                   "\nBottom Left: " + String(velocities.bottomLeft) + " Bottom Right: " + String(velocities.bottomRight) + "\n");

    return velocities;
}
