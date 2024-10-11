#ifndef WHEELVELOCITIES_H
#define WHEELVELOCITIES_H

class WheelVelocities {
public:
    // Public member variables for wheel velocities
    volatile long topLeft;
    volatile long topRight;
    volatile long bottomLeft;
    volatile long bottomRight;

    // Default constructor
    WheelVelocities()
        : topLeft(0), topRight(0), bottomLeft(0), bottomRight(0) {}

    // Parameterized constructor
    WheelVelocities(volatile long tl, volatile long tr, volatile long bl, volatile long br)
        : topLeft(tl), topRight(tr), bottomLeft(bl), bottomRight(br) {}

    // Copy constructor
    WheelVelocities(const WheelVelocities& other)
        : topLeft(other.topLeft), topRight(other.topRight), bottomLeft(other.bottomLeft), bottomRight(other.bottomRight) {}

    // Assignment operator
    WheelVelocities& operator=(const WheelVelocities& other) {
        if (this != &other) {
            topLeft = other.topLeft;
            topRight = other.topRight;
            bottomLeft = other.bottomLeft;
            bottomRight = other.bottomRight;
        }
        return *this;
    }

    // Destructor
    ~WheelVelocities() {}
};

#endif // WHEELVELOCITIES_H