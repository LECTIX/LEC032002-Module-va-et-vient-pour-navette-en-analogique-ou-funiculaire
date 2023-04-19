#ifndef MP6515_H
#define MP6515_H
#include <Arduino.h>
#include "constants.h"

class MP6515
{
private:
    static uint16_t _accelStep_ms;
    static uint16_t _decelStep_ms;
    static uint8_t _maxSpeed;
    static uint8_t _currentSpeed;
    static uint8_t _nextDirection;
    static uint8_t _currentDirection;
    static uint32_t _lastEventTime;

    enum STATE
    {
        STOP,
        RUNNING
    };

    static STATE _state;

    static void changeDirection();
    static void updateMotorSpeed(uint8_t speed);

public:
    static void setAcceleration(uint16_t accel_ms) { _accelStep_ms = accel_ms/_maxSpeed; } 
    static void setDeceleration(uint16_t decel_ms) { _decelStep_ms = decel_ms/_maxSpeed; }
    static uint8_t getSpeed() { return _currentSpeed; }
    static void setMaxSpeed(uint8_t maxSpeed);
    static void begin(uint8_t dir);
    static void brake();
    static void move(uint8_t direction);
    static void loop(); /* function to call regularly to update the motor status */
};

extern MP6515 motor;

#endif