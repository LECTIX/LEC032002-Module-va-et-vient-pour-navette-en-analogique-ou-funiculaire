#include <Arduino.h>
#include "constants.h"
#include "MP6515.h"
#include <EEPROM.h>

uint32_t lastTriggerTmstp = 0;
uint32_t pauseStartTime = 0;
uint32_t pauseDuration = 0;
uint8_t direction = FORWARD_DIR;
uint8_t stateAddr = 1;

enum STATE
{
    INIT,
    WAITING_FOR_STOP_TRIGGER,
    BREAKING,
    PAUSE
};

static STATE state = INIT;

void updateMotorSettings()
{
    motor.setAcceleration(analogRead(PIN_ACCEL) << 4);
    motor.setDeceleration(analogRead(PIN_DECEL) << 4);
    motor.setMaxSpeed(255 - ((analogRead(PIN_MAX_SPEED) >> 2) & 0x00FF));
    pauseDuration = 200ULL * (1024 - analogRead(PIN_DELAY));
}

void registerDir(uint8_t dir)
{
    EEPROM.update(stateAddr, dir); // save next direction in case of shutdown
}

void setup()
{
    pinMode(PIN_MOTOR_EN, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_MOTOR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_FAULT, INPUT_PULLUP);
    pinMode(PIN_ILS1, INPUT);
    pinMode(PIN_ILS2, INPUT);

    updateMotorSettings();

    stateAddr = EEPROM.read(0);
    direction = EEPROM.read(stateAddr);
    stateAddr++;
    if (stateAddr == 0)
        stateAddr = 1;
    else if (stateAddr > 127)
        stateAddr = 1;
    EEPROM.write(0, stateAddr);
    registerDir(direction);

    motor.begin(direction);
}

void loop()
{
    digitalWrite(PIN_LED, !digitalRead(PIN_MOTOR_FAULT));
    motor.loop();
    if (millis() - lastTriggerTmstp > 100)
    {
        lastTriggerTmstp = millis();
        updateMotorSettings();
    }

    switch (state)
    {
    case INIT:
    {
        motor.move(direction);
        state = WAITING_FOR_STOP_TRIGGER;
        break;
    }

    case WAITING_FOR_STOP_TRIGGER:
    {
        uint8_t sensorPin = PIN_ILS2;
        if (direction == FORWARD_DIR)
            sensorPin = PIN_ILS1;

        if (digitalRead(sensorPin) == 0)
        {
            bool falseTrigger = false;
            lastTriggerTmstp = millis();
            while (millis() - lastTriggerTmstp < 10)
                falseTrigger |= digitalRead(sensorPin) == 1;
            if (falseTrigger == false)
            {
                registerDir(!direction); // save next direction in case of shutdown
                motor.brake();
                state = BREAKING;
            }
        }
        break;
    }

    case BREAKING:
    {
        if (motor.getSpeed() == 0)
        {
            pauseStartTime = millis();
            state = PAUSE;
            direction = !direction;
        }
        break;
    }

    case PAUSE:
    {
        if (millis() - pauseStartTime > pauseDuration)
        {
            state = INIT;
        }
        break;
    }

    default:
        break;
    }
}