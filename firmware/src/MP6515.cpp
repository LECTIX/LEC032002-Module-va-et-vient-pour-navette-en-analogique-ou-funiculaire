#include "MP6515.h"
#include "wiring_private.h"

MP6515 motor;

uint16_t MP6515::_accelStep_ms = DEFAULT_ACCELERATION;
uint16_t MP6515::_decelStep_ms = DEFAULT_DECELERATION;
uint8_t MP6515::_currentSpeed = 0;
uint8_t MP6515::_maxSpeed = DEFAULT_MAX_SPEED;
uint8_t MP6515::_nextDirection = LOW;
uint8_t MP6515::_currentDirection = LOW;
uint32_t MP6515::_lastEventTime = millis();
MP6515::STATE MP6515::_state = MP6515::STATE::STOP;

void MP6515::begin(uint8_t dir)
{
    _nextDirection = dir;
    changeDirection();
    _state = MP6515::STATE::STOP;
    updateMotorSpeed(0);
}

void MP6515::updateMotorSpeed(uint8_t speed)
{
    if (speed == 0)
    {
        digitalWrite(PIN_MOTOR_EN, LOW);
    }
    else if (speed == 255)
    {
        digitalWrite(PIN_MOTOR_EN, HIGH);
    }
    else
    {
        // Setting up PWM on PIN_MOTOR_EN

        // Clear OC1B on Compare Match, set
        // OC1B at BOTTOM (non-inverting mode)
        sbi(TCCR1A, COM1B1);
        cbi(TCCR1A, COM1B0);
        // Select Fast PWM, 8-bit mode
        sbi(TCCR1A, WGM10);
        cbi(TCCR1A, WGM11);
        sbi(TCCR1B, WGM12);
        cbi(TCCR1B, WGM13);
        // Clock prescaler /1 selection
        sbi(TCCR1B, CS10);
        cbi(TCCR1B, CS11);
        cbi(TCCR1B, CS12);
        //cbi(TCCR0A, COM0A0);
        OCR1B = speed; // set pwm duty
    }
}

void MP6515::brake()
{
    _state = MP6515::STATE::STOP;
    _nextDirection = _currentDirection; // make sure we were are not going to go the other way once we stopped
}

void MP6515::setMaxSpeed(uint8_t maxSpeed)
{
    _maxSpeed = maxSpeed;
    if (_maxSpeed == 0)
        _maxSpeed = MIN_MAXIMUM_SPEED;
}

void MP6515::changeDirection()
{
    _currentDirection = _nextDirection;
    digitalWrite(PIN_MOTOR_DIR, _currentDirection);
}

void MP6515::move(uint8_t direction)
{
    _nextDirection = direction;
    _state = MP6515::STATE::RUNNING;
}

void MP6515::loop()
{
    uint32_t currentTime = millis();
    switch (_state)
    {
    case MP6515::STATE::STOP:
        if (_currentSpeed > 0)
        {
            if (currentTime - _lastEventTime > _decelStep_ms)
            {
                _lastEventTime = currentTime;
                _currentSpeed--;
                updateMotorSpeed(_currentSpeed);
            }
        }
        else if (_currentDirection != _nextDirection) // if we stopped because we wanted to change direction
        {
            changeDirection();               // change direction
            _state = MP6515::STATE::RUNNING; // set state to running
        }

        break;
    case MP6515::STATE::RUNNING:
        if (_currentDirection != _nextDirection) // if we need to change direction
        {
            _state = MP6515::STATE::STOP; // got to stop state
            break;
        }
        if (_currentSpeed < _maxSpeed)
        {
            if (currentTime - _lastEventTime > _accelStep_ms)
            {
                _lastEventTime = currentTime;
                _currentSpeed++;
                updateMotorSpeed(_currentSpeed);
            }
        }
        else if (_currentSpeed > _maxSpeed)
        {
            if (currentTime - _lastEventTime > _decelStep_ms)
            {
                _lastEventTime = currentTime;
                _currentSpeed--;
                updateMotorSpeed(_currentSpeed);
            }
        }

        break;

    default:
        break;
    }
}