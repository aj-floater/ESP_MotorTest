#ifndef INPUT_H
#define INPUT_H

#include "mbed.h"
#include "wheel.h"
#include "display.h"

// Joystick class to handle joystick input with ISRs
class Joystick {
private:
    InterruptIn up;      // Interrupt for UP button
    InterruptIn down;    // Interrupt for DOWN button
    InterruptIn fire;    // Interrupt for FIRE button

    volatile bool upActive;    // Debounce state for UP
    volatile bool downActive;  // Debounce state for DOWN
    volatile bool leftActive;  // Debounce state for LEFT
    volatile bool rightActive; // Debounce state for RIGHT
    volatile bool fireActive;  // Debounce state for FIRE

    // ISR handlers for rising edges (button pressed)
    void upPressed() {
        upActive = true;
    }
    void downPressed() {
        downActive = true;
    }
    void firePressed() {
        fireActive = true;
    }

    // ISR handlers for falling edges (button released)
    void upReleased() {
        if (upActive) {
            right_wheel.speed(right_wheel.speed() + 1);  // Increase speed
            display.markRefreshNeeded();

            upActive = false;
        }
    }

    void downReleased() {
        if (downActive) {
            right_wheel.speed(right_wheel.speed() - 1);  // Increase speed
            display.markRefreshNeeded();

            downActive = false;
        }
    }

    void fireReleased() {
        if (fireActive) {
            fireActive = false;
        }
    }
public:
    // Constructor to initialize the joystick with pins for each direction
    Joystick(PinName upPin, PinName downPin, PinName leftPin, PinName rightPin, PinName firePin)
        : up(upPin), down(downPin), fire(firePin),
            upActive(false), downActive(false), leftActive(false), rightActive(false), fireActive(false) {
        // Attach ISRs for rising edges (button pressed)
        up.rise(callback(this, &Joystick::upPressed));
        down.rise(callback(this, &Joystick::downPressed));
        fire.rise(callback(this, &Joystick::firePressed));

        // Attach ISRs for falling edges (button released)
        up.fall(callback(this, &Joystick::upReleased));
        down.fall(callback(this, &Joystick::downReleased));
        fire.fall(callback(this, &Joystick::fireReleased));
    }
};

Joystick joystick(A2, A3, A4, A5, D4);

#endif // INPUT_H