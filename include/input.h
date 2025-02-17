#ifndef INPUT_H
#define INPUT_H

#include "mbed.h"
#include "wheel.h"
#include "display.h"

class Input {
private:
    InterruptIn up, down, left, right;  // Interrupt inputs

    // ISR functions
    void upISR() {
        right_wheel.speed(right_wheel.speed() + 1);  // Increase speed
        display.markRefreshNeeded();
    }

    void downISR() {
        right_wheel.speed(right_wheel.speed() - 1);  // Decrease speed
        display.markRefreshNeeded();
    }

public:
    // Constructor initializes interrupt pins and attaches ISR functions
    Input(PinName upPin, PinName downPin, PinName leftPin, PinName rightPin)
        : up(upPin), down(downPin), left(leftPin), right(rightPin) {
        up.rise(callback(this, &Input::upISR));
        down.rise(callback(this, &Input::downISR));
    }

    // Add more functionality as needed (e.g., left/right handling)
};

#endif // INPUT_H