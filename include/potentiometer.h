#ifndef POTENTIOMETER_H  // If POTENTIOMETER_H is not defined
#define POTENTIOMETER_H  // Define POTENTIOMETER_H

#include "mbed.h"
#include "wheel.h"
#include "display.h"

// Header file content goes here
// Potentiometer class
class Potentiometer {
private:

// Function to truncate a float to 4 decimal places
float truncateTo4DP(float value) const {
    return floor(value * 10000) / 10000.0f;
}

public:
    AnalogIn inputSignal;          // Analog input pin
    float currentSampleNorm;       // Normalized sample (0.0 to 1.0)
    float VDD;                     // Reference voltage
    float range[2];
    bool readSignal;

    Potentiometer(PinName pin, float v) : inputSignal(pin), VDD(v) {
        currentSampleNorm = inputSignal.read();
    }

    void sample(void) {
        readSignal = true;
    }

    float getCurrentSampleNorm(void) {
        // Map currentSampleNorm (0.0 - 1.0) to the custom range
        float mappedValue = range[0] + (currentSampleNorm * (range[1] - range[0]));
        return truncateTo4DP(mappedValue);
    }

    void setRange(float s, float e){
        range[0] = s;
        range[1] = e;
    }
};
  
  // SamplingPotentiometer class inheriting from Potentiometer
class SamplingPotentiometer : public Potentiometer {
private:
    float samplingFrequency;        // Sampling frequency in Hz
    float samplingPeriod;           // Sampling period in seconds
    Ticker sampler;                 // Ticker object for periodic sampling

public:
    SamplingPotentiometer(PinName p, float v, float fs)
        : Potentiometer(p, v), samplingFrequency(fs) {
        samplingPeriod = 1.0f / samplingFrequency;  // Calculate period
        sampler.attach(callback(this, &SamplingPotentiometer::sample), samplingPeriod);
    }

    void update() {
        if (readSignal){
            float input = inputSignal.read();  // Read normalized value
            if (input == currentSampleNorm){
                display.markRefreshNeeded();
            }
    
            currentSampleNorm = input;

            readSignal = false;
        }
    }
};

void modify(float left, float right){
    // right_wheel.proportional_gain = left;
    // left_wheel.proportional_gain = left;
    
    // right_wheel.integral_gain = right;
    // left_wheel.integral_gain = right;
    // right_wheel.derivative_gain = right;
    // left_wheel.derivative_gain = right;

    // ----------------------------

    right_wheel.speed(right);
    left_wheel.speed(left);
}

SamplingPotentiometer LeftPot(A0, 3.3, 10);
SamplingPotentiometer RightPot(A1, 3.3, 10);

#endif // End of include guard