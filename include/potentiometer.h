#ifndef POTENTIOMETER_H  // If POTENTIOMETER_H is not defined
#define POTENTIOMETER_H  // Define POTENTIOMETER_H

#include "mbed.h"
#include "display.h"

// Header file content goes here
// Potentiometer class
class Potentiometer {
  private:
      AnalogIn inputSignal;          // Analog input pin
      float VDD;                     // Reference voltage
      float currentSampleNorm;       // Normalized sample (0.0 to 1.0)
      float currentSampleVolts;      // Sample in volts
  
      // Function to truncate a float to 2 decimal places
      float truncateTo2DP(float value) const {
          return floor(value * 100) / 100.0f;
      }
  
  public:
      Potentiometer(PinName pin, float v) : inputSignal(pin), VDD(v) {
          currentSampleNorm = inputSignal.read();
      }
  
      void sample(void) {
          if (inputSignal.read() == currentSampleNorm){
              display.markRefreshNeeded();
          }
  
          currentSampleNorm = inputSignal.read();  // Read normalized value
          currentSampleVolts = currentSampleNorm * VDD; // Convert to volts
      }
  
      float getCurrentSampleNorm(void) {
          return truncateTo2DP(currentSampleNorm);
      }
  
      float getCurrentSampleVolts(void) {
          return currentSampleVolts;
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
  };

#endif // End of include guard