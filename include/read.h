#include "mbed.h"
#include "stdio.h"

#define THRESHOLD 2.8

// Analog Sensors
AnalogIn left_analog_sensor(A0); 
AnalogIn right_analog_sensor(A1); 

// Digital Sensors
DigitalIn leftmost_digital_sensor(A2, PullDown); //up   
DigitalIn left_inner_digital_sensor(A3, PullDown); //down
DigitalIn right_inner_digital_sensor(A4, PullDown); //left
DigitalIn rightmost_digital_sensor(A5, PullDown); //right

float read_sensor(AnalogIn &sensor) {
    return sensor.read()*3.3;
}

// Function to detect white line
bool detect_white_line(AnalogIn &sensor) {
    float sensor_value = read_sensor(sensor);
    return (sensor_value < THRESHOLD);
}

// FIX
bool check_line_with_memory(bool current_reading, bool history[], int buffer_size) {
    for (int i = buffer_size - 1; i > 0; i--) {
        history[i] = history[i - 1];
    }//Shift the value to the right
    history[0] = current_reading;//Add the current value

    for (int i = 0; i < buffer_size; i++) {
        if (history[i]) {
            return true;  
        }
    }
    return false;//find if there are true
}