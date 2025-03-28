#include "mbed.h"
#include "stdio.h"

#define THRESHOLD 2.8

// Analog Sensors
AnalogIn left_analog_sensor(A0); //A3
AnalogIn right_analog_sensor(A1); //A2

// Digital Sensors
DigitalIn leftmost_digital_sensor(A2, PullDown); //up A5
DigitalIn left_inner_digital_sensor(A3, PullDown); //down A4
DigitalIn right_inner_digital_sensor(A4, PullDown); //left A1
DigitalIn rightmost_digital_sensor(A5, PullDown); //right A0

float read_sensor(AnalogIn &sensor) {
    return sensor.read()*3.3;
}

// Function to detect white line
bool detect_white_line(AnalogIn &sensor) {
    float sensor_value = read_sensor(sensor);
    return (sensor_value < THRESHOLD);
}