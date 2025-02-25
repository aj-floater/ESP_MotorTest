#include "mbed.h"
#include "stdio.h"
#include "display.h"

#define NUM_READINGS 10  // Number of initial calibration samples
#define SCALE_FACTOR 5.0  // Adjust for 0-5V range
#define BUFFER_SIZE 20  // Rolling buffer size for handling line breaks
#define LINE_LOSS_DELAY_MS 100  // Delay tolerance for line breaks
#define THRESHOLD 3.0



// Analog Sensors
AnalogIn left_analog_sensor(A3); 
AnalogIn right_analog_sensor(A2); 

// Digital Sensors
DigitalIn leftmost_digital_sensor(A5, PullUp); //up   
DigitalIn left_inner_digital_sensor(A4, PullUp); //down
DigitalIn right_inner_digital_sensor(A1, PullUp); //left
DigitalIn rightmost_digital_sensor(A0, PullUp); //right

// Variables for dynamic thresholding
float left_max_val = 0.0, left_min_val = 5.0;
float right_max_val = 0.0, right_min_val = 5.0;
float left_dynamic_threshold, right_dynamic_threshold;


// Function to read and scale sensor values
float read_sensor(AnalogIn &sensor) {
    return sensor.read() * SCALE_FACTOR;  // Scale the ADC reading to 0-5V
}
// Function to detect white line with dynamic threshold updates
bool detect_white_line(AnalogIn &sensor) {
    float sensor_value = read_sensor(sensor);

    return (sensor_value < THRESHOLD);  // True if white line detected
}

// Rolling history buffer to handle short line breaks
bool left_sensor_history[BUFFER_SIZE] = {true};
bool right_sensor_history[BUFFER_SIZE] = {true};

bool check_line_with_memory(bool current_reading, bool history[], int buffer_size) {
    // Shift the buffer values to make space for the new reading
    for (int i = buffer_size - 1; i > 0; i--) {
        history[i] = history[i - 1];
    }
    history[0] = current_reading;  // Store the latest reading

    // If any recent reading in the buffer was `true`, assume the line still exists
    for (int i = 0; i < buffer_size; i++) {
        if (history[i]) {
            return true;  // If any past detection was true, we assume the line is still there
        }
    }
    
    // If the buffer contains only `false`, then we assume the line is truly lost
    return false;
}

int main() { 
    int direction = 0;

    while (true) {
        // Digital Read
        int leftmost_value = leftmost_digital_sensor.read();
        int left_inner_value = left_inner_digital_sensor.read();
        int right_inner_value = right_inner_digital_sensor.read();
        int rightmost_value = rightmost_digital_sensor.read();
        // Read
        bool left_detected = detect_white_line(left_analog_sensor);
        bool right_detected = detect_white_line(right_analog_sensor);
        
        //left deviation slight
        if (!left_detected && right_detected) {
            printf("Lean Left\n");
            direction = -1;
            //WRITE____________________________________________________________>
        // right deviation slight
        } else if (!right_detected && left_detected) {
            printf("Lean Right\n");
            direction = 1;
            //WRITE____________________________________________________________>
        // left deviation medium
        } else if (!right_inner_value) {
            printf("Left\n");
            direction = -1;
        // right deviation medium
        } else if (!left_inner_value) {    
            printf("Right\n");
            direction = 1;
            
        // left deviation lot
        } else if (!rightmost_value) {
            printf("ProLeft\n");
            direction = -1;
        // right deviation lot
        } else if (!leftmost_value) {    
            printf("ProRight\n");
            direction = 1;
        } else {
            // Update rolling history
            bool on_line = check_line_with_memory(left_detected && right_detected, left_sensor_history, BUFFER_SIZE) ||
                           check_line_with_memory(left_detected && right_detected, right_sensor_history, BUFFER_SIZE);
            
            if (on_line) {
                printf("On line\n");
            } else {
                printf("No Line ");
                if (direction == -1){
                    printf("Should turn right\n");
                }else if(direction == 0){
                    printf("End\n");
                }else if(direction == 1){
                    printf("Should turn left\n");
                }
            }
        }
        
        wait_us(500); // Small delay for stability (50ms)
    }
}
