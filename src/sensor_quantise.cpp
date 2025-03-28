#include "mbed.h"
#include "stdio.h"
#include "display.h"
#include "read.h"
#include "buffer.h"

#define BUFFER_SIZE 16
#define no_signal 80

int main() {
    bool direction = 0;
    float spec = 0;
    bool line_history[BUFFER_SIZE] = {false}; // Moved outside loop for persistence

    while (true) {
        float la = left_analog_sensor.read() * 3.3;
        float ra = right_analog_sensor.read() * 3.3;

        bool lm = leftmost_digital_sensor.read();
        bool li = left_inner_digital_sensor.read();
        bool ri = right_inner_digital_sensor.read();
        bool rm = rightmost_digital_sensor.read();

        bool left_detected = detect_white_line(left_analog_sensor);
        bool right_detected = detect_white_line(right_analog_sensor);

        bool dactive = lm || li || ri || rm;
        //printf(">dactive: %d ", dactive);  // PRINT dactive

        bool linear = left_detected || right_detected;

        float diff = ra - la;
        float sum = ra + la;

        bool stop = memory(dactive, line_history, BUFFER_SIZE);
        //printf(",stop: %d\n", stop);  // PRINT STOP

        if (linear) {
            if (sum < 0.38) {
                spec = 0;
            } else {
                float dist_eqn = 2.3129f * sum + 2.2961f;
                spec = diff > 0 ? dist_eqn : -dist_eqn;
            }
        } else {
            if (!stop) {
                if (lm)      { spec = -52.0f; direction = 0;}
                else if (li) { spec = -27.0f; direction = 0;}
                else if (ri) { spec =  27.0f; direction = 1;}
                else if (rm) { spec =  52.0f; direction = 1;}
            } else {
                spec = no_signal;
            }
        }

        char specbuffer[20];
        floatToString(spec, specbuffer);
        printf(">spec:%s\n", specbuffer);
    }
}