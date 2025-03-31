#include "mbed.h"
#include "stdio.h"
#include "display.h"
#include "read.h"

#define BUFFER_SIZE 16
#define no_signal 80
#define FILTER_WINDOW 8
#define THRESHOLD 2.8

class MovingAverfiltergeFilter {
public:
    MovingAverfiltergeFilter() : index(0), sum(0.0f) {
        for (int i = 0; i < FILTER_WINDOW; ++i) buffer[i] = 0.0f;
    }

    float filter(float new_value) {
        sum -= buffer[index];
        buffer[index] = new_value;
        sum += new_value;
        index = (index + 1) % FILTER_WINDOW;
        return sum / FILTER_WINDOW;
    }

private:
    float buffer[FILTER_WINDOW];
    int index;
    float sum;
};

struct SpecResult {
    float spec;
    bool stop;
};

SpecResult LineDistance(MovingAverfiltergeFilter& left_filter, MovingAverfiltergeFilter& right_filter, bool* line_history) {
    float la = left_analog_sensor.read();
    float ra = right_analog_sensor.read();

    float lfilter = left_filter.filter(la);
    float rfilter = right_filter.filter(ra);

    bool lm = leftmost_digital_sensor.read();
    bool li = left_inner_digital_sensor.read();
    bool ri = right_inner_digital_sensor.read();
    bool rm = rightmost_digital_sensor.read();

    bool left_detected = (lfilter * 3.3 < THRESHOLD);
    bool right_detected = (rfilter * 3.3 < THRESHOLD);

    bool dactive = !(lm && li && ri && rm);
    bool linear = left_detected || right_detected;

    float diff = rfilter - lfilter;
    float sum = rfilter + lfilter;

    bool stop = memory(dactive, line_history, BUFFER_SIZE);

    float spec = 0.0f;
    bool direction = 0;

    if (linear) {
        if (sum < 0.38f) {
            spec = 0.0f;
        } else {
            float dist_eqn = 2.3129f * 3.3 * sum + 2.2961f;
            spec = diff > 0 ? dist_eqn : -dist_eqn;
        }
    } else {
        if (!lm)      { spec = 52.0f; direction = 0; }
        else if (!li) { spec = 27.0f; direction = 0; }
        else if (!ri) { spec = -27.0f; direction = 1; }
        else if (!rm) { spec = -52.0f; direction = 1; }
        else {
            if (stop) {
                return {spec, true};
            }
        }
    }

    return {spec, false};
}

int main() {
    bool line_history[BUFFER_SIZE] = {false};
    MovingAverfiltergeFilter left_filter;
    MovingAverfiltergeFilter right_filter;

    while (true) {
        SpecResult result = LineDistance(left_filter, right_filter, line_history);

        // Optional debug print
        char specbuffer[20];
        floatToString(result.spec, specbuffer);
        printf(">spec:%s, No_line: %i\n", specbuffer, result.stop);
    }
} 
