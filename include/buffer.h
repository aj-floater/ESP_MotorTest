#include "mbed.h"

bool memory(bool current_reading, bool history[], int buffer_size) {
    for (int i = buffer_size - 1; i > 0; --i) {
        history[i] = history[i - 1];
    }
    history[0] = current_reading;

    for (int i = 0; i < buffer_size; ++i) {
        if (!history[i]) return false;
    }
    return true;
}
