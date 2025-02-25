#include "mbed.h"
#include "display.h"
#include "hm10.h"

int main(void)
{

    while (1) {
        if (uint32_t num = hm10.read()) {

            // Echo the input to the display plus -group21 string.
            char output[MAXIMUM_BUFFER_SIZE + 10] = {0}; // Extra space for "group21"
            snprintf(output, sizeof(output), "%s%s", hm10.currentReadBuffer, "-group21");
            hm10.write(output);
            display.refresh();
        }
    }
}