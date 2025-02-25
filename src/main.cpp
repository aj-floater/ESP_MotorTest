#include "mbed.h"
#include "display.h"
/* Set up serial communication between the HM-10 module and the Nucleo board
 For debugging and setting up the module, set up serial between Nucleo and PC
 Set up LED for debugging*/

 
 // Create a DigitalOutput object to toggle an LED whenever data is received.
 static DigitalOut led(PA_5);
 
//  // Maximum number of element the application buffer can contain
 #define MAXIMUM_BUFFER_SIZE                                                  32

// // Create a BufferedSerial object with a default baud rate.
static BufferedSerial serial_port(PA_11, PA_12);

int main(void)
{
    // Set desired properties (9600-8-N-1).
    serial_port.set_baud(9600);
    serial_port.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );

    // Application buffer to receive the data
    char buf[MAXIMUM_BUFFER_SIZE] = {0};

    // float i = 0;

    while (1) {
        // i += 1;
        // if (i > 50) i = 0;

        if (uint32_t num = serial_port.read(buf, sizeof(buf))) {
            // Toggle the LED.
            led = !led;

            // Echo the input back to the terminal.
            // serial_port.write(buf, num);
        }
        // printf("test");

        // floatToString(i, buf);
        // serial_port.write(buf, sizeof(buf));
    }
}