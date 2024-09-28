#ifndef UART_STREAM_HPP
#define UART_STREAM_HPP

// Include any necessary headers here
#include <ArduinoJson.h>


class UartStream {
public:
    // Constructor and destructor
    UartStream(int tx, int rx, int baud, void (*publish)(String));

    // Function declarations

private:
    // Private member variables

    // Private function declarations
};

#endif // UART_STREAM_HPP