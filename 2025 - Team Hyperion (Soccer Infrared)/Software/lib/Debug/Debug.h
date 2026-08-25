#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

#define MAX_DEBUG_MODES 20

/**
 * Simple serial-based debug system.
 * Type commands into Serial to start printing values.
 * Supports multiple commands separated by commas.
 * 
 * Example usage:
 *   debug.init(9600);
 *   debug.addMode("hc", hcDist);
 *   debug.addMode("rg", randomGetter);
 *   debug.setInterval(500);
 *   debug.update(); // call in loop()
 */
class Debug {
public:
    void init(long baudRate = 9600);   // start serial debug
    void update();                     // check serial + print values

    // Register callbacks that return a printable String
    bool addMode(const String& command, String (*func)());
    void setInterval(unsigned long ms);// set min ms between prints

private:
    void stop();                       // stop printing
    struct Mode {
        String command;
        String (*func)();
    };

    Mode modes[MAX_DEBUG_MODES];
    int modeCount = 0;

    String serialMode = "none"; // active comma-separated list
    unsigned long lastPrint = 0;
    unsigned long printInterval = 0;
};

#endif
