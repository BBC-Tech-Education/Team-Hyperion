#include "Debug.h"

/**
 * Start the debug system with a given baud rate.
 * Example: debug.init(9600);
 */
void Debug::init(long baudRate) {
    Serial.begin(baudRate);
}

/**
 * Register a new command and its callback.
 * Command is typed into Serial, callback returns String to print.
 * Returns false if mode list is full.
 */
bool Debug::addMode(const String& command, String (*func)()) {
    if (modeCount >= MAX_DEBUG_MODES) return false;
    modes[modeCount++] = { command, func };
    return true;
}

/**
 * Stop all debug printing (sets mode to "none").
 */
void Debug::stop() {
    serialMode = "none";
}

/**
 * Set time (ms) between lines of output.
 * Example: debug.setInterval(500); // print every 0.5s
 */
void Debug::setInterval(unsigned long ms) {
    printInterval = ms;
}

/**
 * Call this in loop(). 
 * - Reads new serial commands
 * - Splits multiple commands with commas
 * - Calls matching functions and prints all results on one line
 */
void Debug::update() {
    // handle incoming commands
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() == 0) return;
        if (input == "stop") {
            stop();
            Serial.println("Stopped serial output");
        } else {
            serialMode = input; // keep raw comma-separated string
            Serial.println("Switched to mode(s): " + serialMode);
        }
    }

    if (serialMode == "none") return;

    // throttle prints if interval set
    if (printInterval > 0) {
        unsigned long now = millis();
        if (now - lastPrint < printInterval) return;
        lastPrint = now;
    }

    // split serialMode by commas and build one output line
    String outLine;
    int start = 0;
    while (true) {
        int idx = serialMode.indexOf(',', start);
        String token = (idx == -1) ? serialMode.substring(start) : serialMode.substring(start, idx);
        token.trim();

        if (token.length() > 0) {
            bool found = false;
            for (int i = 0; i < modeCount; i++) {
                if (token == modes[i].command && modes[i].func != nullptr) {
                    String val = modes[i].func();
                    if (outLine.length() > 0) outLine += ", ";
                    outLine += val;
                    found = true;
                    break;
                }
            }
            if (!found) {
                if (outLine.length() > 0) outLine += ", ";
                outLine += "<unknown:" + token + ">";
            }
        }

        if (idx == -1) break;
        start = idx + 1;
    }

    Serial.println(outLine); // single line with all values for this cycle
}
