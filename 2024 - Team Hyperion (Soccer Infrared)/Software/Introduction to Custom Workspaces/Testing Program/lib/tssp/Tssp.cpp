#include "Tssp.h"

Tssp::Tssp(int p) {
    pin = p;
}

void Tssp::init() {
    pinMode(pin, INPUT);
}

int Tssp::read() {
    int value = digitalRead(pin);
    return value;
}