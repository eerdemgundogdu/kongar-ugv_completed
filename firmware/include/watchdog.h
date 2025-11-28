#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <Arduino.h>

class Watchdog {
private:
    unsigned long lastCommandTime;
    unsigned long timeoutMs;
    bool enabled;

public:
    Watchdog(unsigned long timeout_ms);
    void feed();
    bool isTimedOut();
    void enable();
    void disable();
};

#endif
