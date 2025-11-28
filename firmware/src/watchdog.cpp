#include "watchdog.h"

Watchdog::Watchdog(unsigned long timeout_ms) 
    : timeoutMs(timeout_ms), enabled(true) {
    lastCommandTime = millis();
}

void Watchdog::feed() {
    lastCommandTime = millis();
}

bool Watchdog::isTimedOut() {
    if (!enabled) return false;
    return (millis() - lastCommandTime) > timeoutMs;
}

void Watchdog::enable() {
    enabled = true;
    lastCommandTime = millis();
}

void Watchdog::disable() {
    enabled = false;
}
