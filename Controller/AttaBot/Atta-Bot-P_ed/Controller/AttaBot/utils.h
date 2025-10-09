#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <map>
#include <array>

// Function prototypes
void SendPositionCommand(float x, float y);
void ProcessPositionCommand(const String& command);
void MoveToPosition(float targetX, float targetY);

// Constants
const char* POSITION_COMMAND_FORMAT = "POSITIONGT|%.2f|%.2f";

#endif // UTILS_H