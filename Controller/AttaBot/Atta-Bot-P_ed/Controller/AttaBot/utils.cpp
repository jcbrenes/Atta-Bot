#include "utils.h"
#include <WiFi.h>
#include <WiFiUdp.h>

extern WiFiUDP udp;
extern std::map<String, IPAddress> robots;

void SendPositionCommand(float targetX, float targetY) {
    // Construct the command string
    char command[50];
    snprintf(command, sizeof(command), "POSITIONGT|%.2f|%.2f", targetX, targetY);

    // Send the command to all robots
    for (const auto& robot : robots) {
        if (robot.first != "Base") { // Avoid sending to the base
            SendMessage(robot.second, command);
        }
    }
}

void SendMessage(IPAddress host, const char* message) {
    udp.beginPacket(host, localPort);
    udp.write(reinterpret_cast<const uint8_t*>(message), strlen(message));
    udp.endPacket();
}