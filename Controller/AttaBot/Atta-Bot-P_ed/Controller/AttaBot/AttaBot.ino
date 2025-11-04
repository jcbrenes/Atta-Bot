#include "utils.h"
#include <Adafruit_APDS9960.h>
#include <EEPROM.h>
#include <FastLED.h>
#include <ICM_20948.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <deque>
#include <map>

#define leftMotorForward 12
#define leftMotorBackward 14
#define rightMotorForward 13
#define rightMotorBackward 15

#define leftEncoderC1 32
#define leftEncoderC2 35
#define rightEncoderC1 23
#define rightEncoderC2 25

#define enableLeftInfraredSensor 5
#define leftInfraredSensor 33
#define frontServoPin 26
#define enableRightInfraredSensor 18
#define rightInfraredSensor 27

#define batteryStatus 19
#define ledPin 2
#define AD0_VAL 1
#define NUM_LEDS 1  

const char* ssid = "Atta-Bot";
const char* password = "attabot1234";
const unsigned int localPort = 6060;
char receivedPacket[255];

float pulsesPerRev = 820;
const float wheelCircumference = PI * 44.5;
float millimetersPerPulse = wheelCircumference / (float)pulsesPerRev;
const float centerToWheelDistance = 41.5;

const unsigned int samplingTime = 10;
unsigned long currentMillis = millis();
unsigned long previousMillis = millis();
volatile int leftPulseCount = 0;
volatile int rightPulseCount = 0;
float currentLeftSpeed = 0.0;
float currentRightSpeed = 0.0;

enum PossibleStates { WAIT = 0, MOVE, TURN, STOP, POSITIONGT };
PossibleStates state = WAIT;

void setup() {
  // Initialization code...
}

void loop() {
  switch (state) {
    case WAIT:
      // Waiting for commands...
      break;

    case POSITIONGT:
      // Logic to handle POSITIONGT command...
      break;

    // Other states...
  }
}

void SendPositionCommand(float x, float y) {
  char command[50];
  snprintf(command, sizeof(command), "POSITIONGT|%.2f|%.2f", x, y);
  for (const auto& robot : robots) {
    SendMessage(robot.second, command);
  }
}

void ReadUdpPackets() {
  int packetBytes = udp.parsePacket();
  if (!packetBytes) return;

  int len = udp.read(receivedPacket, sizeof(receivedPacket) - 1);
  if (len > 0) {
    receivedPacket[len] = '\0';
    String command(receivedPacket);
    
    if (command.startsWith("POSITIONGT|")) {
      // Extract x and y values and move to the position
      int firstPipe = command.indexOf('|', 11);
      int secondPipe = command.indexOf('|', firstPipe + 1);
      if (firstPipe != -1 && secondPipe != -1) {
        float x = command.substring(firstPipe + 1, secondPipe).toFloat();
        float y = command.substring(secondPipe + 1).toFloat();
        // Logic to move to (x, y)
      }
    }
  }
}

// Other functions...