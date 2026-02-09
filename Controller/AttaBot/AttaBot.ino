#include "utils.h"
#include <Adafruit_APDS9960.h> // v1.3.0
#include <FastLED.h> // v3.10.2
#include <ICM_20948.h> // v1.2.12
#include <ESP32Servo.h> // v3.0.9
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Preferences.h>
#include <EEPROM.h>

// ============================================================================
// CONFIGURACIÓN DE DEBUG
// ============================================================================

// Descomentar solo para debug
//#define DebugSerial
#ifdef DebugSerial
  #define DebugSerialPrint(x) Serial.print(x)
  #define DebugSerialPrintln(x) Serial.println(x)
  #define DebugSerialPrintf(x, ...) Serial.printf(x, ##__VA_ARGS__)
#else
  #define DebugSerialPrint(x)
  #define DebugSerialPrintln(x)
  #define DebugSerialPrintf(x, ...)
#endif


// ============================================================================
// DEFINICIÓN DE PINES
// ============================================================================

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

#define pwm_freq 1000
#define pwm_resolution 14


// ============================================================================
// CONSTANTES DEL SISTEMA
// ============================================================================

// WiFi
const char* ssid = "Atta-Bot";
const char* password = "attabot1234";
const unsigned int localPort = 6060;
char receivedPacket[255];

// Constantes del robot
float pulsesPerRev = 574;
const float wheelCircumference = PI * 44.5;
float millimetersPerPulse = wheelCircumference / pulsesPerRev;
const float centerToWheelDistance = 41.5;

// Muestreo y velocidad
const unsigned int samplingTime = 10;
const float samplingTimeS = samplingTime * 0.001;
const unsigned int SteadyStateTime = 800;
const float distanceOffset = 1 * millimetersPerPulse;
const float baseSpeed = millimetersPerPulse / samplingTimeS;
const float maxSpeed = baseSpeed * 5;
const float minSpeed = 12;
const int speedReductionThreshold = 16;

// Control PID
const int maxPWMValue = (1 << pwm_resolution) - 1;
const int minPWMValue = maxPWMValue * 0.20;
pidConstants pidSpeed(110, 375, 2);
kalmanFilter kfPID(6.0, 1.0, 1.0);

// Sensores de obstáculos
const int observationPeriod = 28800;
const int observationTime = 1600;
const int numberOfCycles = observationPeriod / observationTime;
const int lateralCycle = random(numberOfCycles);
const int centralCycle = (lateralCycle + random(1, numberOfCycles)) % numberOfCycles;
const unsigned minObstacleTime = 1350;

// Detección de robots
const float robotDistanceMargin = 260;
const float maxRobotAngleMargin = 80;
const int obstacleWaitTime = 600;
const int reverseDistance = -40;

// Debug
int debugUdp = 0;
int debugCounter = 0;
char direction = '+';
const char* debugMessage = "DEBUG: %d, ID: %s, Direccion: %c, val: Izq|Der, Encoder: %d|%d, Vel: %.2f|%.2f, Pwm: %d|%d, ErrorP: %.2f|%.2f, ErrorI: %.2f|%.2f, Dis: %.2f|%.2f, Tiempo: %d";

// Random Walk
const std::array<int, 7> possibleAngles = { 30, 45, 60, 75, 90, 135, 180 };
const std::array<int, 4> possibleAdvances = { 200, 250, 300, 350 };
enum possibleDirections { TURN_POS = 0, MOVE_FORWARD, TURN_NEG };

// Límites del área de trabajo (en milímetros)
const float max_workspace_x = 2000;
const float max_workspace_y = 2000;

// Filtro de saltos bruscos en actualización de pose
const float max_pose_jump = 500;  // Máximo salto permitido en mm por actualización
const float max_angle_jump = 90;  // Máximo salto permitido en grados

// IMU
const float gravity = 9806.65;
const float conversionFactor = 8192.0;
float yaw;
float imuGravity;
biasStore store;

// LEDs
int maxBrightness = 140;

// Batería
volatile unsigned long lowBatteryTime = 0;
int minLowBatteryTime = 200;

// Contador de mensajes
int countMessages = 0;
int sendMessages = 0;

// Servo
bool frontSensorInitialized = false;
unsigned long lastFrontSensorAttempt = 0;
const unsigned long frontSensorRetryInterval = 5000;  // Reintentar cada 5 segundos
volatile bool lateralSensorsEnabled = false;


// ============================================================================
// VARIABLES GLOBALES REFACTORIZADAS (usando estructuras de utils.h)
// ============================================================================

NavigationTarget navTarget;
InterruptionContext intContext;
EvasionTracker evasionTracker;
CongregationState congregation;
ObstacleState obstacles;
MovementMetrics movement;
LedController ledCtrl;

// Variables de control de movimiento
unsigned long currentMillis = millis();
unsigned long previousMillisRW = 0;
int millisDifference;
int pastLeftEncoder = 0;
int pastRightEncoder = 0;

// Variables de sensores
unsigned long currentMicros = micros();
unsigned long previousMicros = micros();
bool isLateralCycleActive = false;
bool isCentralCycleActive = false;
int cycleCounter = 0;
int microsDifference;
volatile unsigned long leftObsStartTime = 0;
volatile unsigned long rightObsStartTime = 0;
unsigned long centralObsStartTime = 0;
int centralDistance;

// Estado del robot
String robotID = "-1";
std::map<String, IPAddress> robots;
RobotState state = WAIT;
float instructionValue = 100;
bool movementReady = true;
const int instructionCompletedDelay = 400;
std::array<float, 2> fsmInstruction;
std::deque<std::array<float, 2>> instructionList;
pose robotPose(0, 0, 0);

// Evasión
bool isEvading = false;
unsigned long evasionStartTime = 0;
const unsigned long evasionCooldown = 2000;
bool resumeScheduled = false;
bool obstacleDetected = false;

// Controladores PID
pidController leftControl(kfPID, pidSpeed, samplingTimeS, minPWMValue, maxPWMValue);
pidController rightControl(kfPID, pidSpeed, samplingTimeS, minPWMValue, maxPWMValue);

// Hardware
Servo frontServo;
Adafruit_APDS9960 frontSensor;
CRGB leds[NUM_LEDS];
WiFiUDP udp;
ICM_20948_I2C imu;
Preferences preferences;


// ============================================================================
// DECLARACIONES FORWARD DE FUNCIONES
// ============================================================================

// Setup y configuración
void SetupFrontSensor();
void WiFiStatus();
void setupIMU();
void updateMillimetersPerPulse();
void InitializePPR();
void SavePPR(float newPPR);

// Comunicación
void ReadUdpPackets();
void SendMessage(IPAddress host, const char* message);
void SendPose();
void MessageDebugf(const char* format, ...);
void CommunicationTest();

// Sensores y control
void ReadSensors();
void ResetPID();
void ConfigureHBridge(int leftWheelPWM, int rightWheelPWM);

// Movimiento
bool MoveDistanceByWheel(float leftDistance, float rightDistance);
float DesiredSpeed(float distance, float wheelDistance);
bool IsStationary(float currentLeftSpeed, float currentRightSpeed, float leftWheelDistance, float rightWheelDistance);
void SelectMovementRW();

// Navegación iterativa
void InitiateIterativeNavigation(float targetX, float targetY);
void CalculateIterativeMovement();

// Auxiliares
std::array<String, 5> SeparateCommand(const String& command, char delimiter);
bool IsRobotObstacle(float x2, float y2, float angle, int sensors, String id);
void ReadSerialCommands();

// LED
void setLedColor(uint8_t red, uint8_t green, uint8_t blue);
void setLedBrightness(uint8_t brightness);
void setLedBlink(uint8_t red, uint8_t green, uint8_t blue, unsigned long intervalMs);

// IMU
bool StoreValido(biasStore* store);
void LeerYaw();


// ============================================================================
// INTERRUPCIONES (ISR)
// ============================================================================

void IRAM_ATTR LeftWheelPulses() {
  int MSB = digitalRead(leftEncoderC2);
  int LSB = digitalRead(leftEncoderC1);
  int encoder = (MSB << 1) | LSB;
  int sum = (pastLeftEncoder << 2) | encoder;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    movement.leftPulseCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    movement.leftPulseCount--;
  }
  pastLeftEncoder = encoder;
}

void IRAM_ATTR RightWheelPulses() {
  int MSB = digitalRead(rightEncoderC1);
  int LSB = digitalRead(rightEncoderC2);
  int encoder = (MSB << 1) | LSB;
  int sum = (pastRightEncoder << 2) | encoder;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    movement.rightPulseCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    movement.rightPulseCount--;
  }
  pastRightEncoder = encoder;
}

void IRAM_ATTR DetectLeftObstacle() {
  if (lateralSensorsEnabled && digitalRead(leftInfraredSensor) == LOW) {
    leftObsStartTime = micros();
  }
}

void IRAM_ATTR DetectRightObstacle() {
  if (lateralSensorsEnabled && digitalRead(rightInfraredSensor) == LOW) {
    rightObsStartTime = micros();
  }
}

void LowBattery() {
  if (digitalRead(batteryStatus) == LOW) {
    lowBatteryTime = millis();
  }
}


// ============================================================================
// FUNCIONES DE SETUP Y CONFIGURACIÓN
// ============================================================================

void InitializePPR() {
  preferences.begin("attabot-config", false);
  
  float storedPPR = preferences.getFloat("ppr", 0);
  
  if (storedPPR == 0) {
    preferences.putFloat("ppr", pulsesPerRev);
    DebugSerialPrintf("PPR inicial guardado: %.2f\n", pulsesPerRev);
  } else {
    pulsesPerRev = storedPPR;
    DebugSerialPrintf("PPR cargado desde memoria: %.2f\n", pulsesPerRev);
  }
  
  updateMillimetersPerPulse();
  preferences.end();
  
  uint64_t chipid = ESP.getEfuseMac();
  DebugSerialPrintf("Robot Chip ID: %04X%08X\n", 
                    (uint16_t)(chipid>>32), 
                    (uint32_t)chipid);
}

void SavePPR(float newPPR) {
  preferences.begin("attabot-config", false);
  preferences.putFloat("ppr", newPPR);
  preferences.end();
  DebugSerialPrintf("PPR guardado permanentemente: %.2f\n", newPPR);
}

void updateMillimetersPerPulse() {
  millimetersPerPulse = wheelCircumference / pulsesPerRev;
}

void setup() {
  #ifdef DebugSerial
    Serial.begin(115200);
    delay(500);  // Dar tiempo al Serial Monitor para conectar
    Serial.println("\n\n=== INICIO DE SETUP ===");
  #endif

  // Delay aleatorio para evitar colisiones DHCP cuando múltiples ESP32 arrancan juntos
  // Usa la MAC address como semilla para que cada robot tenga un delay único
  randomSeed(ESP.getEfuseMac());
  unsigned long startupDelay = random(100, 2000);  // Entre 100ms y 2 segundos
  DebugSerialPrintf("Esperando %lu ms antes de iniciar WiFi...\n", startupDelay);
  delay(startupDelay);

  DebugSerialPrintln("[1] Inicializando PPR desde flash...");
  InitializePPR();
  DebugSerialPrintln("[1] PPR OK");

  DebugSerialPrintln("[2] Inicializando servo...");
  frontServo.setPeriodHertz(50);
  frontServo.attach(frontServoPin, 1000, 2000);
  frontServo.write(90);
  DebugSerialPrintln("[2] Servo OK");

  DebugSerialPrintln("[3] Inicializando motores PWM...");
  ledcAttach(leftMotorForward, pwm_freq, pwm_resolution);
  ledcAttach(leftMotorBackward, pwm_freq, pwm_resolution);
  ledcAttach(rightMotorForward, pwm_freq, pwm_resolution);
  ledcAttach(rightMotorBackward, pwm_freq, pwm_resolution);
  DebugSerialPrintln("[3] Motores OK");

  DebugSerialPrintln("[4] Inicializando I2C...");
  Wire.begin();
  Wire.setClock(400000);
  DebugSerialPrintln("[4] I2C OK");

  // IMPORTANTE: setHostname DEBE estar ANTES de WiFi.begin()
  DebugSerialPrintln("[5] Inicializando WiFi...");
  WiFi.mode(WIFI_STA);
  String hostname = "AttaBot-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  WiFi.setHostname(hostname.c_str());

  DebugSerialPrintf("Iniciando WiFi con hostname: %s\n", hostname.c_str());
  WiFi.begin(ssid, password);
  DebugSerialPrintln("[5] WiFi iniciado");

  DebugSerialPrintln("[6] Inicializando LEDs...");
  FastLED.addLeds<WS2812, ledPin, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(maxBrightness);
  FastLED.setMaxRefreshRate(120);
  ledCtrl.setOff();
  DebugSerialPrintln("[6] LEDs OK");

  DebugSerialPrintln("[7] Inicializando OTA y UDP...");
  ArduinoOTA.setHostname(hostname.c_str());
  ArduinoOTA.begin();

  udp.begin(localPort);
  DebugSerialPrintf("El servidor UDP se inició en el puerto: %u\n", localPort);
  DebugSerialPrintln("[7] OTA y UDP OK");

  DebugSerialPrintln("[8] Configurando encoders...");
  pinMode(leftEncoderC1, INPUT_PULLUP);
  pinMode(leftEncoderC2, INPUT);  // GPIO 35 es input-only, sin pull-up (compatible con ESP32 Core 3.x)
  pinMode(rightEncoderC1, INPUT_PULLUP);
  pinMode(rightEncoderC2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderC1), LeftWheelPulses, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC2), LeftWheelPulses, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC1), RightWheelPulses, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC2), RightWheelPulses, CHANGE);
  DebugSerialPrintln("[8] Encoders OK");

  DebugSerialPrintln("[9] Configurando sensores infrarrojos...");
  pinMode(enableLeftInfraredSensor, OUTPUT);
  pinMode(enableRightInfraredSensor, OUTPUT);

  digitalWrite(enableLeftInfraredSensor, LOW);
  digitalWrite(enableRightInfraredSensor, LOW);

  pinMode(batteryStatus, INPUT);
  attachInterrupt(digitalPinToInterrupt(batteryStatus), LowBattery, FALLING);
  pinMode(leftInfraredSensor, INPUT);
  pinMode(rightInfraredSensor, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftInfraredSensor), DetectLeftObstacle, FALLING);
  attachInterrupt(digitalPinToInterrupt(rightInfraredSensor), DetectRightObstacle, FALLING);
  DebugSerialPrintln("[9] Sensores OK");

  DebugSerialPrintln("\n=== SETUP COMPLETO ===\n");

  
  SetupFrontSensor();
  delay(200);
}


// ============================================================================
// LOOP PRINCIPAL
// ============================================================================

void loop() {
  ledCtrl.update();
  WiFiStatus();
  if (WiFi.status() != WL_CONNECTED) return;
  ReadUdpPackets();
  ReadSensors();
  SetupFrontSensor(); 
  
  #ifdef DebugSerial
    ReadSerialCommands();
  #endif

  switch (state) {
    case WAIT: {
      ArduinoOTA.handle();

      if ((millis() - movement.previousMillis) >= instructionValue) {
        movement.previousMillis = millis();
        ResetPID();
        if (movementReady) {
          state = READ_INSTRUCTION;
        } else {
          direction = '-';
          state = REVERSE;
        }
      }

      break;
    }

    case MOVE: {
      movementReady = MoveDistanceByWheel(instructionValue, instructionValue);

      if (movementReady) {
        MessageDebugf("DEBUG: -1, ID: %s, Movimiento completado", robotID.c_str());
        intContext.Clear();
        isEvading = false;
        state = STOP;
          
      } else if (obstacles.HasAnyObstacle() && !isEvading) {
          obstacles.UpdateBitmap();
          
          intContext.wasInterrupted = true;
          intContext.previousState = MOVE;
          intContext.leftPulsesBeforeStop = movement.pastLeftPulseCount;
          intContext.rightPulsesBeforeStop = movement.pastRightPulseCount;
          
          float avgTraveled = (movement.pastLeftPulseCount + movement.pastRightPulseCount) / 2.0 * millimetersPerPulse;
          intContext.remainingValue = instructionValue - avgTraveled;

          isEvading = true;
          evasionStartTime = millis();
          
          MessageDebugf("DEBUG: -1, ID: %s, MOVE interrumpido: restante=%.1fmm", 
                        robotID.c_str(), intContext.remainingValue);
          
          state = STOP;
      }

      break;
    }

    case TURN: {
      movementReady = MoveDistanceByWheel(instructionValue, -instructionValue);

      if (movementReady) {
        MessageDebugf("DEBUG: -1, ID: %s, Giro completado", robotID.c_str());
        intContext.Clear();
        isEvading = false;
        state = STOP;
          
      } else if (obstacles.HasAnyObstacle() && !isEvading) {
        obstacles.UpdateBitmap();
        
        intContext.wasInterrupted = true;
        intContext.previousState = TURN;
        intContext.leftPulsesBeforeStop = movement.pastLeftPulseCount;
        
        float traveled = movement.pastLeftPulseCount * millimetersPerPulse;
        intContext.remainingValue = instructionValue - traveled;

        isEvading = true;
        evasionStartTime = millis();
        
        float angleRemaining = (intContext.remainingValue / centerToWheelDistance) * RAD_TO_DEG;
        MessageDebugf("DEBUG: -1, ID: %s, TURN interrumpido: restante=%.1f°", 
                      robotID.c_str(), angleRemaining);

        if (obstacles.centralObstacle) {
          movementReady = false;
        }
        
        state = STOP;
      }

      break;
    }

    case RANDOM_WALK: {
      if (previousMillisRW == 0) {
        previousMillisRW = millis();
      }

      currentMillis = millis();
      millisDifference = currentMillis - previousMillisRW;
      if (millisDifference < instructionValue) {
        previousMillisRW = currentMillis;
        fsmInstruction[0] = RANDOM_WALK;
        fsmInstruction[1] = instructionValue - millisDifference;
        instructionList.push_front(fsmInstruction);
        SelectMovementRW();
      } else {
        previousMillisRW = 0;
        MessageDebugf("DEBUG: -1, ID: %s, Random Walk terminado", robotID.c_str());
      }

      state = READ_INSTRUCTION;
      break;
    }

    case REVERSE: {
      movementReady = MoveDistanceByWheel(reverseDistance, reverseDistance);

      if (movementReady) {
          obstacleDetected = true;
          MessageDebugf("DEBUG: -1, ID: %s, Retroceso completado", robotID.c_str());
          state = STOP;
      }
      else if (obstacles.HasAnyObstacle() && !isEvading) {
          MessageDebugf("DEBUG: -1, ID: %s, Obstáculo durante retroceso!", robotID.c_str());
          state = STOP;
      }

      break;
    }

    case STOP: {
      ConfigureHBridge(0, 0);
      
      if (movementReady == true) {
        if (!intContext.wasInterrupted) {
          isEvading = false;
          resumeScheduled = false;
        }
        
        state = WAIT;
        instructionValue = instructionCompletedDelay;
          
      } else {
        SendPose();
        state = IDENTIFY_OBSTACLE;
      }
      
      break;
    }

    case READ_INSTRUCTION: {
      if (!isEvading && !intContext.wasInterrupted) {

        obstacles.Clear();
      }
      
      if (!instructionList.empty()) {
        fsmInstruction = instructionList.front();
        instructionList.pop_front();
        instructionValue = fsmInstruction[1];
        state = static_cast<RobotState>(fsmInstruction[0]);
        direction = instructionValue > 0 ? '+' : '-';

      } else {
        state = WAIT;
        instructionValue = instructionCompletedDelay;
      }

      break;
    }

    case MESSAGE_BASE: {
      const char* message = "";
      if (instructionValue == 1) {
        message = "READY";
      }

      SendMessage(robots["Base"], message);
      state = WAIT;
      instructionValue = instructionCompletedDelay;

      break;
    }

    case IDENTIFY_OBSTACLE: {
      currentMillis = millis();
      if (obstacles.robotDetected) {
        state = WAIT;
        instructionValue = instructionCompletedDelay / 2;
        MessageDebugf("DEBUG: -1, ID: %s, Obstáculo encontrado es robot id: %s", 
                     robotID.c_str(), obstacles.fromRobotID.c_str());
      } else if ((currentMillis - movement.previousMillis) >= obstacleWaitTime) {
        movement.previousMillis = currentMillis;
        state = ACTIVE_EVASION;
        instructionValue = 0;
        MessageDebugf("DEBUG: -1, ID: %s, Obstáculo encontrado no es un robot", robotID.c_str());
      }

      break;
    }

    case REQUEST_POSITION: {
      if (robots.find("Base") == robots.end() || robots["Base"] == IPAddress(0,0,0,0)) {
        MessageDebugf("DEBUG: -1, ID: %s, No hay IP de base", robotID.c_str());
        congregation.CompleteRequest();
        
        if (navTarget.isActive) {
          MessageDebugf("DEBUG: -1, ID: %s, Abortando navegación por falta de base", 
                        robotID.c_str());
          navTarget.Reset();
        }
        
        state = WAIT;
        instructionValue = 500;
        break;
      }

      if (!congregation.waitingForResponse) {
        SendMessage(robots["Base"], "REQUEST_POSITION");
        MessageDebugf("DEBUG: -1, ID: %s, Solicitud enviada (iteración %d)", 
                      robotID.c_str(), navTarget.currentIteration);
        congregation.StartRequest();
      }

      if (congregation.HasTimedOut()) {
        MessageDebugf("DEBUG: -1, ID: %s, Timeout en REQUEST_POSITION", robotID.c_str());
        congregation.CompleteRequest();
        
        if (navTarget.isActive && navTarget.currentIteration < navTarget.maxIterations) {
          fsmInstruction[0] = REQUEST_POSITION;
          fsmInstruction[1] = 0;
          instructionList.push_back(fsmInstruction);
          MessageDebugf("DEBUG: -1, ID: %s, Reintentando REQUEST_POSITION", 
                        robotID.c_str());
        } else {
          navTarget.Reset();
        }
        
        state = WAIT;
        instructionValue = 500;
        break;
      }

      if (congregation.positionReceived) {
        MessageDebugf("DEBUG: -1, ID: %s, Respuesta recibida", robotID.c_str());
        congregation.CompleteRequest();
        congregation.positionReceived = false;
        state = READ_INSTRUCTION;
        instructionValue = 0;
      }
      
      break;
    }

    case ACTIVE_EVASION: {
      if (obstacles.obstacleSensors == 0 || !obstacles.HasAnyObstacle()) {
        MessageDebugf("DEBUG: -1, ID: %s, ACTIVE_EVASION sin obstáculo válido. Abortando.", 
                      robotID.c_str());
        obstacles.Clear();
        state = READ_INSTRUCTION;
        break;
      }
      
      unsigned long timeSinceDetection = millis() - evasionStartTime;
      if (timeSinceDetection > 1500) {
        MessageDebugf("DEBUG: -1, ID: %s, Datos de obstáculo obsoletos (%lums). Re-escaneando.", 
                      robotID.c_str(), timeSinceDetection);
        state = STOP; 
        break;
      }
      
      evasionTracker.RecordEvasion();
      
      int avoidanceDistance = 0;
      int avoidanceAngle = 0;
      bool needsRetreat = evasionTracker.ShouldRetreat();
      
      if (needsRetreat) {
        MessageDebugf("DEBUG: -1, ID: %s, Ejecutando retroceso forzado (evasiones: %d)", 
                      robotID.c_str(), evasionTracker.consecutiveEvasions);
        
        std::deque<std::array<float, 2>> retreatSequence;
        
        fsmInstruction[0] = REVERSE;
        fsmInstruction[1] = reverseDistance * 3;
        retreatSequence.push_back(fsmInstruction);
        
        fsmInstruction[0] = TURN;
        fsmInstruction[1] = radians(random(2) ? 180 : -180) * centerToWheelDistance;
        retreatSequence.push_back(fsmInstruction);
        
        fsmInstruction[0] = MOVE;
        fsmInstruction[1] = 300;
        retreatSequence.push_back(fsmInstruction);
        
        for (auto it = retreatSequence.rbegin(); it != retreatSequence.rend(); ++it) {
          instructionList.push_front(*it);
        }
        
        evasionTracker.Reset();
        
      } else {
        if (obstacles.obstacleSensors == 0b100) {  
          avoidanceAngle = 45;
          avoidanceDistance = 200;
        } else if (obstacles.obstacleSensors == 0b001) {  
          avoidanceAngle = -45;
          avoidanceDistance = 200;
        } else if (obstacles.obstacleSensors == 0b010) {  
          avoidanceAngle = (random(2) == 0) ? 60 : -60;
          avoidanceDistance = 250;
        } else if (obstacles.obstacleSensors == 0b110) {  
          avoidanceAngle = 90;
          avoidanceDistance = 150;
        } else if (obstacles.obstacleSensors == 0b011) {  
          avoidanceAngle = -90;
          avoidanceDistance = 150;
        } else if (obstacles.obstacleSensors == 0b111) {  
          avoidanceAngle = (random(2) == 0) ? 135 : -135;
          avoidanceDistance = 100;
        }

        if (!obstacles.HasAnyObstacle()) {
          MessageDebugf("DEBUG: -1, ID: %s, Obstáculo desapareció durante cálculo de evasión",
                        robotID.c_str());
          obstacles.Clear();
          state = READ_INSTRUCTION;
          break;
        }

        std::deque<std::array<float, 2>> evasionSequence;

        if (obstacles.obstacleSensors & 0b010 || obstacles.obstacleSensors == 0b111) {
          fsmInstruction[0] = REVERSE;
          fsmInstruction[1] = reverseDistance * 1.5;
          evasionSequence.push_back(fsmInstruction);
        }

        if (avoidanceAngle != 0) {
          fsmInstruction[0] = TURN;
          fsmInstruction[1] = radians(avoidanceAngle) * centerToWheelDistance;
          evasionSequence.push_back(fsmInstruction);
        }

        if (avoidanceDistance > 0) {
          fsmInstruction[0] = MOVE;
          fsmInstruction[1] = avoidanceDistance;
          evasionSequence.push_back(fsmInstruction);
        }

        for (auto it = evasionSequence.rbegin(); it != evasionSequence.rend(); ++it) {
          instructionList.push_front(*it);
        }
      }
    
      if (intContext.wasInterrupted && !resumeScheduled) {
        fsmInstruction[0] = RESUME_AFTER_EVASION;
        fsmInstruction[1] = 0;
        instructionList.push_back(fsmInstruction);
        resumeScheduled = true;
      }
  
      obstacleDetected = false;
      
      state = READ_INSTRUCTION;
      
      MessageDebugf("DEBUG: -1, ID: %s, Evasión: patrón=%s, giro=%d°, avance=%dmm, forzado=%d", 
                    robotID.c_str(), 
                    obstacles.GetObstaclePattern().c_str(),
                    avoidanceAngle, 
                    avoidanceDistance, 
                    needsRetreat);
      
      break;
    }

    case RESUME_AFTER_EVASION: {
      if (!intContext.wasInterrupted) {
        resumeScheduled = false;
        obstacles.Clear(); 
        state = READ_INSTRUCTION;
        break;
      }
      
      MessageDebugf("DEBUG: -1, ID: %s, Resumiendo: estado=%d, valor=%.1f, navActive=%d", 
                    robotID.c_str(), 
                    intContext.previousState, 
                    intContext.remainingValue,
                    navTarget.isActive);
      
      if (navTarget.isActive) {
        MessageDebugf("DEBUG: -1, ID: %s, Navegación activa: recalculando ruta en lugar de reanudar", 
                      robotID.c_str());
        
        
        fsmInstruction[0] = REQUEST_POSITION;
        fsmInstruction[1] = 0;
        instructionList.push_front(fsmInstruction);
        
      } else {
        switch (intContext.previousState) {
          case MOVE: {
            if (intContext.remainingValue > 20) {
              fsmInstruction[0] = MOVE;
              fsmInstruction[1] = intContext.remainingValue;
              instructionList.push_front(fsmInstruction);
              
              MessageDebugf("DEBUG: -1, ID: %s, Reanudando MOVE: %.1fmm restantes", 
                            robotID.c_str(), intContext.remainingValue);
            }
            break;
          }
            
          case TURN: {
            float angleRemaining = abs((intContext.remainingValue / centerToWheelDistance) * RAD_TO_DEG);
            if (angleRemaining > 5) {
              fsmInstruction[0] = TURN;
              fsmInstruction[1] = intContext.remainingValue;
              instructionList.push_front(fsmInstruction);
              
              MessageDebugf("DEBUG: -1, ID: %s, Reanudando TURN: %.1f° restantes", 
                            robotID.c_str(), angleRemaining);
            }
            break;
          }
        }
      }
      
      intContext.Clear();
      resumeScheduled = false;
      isEvading = false;
      obstacles.Clear();
      
      evasionTracker.Reset();
      
      state = READ_INSTRUCTION;
      break;
    }
  }
}


// ============================================================================
// FUNCIONES DE NAVEGACIÓN ITERATIVA
// ============================================================================

void InitiateIterativeNavigation(float targetX, float targetY) {
    navTarget.targetX = targetX;
    navTarget.targetY = targetY;
    navTarget.StartNavigation(); 
    
    MessageDebugf("DEBUG: -1, ID: %s, Navegación iterativa iniciada: objetivo=(%.1f, %.1f)", 
                  robotID.c_str(), targetX, targetY);
    
    fsmInstruction[0] = REQUEST_POSITION;
    fsmInstruction[1] = 0;
    instructionList.push_back(fsmInstruction);
}

void CalculateIterativeMovement() {
    if (!navTarget.isActive) {
        MessageDebugf("DEBUG: -1, ID: %s, NavigationTarget no está activo", 
                      robotID.c_str());
        return;
    }
    
    if (navTarget.HasExceededMaxIterations()) {
        MessageDebugf("DEBUG: -1, ID: %s, Límite de iteraciones alcanzado (%d). Abortando navegación.", 
                      robotID.c_str(), navTarget.maxIterations);
        navTarget.Reset();
        return;
    }
    
    if (navTarget.HasTimedOut()) {
        MessageDebugf("DEBUG: -1, ID: %s, Timeout de navegación (>5min). Abortando.", 
                      robotID.c_str());
        navTarget.Reset();
        return;
    }
    
    navTarget.currentIteration++;
    
    float deltaX = navTarget.targetX - robotPose.x;
    float deltaY = navTarget.targetY - robotPose.y;
    float totalDistance = sqrt(deltaX * deltaX + deltaY * deltaY);
    
    MessageDebugf("DEBUG: -1, ID: %s, Iteración %d: pos=(%.1f,%.1f), target=(%.1f,%.1f), dist=%.1fmm", 
                  robotID.c_str(), navTarget.currentIteration,
                  robotPose.x, robotPose.y, 
                  navTarget.targetX, navTarget.targetY, totalDistance);
    
    if (navTarget.IsInLoop(robotPose.x, robotPose.y)) {
        MessageDebugf("DEBUG: -1, ID: %s, LOOP DETECTADO. Abortando navegación.", 
                      robotID.c_str());
        navTarget.Reset();
        return;
    }
    
    if (!navTarget.IsMakingProgress(totalDistance)) {
        MessageDebugf("DEBUG: -1, ID: %s, Sin progreso en %d iteraciones. Abortando.", 
                      robotID.c_str(), navTarget.maxIterationsWithoutProgress);
        navTarget.Reset();
        return;
    }
    
    navTarget.RecordPosition(robotPose.x, robotPose.y);
    
    if (totalDistance < navTarget.arrivalThreshold) {
        MessageDebugf("DEBUG: -1, ID: %s, Objetivo alcanzado. Distancia final: %.1fmm (iteraciones: %d)", 
                      robotID.c_str(), totalDistance, navTarget.currentIteration);
        navTarget.Reset();
        return;
    }
    
    float targetAngle = atan2(deltaY, deltaX) * RAD_TO_DEG;
    float angleDiff = NormalizeAngle(targetAngle - robotPose.angle);
    
    float segmentDistance;
    
    if (totalDistance <= navTarget.segmentDistance) {
        segmentDistance = totalDistance * 0.9;
    } else {
        segmentDistance = navTarget.segmentDistance;
    }
    
    segmentDistance = constrain(segmentDistance, 
                                navTarget.minSegmentDistance, 
                                navTarget.maxSegmentDistance);
    
    MessageDebugf("DEBUG: -1, ID: %s, Segmento: dist=%.1fmm, ángulo=%.1f°, progreso=%d/%d", 
                  robotID.c_str(), 
                  segmentDistance, 
                  angleDiff,
                  navTarget.maxIterationsWithoutProgress - navTarget.iterationsWithoutProgress,
                  navTarget.maxIterationsWithoutProgress);
    
    if (abs(angleDiff) > 5) {
        fsmInstruction[0] = TURN;
        fsmInstruction[1] = radians(angleDiff) * centerToWheelDistance;
        instructionList.push_back(fsmInstruction);
    }
    
    if (segmentDistance > navTarget.minSegmentDistance) {
        fsmInstruction[0] = MOVE;
        fsmInstruction[1] = segmentDistance;
        instructionList.push_back(fsmInstruction);
    }
    
    fsmInstruction[0] = REQUEST_POSITION;
    fsmInstruction[1] = 0;
    instructionList.push_back(fsmInstruction);
    
    MessageDebugf("DEBUG: -1, ID: %s, Programando REQUEST_POSITION para siguiente iteración", 
                  robotID.c_str());
}


// ============================================================================
// FUNCIONES DE SENSORES Y HARDWARE
// ============================================================================


void SetupFrontSensor() {
  if (frontSensorInitialized) return;

  unsigned long now = millis();
  
  // CORRECCIÓN CLAVE: Permite la ejecución si es el primer intento (lastFrontSensorAttempt == 0),
  // o si han pasado 5 segundos desde el último intento fallido.
  if (lastFrontSensorAttempt != 0 && (now - lastFrontSensorAttempt < frontSensorRetryInterval)) {
      return;
  }
  
  lastFrontSensorAttempt = now;
  
  DebugSerialPrintln("Intentando inicializar APDS9960...");

    
  
  if (frontSensor.begin()) {
    frontSensorInitialized = true;
    ledCtrl.setOff();
    DebugSerialPrintln(" Sensor APDS-9960 inicializado correctamente");
  } else {
    DebugSerialPrintln(" Falló la inicialización del sensor APDS-9960. Reintentando...");
    ledCtrl.setBlink(255, 128, 0, maxBrightness, 500);
  }
}

void WiFiStatus() {
  static uint8_t lastStatus = 255;  // Inicializar con valor inválido
  uint8_t currentStatus = WiFi.status();

  // Debug: mostrar cambios de estado
  if (currentStatus != lastStatus) {
    const char* statusStr[] = {
      "WL_IDLE_STATUS",      // 0
      "WL_NO_SSID_AVAIL",    // 1
      "WL_SCAN_COMPLETED",   // 2
      "WL_CONNECTED",        // 3
      "WL_CONNECT_FAILED",   // 4
      "WL_CONNECTION_LOST",  // 5
      "WL_DISCONNECTED"      // 6
    };
    if (currentStatus <= 6) {
      DebugSerialPrintf("WiFi Status cambió: %s (%d)\n", statusStr[currentStatus], currentStatus);
    }
    lastStatus = currentStatus;
  }

  if (WiFi.status() != WL_CONNECTED) {
    static bool wifiConnecting = false;
    static unsigned long lastWifiAttempt = 0;
    static int retryCount = 0;

    if (!wifiConnecting) {
      ConfigureHBridge(0, 0);
      DebugSerialPrintln("=== Iniciando conexión WiFi ===");
      DebugSerialPrintf("SSID: %s\n", ssid);
      DebugSerialPrintf("MAC: %s\n", WiFi.macAddress().c_str());
      DebugSerialPrintf("Hostname: %s\n", WiFi.getHostname());
      ledCtrl.setBlink(0, 255, 255, maxBrightness, 250);
      wifiConnecting = true;
      lastWifiAttempt = millis();
      retryCount = 0;
    }

    // Timeout de conexión: reintentar después de 10 segundos
    if (millis() - lastWifiAttempt > 10000) {
      retryCount++;
      DebugSerialPrintf("⚠ Timeout WiFi (intento #%d). Estado: %d\n", retryCount, WiFi.status());

      // Después de 3 intentos, hacer un reset más agresivo
      if (retryCount >= 3) {
        DebugSerialPrintln("🔴 Múltiples fallos. Reiniciando WiFi completamente...");
        WiFi.mode(WIFI_OFF);
        delay(500);
        WiFi.mode(WIFI_STA);
        String hostname = "AttaBot-" + String((uint32_t)ESP.getEfuseMac(), HEX);
        WiFi.setHostname(hostname.c_str());
        retryCount = 0;
      }

      WiFi.disconnect();
      delay(100);
      WiFi.begin(ssid, password);
      lastWifiAttempt = millis();
    }

    return;
  } else {
    static bool firstConnect = true;
    if (firstConnect) {
      DebugSerialPrintln("=== ✓ WiFi CONECTADO ===");
      DebugSerialPrintf("IP: %s\n", WiFi.localIP().toString().c_str());
      DebugSerialPrintf("Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
      DebugSerialPrintf("Subnet: %s\n", WiFi.subnetMask().toString().c_str());
      DebugSerialPrintf("DNS: %s\n", WiFi.dnsIP().toString().c_str());
      DebugSerialPrintf("MAC: %s\n", WiFi.macAddress().c_str());
      DebugSerialPrintf("Hostname: %s\n", WiFi.getHostname());
      DebugSerialPrintf("RSSI: %d dBm\n", WiFi.RSSI());
      DebugSerialPrintln("=======================");
      firstConnect = false;
    }
    ledCtrl.setOff();
  }
}

void ReadSensors() {
  if (((millis() - movement.previousMillis) <= samplingTime - 2) || isLateralCycleActive || isCentralCycleActive || (debugUdp == 3)) {
    currentMicros = micros();
    if (currentMicros - previousMicros >= observationTime) {
      previousMicros = currentMicros;
      cycleCounter = (cycleCounter + 1) % numberOfCycles;

      isLateralCycleActive = (lateralCycle == cycleCounter);

      if (isLateralCycleActive != lateralSensorsEnabled) {
        lateralSensorsEnabled = isLateralCycleActive;
        digitalWrite(enableLeftInfraredSensor, lateralSensorsEnabled);
        digitalWrite(enableRightInfraredSensor, lateralSensorsEnabled);

        if (lateralSensorsEnabled) {
          noInterrupts();
          leftObsStartTime = micros();
          rightObsStartTime = micros();
          interrupts();
        }
      }

      isCentralCycleActive = (centralCycle == cycleCounter);
      
      // GUARDIA 1: Solo intenta habilitar la proximidad si el sensor ya está inicializado.
      if (frontSensorInitialized) { 
          frontSensor.enableProximity(isCentralCycleActive);
      }
    }

    if (isLateralCycleActive) {
      noInterrupts();
      unsigned long leftTime = leftObsStartTime;
      unsigned long rightTime = rightObsStartTime;
      interrupts();

      unsigned long now = micros();
      obstacles.leftObstacle = (digitalRead(leftInfraredSensor) == LOW) && ((now - leftTime) >= minObstacleTime);
      obstacles.rightObstacle = (digitalRead(rightInfraredSensor) == LOW) && ((now - rightTime) >= minObstacleTime);
    }

    if (isCentralCycleActive) {
      // GUARDIA 2: Solo intenta leer la proximidad si el sensor ya está inicializado.
      if (frontSensorInitialized) { 
          centralDistance = frontSensor.readProximity();
          if (centralDistance > 2) {
            obstacles.centralObstacle = (micros() - centralObsStartTime) >= minObstacleTime / 2;
          } else {
            centralObsStartTime = micros();
            obstacles.centralObstacle = false;
          }
      } else {
          // Si no está inicializado, asumimos que no hay obstáculo
          obstacles.centralObstacle = false; 
      }
    }
  }

  if (debugUdp == 3) {
    if (obstacles.HasAnyObstacle()) {
      ledCtrl.setSolid(255, 128, 0, maxBrightness);
    } else {
      ledCtrl.setOff();
    }
  } else {
    if ((digitalRead(batteryStatus) == LOW) && ((millis() - lowBatteryTime) >= minLowBatteryTime)) {
      ledCtrl.setSolid(255, 255, 0, 255);
    }
  }

  if (isEvading && (millis() - evasionStartTime > evasionCooldown)) {
    isEvading = false;
    MessageDebugf("DEBUG: -1, ID: %s, Cooldown de evasión completado", 
                  robotID.c_str());
  }
}


// ============================================================================
// FUNCIONES DE CONTROL DE MOTORES
// ============================================================================

void ResetPID() {
  leftControl.Reset();
  rightControl.Reset();
  debugCounter = 0;
  movement.Reset();
}

void ConfigureHBridge(int leftWheelPWM, int rightWheelPWM) {
  if (leftWheelPWM >= 0) {
    ledcWrite(leftMotorBackward, 0);
    ledcWrite(leftMotorForward, leftWheelPWM);
  } else {
    ledcWrite(leftMotorForward, 0);
    ledcWrite(leftMotorBackward, abs(leftWheelPWM));
  }

  if (rightWheelPWM >= 0) {
    ledcWrite(rightMotorBackward, 0);
    ledcWrite(rightMotorForward, rightWheelPWM);
  } else {
    ledcWrite(rightMotorForward, 0);
    ledcWrite(rightMotorBackward, abs(rightWheelPWM));
  }
}

float DesiredSpeed(float distance, float wheelDistance) {
  float remainingDistance = distance - wheelDistance;
  float desiredSpeed = maxSpeed;
  if (abs(remainingDistance) < speedReductionThreshold) {
    desiredSpeed = map(abs(remainingDistance), 0, speedReductionThreshold, minSpeed, maxSpeed);
  }

  return (remainingDistance < 0) ? -desiredSpeed : desiredSpeed;
}

bool IsStationary(float currentLeftSpeed, float currentRightSpeed, float leftWheelDistance, float rightWheelDistance) {
  bool speedsAtZero = (static_cast<int>(abs(currentLeftSpeed) + abs(currentRightSpeed)) == 0);
  bool wheelsHaveMoved = (static_cast<int>(abs(leftWheelDistance) + abs(rightWheelDistance)) != 0);
  if (!speedsAtZero) {
    movement.steadyStatePreviousMillis = millis();
  } else if ((millis() - movement.steadyStatePreviousMillis >= SteadyStateTime) && wheelsHaveMoved) {
    return true;
  }

  return false;
}

bool MoveDistanceByWheel(float leftDistance, float rightDistance) {
  currentMillis = millis();
  millisDifference = currentMillis - movement.previousMillis;
  if (millisDifference < samplingTime) {
    return false;
  }

  movement.previousMillis = currentMillis;

  movement.currentLeftSpeed = ((movement.leftPulseCount - movement.pastLeftPulseCount) * millimetersPerPulse) / samplingTimeS;
  movement.pastLeftPulseCount = movement.leftPulseCount;
  movement.currentRightSpeed = ((movement.rightPulseCount - movement.pastRightPulseCount) * millimetersPerPulse) / samplingTimeS;
  movement.pastRightPulseCount = movement.rightPulseCount;

  float leftWheelDistance = movement.pastLeftPulseCount * millimetersPerPulse;
  float desiredLeftSpeed = DesiredSpeed(leftDistance, leftWheelDistance);
  int leftWheelPWM = leftControl.Calculate(desiredLeftSpeed, movement.currentLeftSpeed);

  float rightWheelDistance = movement.pastRightPulseCount * millimetersPerPulse;
  float desiredRightSpeed = DesiredSpeed(rightDistance, rightWheelDistance);
  int rightWheelPWM = rightControl.Calculate(desiredRightSpeed, movement.currentRightSpeed);

  ConfigureHBridge(leftWheelPWM, rightWheelPWM);
  bool IsMoveFinished = ((abs(leftWheelDistance) + distanceOffset) >= abs(leftDistance)) &&
                        ((abs(rightWheelDistance) + distanceOffset) >= abs(rightDistance));
  IsMoveFinished = IsMoveFinished || IsStationary(movement.currentLeftSpeed, movement.currentRightSpeed, 
                                                   leftWheelDistance, rightWheelDistance);
  
  if (debugUdp >= 2) {
    MessageDebugf(debugMessage, debugCounter, robotID.c_str(), direction, 
                  movement.leftPulseCount, movement.rightPulseCount, 
                  movement.currentLeftSpeed, movement.currentRightSpeed, 
                  leftWheelPWM, rightWheelPWM,
                  leftControl.error, rightControl.error, 
                  leftControl.sumError, rightControl.sumError, 
                  leftWheelDistance, rightWheelDistance, millisDifference);
  }

  return IsMoveFinished;
}


// ============================================================================
// FUNCIONES DE COMUNICACIÓN
// ============================================================================

void SendMessage(IPAddress host, const char* message) {
  udp.beginPacket(host, localPort);
  udp.write(reinterpret_cast<const uint8_t*>(message), strlen(message));
  udp.endPacket();
}

void MessageDebugf(const char* format, ...) {
  char buffer[200];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  DebugSerialPrintln(buffer);
  if (debugUdp != 0) {
    SendMessage(robots["Base"], buffer);
  }

  debugCounter++;
}

void SendPose() {
  obstacles.robotDetected = false;
  const char* message = "CHECK_OBSTACLE|%d|%.1f|%.1f|%.1f";
  char buffer[40];
  snprintf(buffer, sizeof(buffer), message, obstacles.obstacleSensors, 
           robotPose.x, robotPose.y, robotPose.angle);
  SendMessage(robots["Broadcast"], buffer);
  movement.previousMillis = millis();
}

std::array<String, 5> SeparateCommand(const String& command, char delimiter) {
  std::array<String, 5> results;
  int startIndex = 0;
  int endIndex;
  int count = 0;

  while (count < results.size()) {
    endIndex = command.indexOf(delimiter, startIndex);
    if (endIndex == -1) {
      results[count] = command.substring(startIndex);
      break;
    } else {
      results[count] = command.substring(startIndex, endIndex);
      startIndex = endIndex + 1;
    }
    count++;
  }

  return results;
}

bool IsRobotObstacle(float x2, float y2, float angle, int sensors, String id) {
  float deltaX = x2 - robotPose.x;
  float deltaY = y2 - robotPose.y;

  float distanceBetweenRobots = sqrt(deltaX * deltaX + deltaY * deltaY);
  if (distanceBetweenRobots > robotDistanceMargin) {
    return false;
  }

  float angleBetweenRobots = atan2f(deltaY, deltaX) * RAD_TO_DEG + 180;
  float angleDifference = angleBetweenRobots - angle;

  if (angleDifference > 180) {
    angleDifference -= 360;
  } else if (angleDifference < -180) {
    angleDifference += 360;
  }
  
  MessageDebugf("DEBUG: -1, ID: %s, From ID: %s, Distancia: %.1f, Angulo: %.1f, DifAngulo: %.1f, sensors: %d", 
                robotID.c_str(), id.c_str(), distanceBetweenRobots, angleBetweenRobots, angleDifference, sensors);

  if (abs(angleDifference) <= maxRobotAngleMargin) {
    if (sensors == 0b100 && angleDifference <= 0) {
      return true;
    } else if (sensors == 0b001 && angleDifference >= 0) {
      return true;
    } else if (sensors != 0b100 && sensors != 0b001) {
      return true;
    }
  }

  return false;
}


// ============================================================================
// FUNCIÓN DE LECTURA DE PAQUETES UDP (REFACTORIZADA)
// ============================================================================

void ReadUdpPackets() {
  int packetBytes = udp.parsePacket();
  if (!packetBytes) {
    return;
  }

  int len = udp.read(receivedPacket, sizeof(receivedPacket) - 1);
  if (len > 0) {
    receivedPacket[len] = 0;
  }
  DebugSerialPrintf("Recibidos %d bytes de %s: %s\n", packetBytes, 
                    udp.remoteIP().toString().c_str(), receivedPacket);

  String command(receivedPacket);
  std::array<String, 5> arguments = SeparateCommand(command, '|');
  command = arguments[0];

  // CONFIG
  if (command == "CONFIG") {
    if (arguments[1] == "START") {
      robots["Base"] = udp.remoteIP();
      IPAddress ipAddress;
      ipAddress.fromString(arguments[2]);
      robots["Broadcast"] = ipAddress;
      SendMessage(robots["Base"], "CONFIG|RECEIVED");
      debugUdp = 0;
      countMessages = 0;
      sendMessages = 0;

    } else if (arguments[1] == "SAVE") {
      robots[arguments[2]] = udp.remoteIP();

    } else if (arguments[1] == "ROBOTS") {
      for (const auto& pair : robots) {
        DebugSerialPrintf("Nombre: %s, IP: %s\n", pair.first.c_str(), pair.second.toString().c_str());
      }

    } else if (arguments[1] == "DEBUG") {
      debugUdp = arguments[2].toInt();
      SendMessage(robots["Base"], debugUdp != 0 ? "Modo debug activado" : "Modo debug desactivado");

    } else {
      robotID = arguments[1];
      char buffer[50];
      snprintf(buffer, sizeof(buffer), "CONFIG|SAVE|%s", robotID.c_str());
      SendMessage(robots["Broadcast"], buffer);
    }
  }
  
  // INSTRUCCIONES DE MOVIMIENTO
  else if (command == "MOVE" || command == "TURN" || command == "WAIT" || 
           command == "RANDOMW" || command == "MESSAGE_BASE") {
    short value = arguments[1].toInt();

    if (command == "MOVE") {
      fsmInstruction[0] = MOVE;
      fsmInstruction[1] = value;
    } else if (command == "TURN") {
      fsmInstruction[0] = TURN;
      fsmInstruction[1] = radians(value) * centerToWheelDistance;
    } else if (command == "WAIT") {
      fsmInstruction[0] = WAIT;
      fsmInstruction[1] = value * 1000;
    } else if (command == "RANDOMW") {
      fsmInstruction[0] = RANDOM_WALK;
      fsmInstruction[1] = value * 1000;
    } else if (command == "MESSAGE_BASE") {
      fsmInstruction[0] = MESSAGE_BASE;
      fsmInstruction[1] = arguments[1].toInt();
    }
    instructionList.push_back(fsmInstruction);
  }
  
  // RESET
  else if (command == "RESET") {
    ESP.restart();
  }
  
  // SERVO
  else if (command == "SERVO") {
    int servoAngle = constrain(arguments[1].toInt(), 5, 175);
    DebugSerialPrintf("Servo: %d°\n", servoAngle);
    frontServo.write(servoAngle);
  }
  
  // PID
  else if (command == "PID") {
    float kp = arguments[1].toFloat();
    float ki = arguments[2].toFloat();
    float kd = arguments[3].toFloat();

    leftControl.pidConst.kp = kp;
    leftControl.pidConst.ki = ki;
    leftControl.pidConst.kd = kd;
    rightControl.pidConst.kp = kp;
    rightControl.pidConst.ki = ki;
    rightControl.pidConst.kd = kd;
    SendMessage(robots["Base"], "PID velocidad modificado");
  }
  
  // KFPID
  else if (command == "KFPID") {
    float R = arguments[1].toFloat();
    float H = arguments[2].toFloat();
    float Q = arguments[3].toFloat();

    leftControl.kf.R = R;
    leftControl.kf.H = H;
    leftControl.kf.Q = Q;
    rightControl.kf.R = R;
    rightControl.kf.H = H;
    rightControl.kf.Q = Q;
    SendMessage(robots["Base"], "Filtro de kalman modificado");
  }
  
  // POSE
  else if (command == "POSE") {
    float newX = arguments[1].toFloat();
    float newY = arguments[2].toFloat();
    float newAngle = arguments[3].toFloat();

    if (robotPose.x != 0 || robotPose.y != 0) {  
      float deltaX = newX - robotPose.x;
      float deltaY = newY - robotPose.y;
      float distanceJump = sqrt(deltaX * deltaX + deltaY * deltaY);

      float angleDiff = NormalizeAngle(newAngle - robotPose.angle);

      if (distanceJump > max_pose_jump || abs(angleDiff) > max_angle_jump) {
        char buffer[150];
        snprintf(buffer, sizeof(buffer),
                "WARNING: Salto brusco detectado. ΔPos=%.1fmm, ΔAng=%.1f°. Ignorando actualización.",
                distanceJump, angleDiff);
        MessageDebugf("DEBUG: -1, ID: %s, %s", robotID.c_str(), buffer);
        return;
      }
    }

    robotPose.x = newX;
    robotPose.y = newY;
    robotPose.angle = newAngle;
  }
  
  // SETPPR
  else if (command == "SETPPR") {
    float newPPR = arguments[1].toFloat();
    bool permanent = (arguments[2] == "SAVE");
    
    if (newPPR > 100 && newPPR < 5000) {
      pulsesPerRev = newPPR;
      updateMillimetersPerPulse();
      
      char buffer[100];
      if (permanent) {
        SavePPR(newPPR);
        snprintf(buffer, sizeof(buffer), 
                "PPR modificado y GUARDADO: %.2f (Robot ID: %s)", 
                newPPR, robotID.c_str());
      } else {
        snprintf(buffer, sizeof(buffer), 
                "PPR modificado temporalmente: %.2f (Robot ID: %s)", 
                newPPR, robotID.c_str());
      }
      SendMessage(robots["Base"], buffer);
      
    } else {
      SendMessage(robots["Base"], "Error: PPR debe estar entre 100-5000");
    }
  }
  
  // GETPPR
  else if (command == "GETPPR") {
    char buffer[100];
    snprintf(buffer, sizeof(buffer), 
            "Robot %s - PPR actual: %.2f, Chip ID: %04X%08X", 
            robotID.c_str(), 
            pulsesPerRev,
            (uint16_t)(ESP.getEfuseMac()>>32),
            (uint32_t)ESP.getEfuseMac());
    SendMessage(robots["Base"], buffer);
  }
  
  // CHECK_OBSTACLE
  else if (command == "CHECK_OBSTACLE") {
    int sensors = arguments[1].toInt();
    float x = arguments[2].toFloat();
    float y = arguments[3].toFloat();
    float angle = arguments[4].toFloat();

    String id = "-1";
    for (const auto& pair : robots) {
      if (pair.second.toString() == udp.remoteIP().toString()) {
        id = pair.first;
        break;
      }
    }

    if (IsRobotObstacle(x, y, angle, sensors, id)) {
      char buffer[50];
      snprintf(buffer, sizeof(buffer), "OBSTACLE_DETECTED|%s", robotID.c_str());
      delayMicroseconds(800);
      SendMessage(robots[id], buffer);
      SendMessage(robots[id], buffer);
    }
  }
  
  // OBSTACLE_DETECTED
  else if (command == "OBSTACLE_DETECTED") {
    obstacles.fromRobotID = arguments[1];
    obstacles.robotDetected = true;
  }
  
  // COUNT_MESSAGE
  else if (command == "COUNT_MESSAGE") {
    countMessages++;
  }
  
  // SEND_COUNT_MESSAGE
  else if (command == "SEND_COUNT_MESSAGE") {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Robot ID: %s, Total messages: %d", 
             robotID.c_str(), countMessages);
    SendMessage(robots["Base"], buffer);
  }
  
  // CONGREGATION
  else if (command == "CONGREGATION") {
    congregation.leaderID = arguments[1];
    congregation.isLeader = (congregation.leaderID == robotID);
    congregation.positionReceived = false;
    congregation.hasGlobalTarget = false;

    navTarget.Reset();

    MessageDebugf("DEBUG: -1, ID: %s, Congregación iniciada. Líder: %s", 
                  robotID.c_str(), congregation.leaderID.c_str());

    int delay = robotID.toInt() * 200;
    fsmInstruction[0] = WAIT;
    fsmInstruction[1] = delay;
    instructionList.push_back(fsmInstruction);

    fsmInstruction[0] = REQUEST_POSITION;
    fsmInstruction[1] = 0;
    instructionList.push_back(fsmInstruction);
  }
  
  // POSITIONGT
  else if (command == "POSITIONGT") {
    float targetX = arguments[1].toFloat();
    float targetY = arguments[2].toFloat();

    if (abs(targetX) > max_workspace_x || abs(targetY) > max_workspace_y) {
      char buffer[100];
      snprintf(buffer, sizeof(buffer),
              "ERROR: Objetivo fuera de rango. X=%.1f (max=%.1f), Y=%.1f (max=%.1f)",
              targetX, max_workspace_x, targetY, max_workspace_y);
      SendMessage(robots["Base"], buffer);
      MessageDebugf("DEBUG: -1, ID: %s, %s", robotID.c_str(), buffer);
      return;
    }

    if (arguments[3] != "") {
      float customSegmentDist = arguments[3].toFloat();
      if (customSegmentDist >= 50 && customSegmentDist <= 400) {
        navTarget.segmentDistance = customSegmentDist;
      }
    }

    MessageDebugf("DEBUG: -1, ID: %s, Objetivo global recibido: x=%.1f, y=%.1f, segDist=%.1fmm",
                  robotID.c_str(), targetX, targetY, navTarget.segmentDistance);

    InitiateIterativeNavigation(targetX, targetY);
  }
  
  // POSITION_RESPONSE
  else if (command == "POSITION_RESPONSE") {
    robotPose.x = arguments[1].toFloat();
    robotPose.y = arguments[2].toFloat();
    robotPose.angle = arguments[3].toFloat();

    MessageDebugf("DEBUG: -1, ID: %s, Posición recibida: x=%.1f, y=%.1f, ángulo=%.1f", 
                  robotID.c_str(), robotPose.x, robotPose.y, robotPose.angle);

    if (congregation.isLeader && congregation.leaderID != "-1") {
      char buffer[64];
      snprintf(buffer, sizeof(buffer), "LEADER_POSITION|%s|%.1f|%.1f|%.1f", 
              robotID.c_str(), robotPose.x, robotPose.y, robotPose.angle);
      
      if (robots.find("Broadcast") != robots.end()) {
        SendMessage(robots["Broadcast"], buffer);
        delayMicroseconds(500);
        SendMessage(robots["Broadcast"], buffer);
      }
    }
    
    if (navTarget.isActive) {
      CalculateIterativeMovement();
    }
    else if (congregation.hasGlobalTarget) {
      InitiateIterativeNavigation(congregation.globalTargetX, congregation.globalTargetY);
      congregation.hasGlobalTarget = false;
      congregation.globalTargetX = 0;
      congregation.globalTargetY = 0;
    }
    
    congregation.positionReceived = true;
  }
  
  // LEADER_POSITION
  else if (command == "LEADER_POSITION") {
    String receivedLeaderID = arguments[1];

    if (!congregation.isLeader && receivedLeaderID == congregation.leaderID) {
      float leaderX = arguments[2].toFloat();
      float leaderY = arguments[3].toFloat();
      
      if (congregation.positionReceived) {
        MessageDebugf("DEBUG: -1, ID: %s, Procesando posición del líder", 
                      robotID.c_str());
        InitiateIterativeNavigation(leaderX, leaderY);
      } else {
        congregation.globalTargetX = leaderX;
        congregation.globalTargetY = leaderY;
        congregation.hasGlobalTarget = true;
        MessageDebugf("DEBUG: -1, ID: %s, Guardando posición del líder para después", 
                      robotID.c_str());
      }
    }
  }
  
  // CANCEL_CONGREGATION
  else if (command == "CANCEL_CONGREGATION") {
    congregation.Reset();
    navTarget.Reset();
    instructionList.clear();
    state = STOP;
    MessageDebugf("DEBUG: -1, ID: %s, Congregación cancelada", robotID.c_str());
  }
  
  // CLEAR_EVASION
  else if (command == "CLEAR_EVASION") {
    intContext.Clear();
    isEvading = false;
    resumeScheduled = false;
    obstacles.Clear();
    instructionList.clear();
    state = STOP;
    MessageDebugf("DEBUG: -1, ID: %s, Sistema de evasión reseteado", 
                  robotID.c_str());
  }
  
  // NAV_CONFIG
  else if (command == "NAV_CONFIG") {
    if (arguments[1] == "SEGMENT_DIST") {
      float newDist = arguments[2].toFloat();
      if (newDist >= 50 && newDist <= 400) {
        navTarget.segmentDistance = newDist;
        MessageDebugf("DEBUG: -1, ID: %s, Distancia de segmento: %.1fmm", 
                      robotID.c_str(), newDist);
      }
    } else if (arguments[1] == "ARRIVAL_THRESHOLD") {
      float newThreshold = arguments[2].toFloat();
      if (newThreshold >= 30 && newThreshold <= 150) {
        navTarget.arrivalThreshold = newThreshold;
        MessageDebugf("DEBUG: -1, ID: %s, Umbral de llegada: %.1fmm", 
                      robotID.c_str(), newThreshold);
      }
    } else if (arguments[1] == "MAX_ITER") {
      int newMax = arguments[2].toInt();
      if (newMax >= 10 && newMax <= 100) {
        navTarget.maxIterations = newMax;
        MessageDebugf("DEBUG: -1, ID: %s, Máximo de iteraciones: %d", 
                      robotID.c_str(), newMax);
      }
    }
  }

  //GET_STATUS
  else if (command == "GET_STATUS") {
    char buffer[200];
    snprintf(buffer, sizeof(buffer), 
            "STATUS|ID:%s|State:%d|NavActive:%d|Evading:%d|Obstacles:%d|Sensors:L%d-C%d-R%d|Pos:(%.1f,%.1f,%.1f)|Target:(%.1f,%.1f)|Iter:%d/%d",
            robotID.c_str(),
            state,
            navTarget.isActive,
            isEvading,
            obstacles.HasAnyObstacle(),
            obstacles.leftObstacle,
            obstacles.centralObstacle,
            obstacles.rightObstacle,
            robotPose.x, robotPose.y, robotPose.angle,
            navTarget.targetX, navTarget.targetY,
            navTarget.currentIteration, navTarget.maxIterations);
    SendMessage(robots["Base"], buffer);
  }

  else if (command == "RESET_EVASION") {
    evasionTracker.Reset();
    intContext.Clear();
    isEvading = false;
    resumeScheduled = false;
    obstacles.Clear();
    MessageDebugf("DEBUG: -1, ID: %s, ✅ Sistema de evasión reseteado manualmente", 
                  robotID.c_str());
  }

  else if (command == "ABORT_NAV") {
    navTarget.Reset();
    instructionList.clear();
    state = STOP;
    MessageDebugf("DEBUG: -1, ID: %s, ✅ Navegación abortada manualmente", 
                  robotID.c_str());
  }
}


// ============================================================================
// FUNCIONES AUXILIARES
// ============================================================================

void SelectMovementRW() {
  int probabilityTurnPos = 15;
  int probabilityMove = 70 * (obstacleDetected ? 0 : 1);
  int probabilityTurnNeg = 15;
  int totalProbabilities = probabilityTurnPos + probabilityMove + probabilityTurnNeg;
  std::array<int, 3> cumulativeProbabilities = { probabilityTurnPos, probabilityTurnPos + probabilityMove, totalProbabilities };
  obstacleDetected = false;

  int randomSelection = random(totalProbabilities);
  int directionRW;
  if (randomSelection < cumulativeProbabilities[0]) {
    directionRW = TURN_POS;
  } else if (randomSelection < cumulativeProbabilities[1]) {
    directionRW = MOVE_FORWARD;
  } else {
    directionRW = TURN_NEG;
  }

  int angle = possibleAngles[random(possibleAngles.size() * 10) % possibleAngles.size()];
  int distance = possibleAdvances[random(possibleAdvances.size() * 10) % possibleAdvances.size()];
  switch (directionRW) {
    case TURN_POS: {
      fsmInstruction[0] = TURN;
      fsmInstruction[1] = radians(angle) * centerToWheelDistance;
      break;
    }

    case MOVE_FORWARD: {
      fsmInstruction[0] = MOVE;
      fsmInstruction[1] = distance;
      break;
    }

    case TURN_NEG: {
      fsmInstruction[0] = TURN;
      fsmInstruction[1] = -radians(angle) * centerToWheelDistance;
      break;
    }
  }

  instructionList.push_front(fsmInstruction);
}

void CommunicationTest() {
  if (sendMessages < 500) {
    sendMessages++;
    for (const auto& pair : robots) {
      if (pair.first != "Broadcast" && pair.first != "Base") {
        SendMessage(pair.second, "COUNT_MESSAGE");
      }
    }
  } else {
    ledCtrl.setSolid(255, 0, 0, 255);
  }
}

#ifdef DebugSerial
void ReadSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    int separatorIndex = command.indexOf('|');
    String cmd = command.substring(0, separatorIndex);
    String valueStr = command.substring(separatorIndex + 1);
    int value = valueStr.toInt();
    
    if (cmd == "MOVE") {
      fsmInstruction[0] = MOVE;
      fsmInstruction[1] = value;
      instructionList.push_back(fsmInstruction);
      Serial.printf("Comando MOVE %d mm agregado\n", value);
      
    } else if (cmd == "TURN") {
      fsmInstruction[0] = TURN;
      fsmInstruction[1] = radians(value) * centerToWheelDistance;
      instructionList.push_back(fsmInstruction);
      Serial.printf("Comando TURN %d grados agregado\n", value);
      
    } else if (cmd == "STOP") {
      instructionList.clear();
      ConfigureHBridge(0, 0);
      state = STOP;
      Serial.println("Robot detenido");
      
    } else {
      Serial.println("Comandos: MOVE|valor, TURN|valor, STOP");
    }
  }
}
#endif


// ============================================================================
// FUNCIONES DE LED
// ============================================================================

void LedController::update() {
  unsigned long now = millis();
  switch (currentState) {
    case OFF:
      if (brightness != 0) {
        brightness = 0;
        FastLED.setBrightness(0);
        FastLED.show();
      }
      break;
      
    case SOLID:
      if (now - lastUpdate > 50) {
        leds[0] = CRGB(red, green, blue);
        FastLED.setBrightness(brightness);
        FastLED.show();
        lastUpdate = now;
      }
      break;
      
    case BLINKING:
      if (now - lastUpdate >= interval) {
        blinkState = !blinkState;
        if (blinkState) {
          leds[0] = CRGB(red, green, blue);
          FastLED.setBrightness(brightness);
        } else {
          FastLED.setBrightness(0);
        }
        FastLED.show();
        lastUpdate = now;
      }
      break;
  }
}

void setLedColor(uint8_t red, uint8_t green, uint8_t blue) {
  ledCtrl.setSolid(red, green, blue, maxBrightness);
}

void setLedBrightness(uint8_t brightness) {
  ledCtrl.brightness = brightness;
  if (brightness == 0) ledCtrl.setOff();
  else ledCtrl.currentState = LedController::SOLID;
}

void setLedBlink(uint8_t red, uint8_t green, uint8_t blue, unsigned long intervalMs) {
  ledCtrl.setBlink(red, green, blue, maxBrightness, intervalMs);
}


// ============================================================================
// FUNCIONES IMU (OPCIONAL)
// ============================================================================

bool StoreValido(biasStore* store) {
  int32_t suma = store->header;

  if (suma != 0x42) return false;

  suma += store->biasGyroX;
  suma += store->biasGyroY;
  suma += store->biasGyroZ;
  suma += store->biasAccelX;
  suma += store->biasAccelY;
  suma += store->biasAccelZ;
  suma += store->biasCPassX;
  suma += store->biasCPassY;
  suma += store->biasCPassZ;

  return (store->sum == suma);
}

void setupIMU() {
  DebugSerialPrintln("Inicializando IMU ICM-20948...");
  
  bool imuDetected = false;
  int attempts = 0;
  const int maxAttempts = 3;
  
  while (!imuDetected && attempts < maxAttempts) {
    attempts++;
    DebugSerialPrintf("Intento %d/%d de conexión con IMU...\n", attempts, maxAttempts);
    
    imu.begin(Wire, AD0_VAL);
    
    if (imu.status == ICM_20948_Stat_Ok) {
      imuDetected = true;
      DebugSerialPrintln("IMU detectada correctamente");
    } else {
      DebugSerialPrintf("Error al conectar con IMU. Status: %d\n", imu.status);
      delay(500);
    }
  }
  
  if (!imuDetected) {
    DebugSerialPrintln("ERROR CRÍTICO: No se pudo detectar la IMU");
    DebugSerialPrintln("Verifica:");
    DebugSerialPrintln("  1. Conexión física del cable Qwiic");
    DebugSerialPrintln("  2. AD0_VAL debe ser 1 (dirección 0x69) o 0 (dirección 0x68)");
    DebugSerialPrintln("  3. Que no haya conflictos con otros dispositivos I2C");
    
    ledCtrl.setBlink(255, 0, 0, maxBrightness, 500);
    return;
  }
  
  DebugSerialPrintln("Inicializando DMP...");
  bool success = true;
  
  success &= (imu.initializeDMP() == ICM_20948_Stat_Ok);
  if (!success) {
    DebugSerialPrintln("ERROR: Falló initializeDMP()");
    ledCtrl.setBlink(255, 128, 0, maxBrightness, 300);
    return;
  }
  
  success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
  
  if (!success) {
    DebugSerialPrintln("ERROR: Falló habilitando sensores DMP");
    return;
  }
  
  success &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 1) == ICM_20948_Stat_Ok);
  success &= (imu.setDMPODRrate(DMP_ODR_Reg_Accel, 1) == ICM_20948_Stat_Ok);
  
  success &= (imu.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (imu.enableDMP() == ICM_20948_Stat_Ok);
  success &= (imu.resetDMP() == ICM_20948_Stat_Ok);
  success &= (imu.resetFIFO() == ICM_20948_Stat_Ok);
  
  if (!success) {
    DebugSerialPrintln("ERROR: Falló configurando FIFO/DMP");
    DebugSerialPrintln("Verifica que ICM_20948_USE_DMP esté definido en ICM_20948_C.h");
    ledCtrl.setBlink(0, 255, 0, maxBrightness, 300);
    return;
  }
  
  if (!EEPROM.begin(128)) {
    DebugSerialPrintln("ADVERTENCIA: Falló inicializando EEPROM");
  } else {
    EEPROM.get(0, store);
    
    if (StoreValido(&store)) {
      DebugSerialPrintln("Calibración válida encontrada en EEPROM");
      
      success &= (imu.setBiasGyroX(store.biasGyroX) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasGyroY(store.biasGyroY) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasGyroZ(store.biasGyroZ) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasAccelX(store.biasAccelX) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasAccelY(store.biasAccelY) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasAccelZ(store.biasAccelZ) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasCPassX(store.biasCPassX) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasCPassY(store.biasCPassY) == ICM_20948_Stat_Ok);
      success &= (imu.setBiasCPassZ(store.biasCPassZ) == ICM_20948_Stat_Ok);
      
      if (success) {
        DebugSerialPrintln("Calibración restaurada correctamente");
      } else {
        DebugSerialPrintln("ADVERTENCIA: Falló al aplicar calibración");
      }
    } else {
      DebugSerialPrintln("ADVERTENCIA: No hay calibración válida en EEPROM");
      DebugSerialPrintln("La IMU funcionará con valores por defecto");
    }
  }
  
  DebugSerialPrintln("IMU inicializada exitosamente");
  ledCtrl.setSolid(0, 255, 0, maxBrightness);
  delay(1000);
  ledCtrl.setOff();
}

void LeerYaw() {
  icm_20948_DMP_data_t data;
  imu.readDMPdataFromFIFO(&data);
  
  if ((imu.status != ICM_20948_Stat_Ok) && 
      (imu.status != ICM_20948_Stat_FIFOMoreDataAvail)) {
    DebugSerialPrintf("Error leyendo FIFO: %d\n", imu.status);
    return;
  }
  
  if ((data.header & DMP_header_bitmap_Quat9) == 0) {
    DebugSerialPrintln("No hay datos de Quaternion disponibles");
    return;
  }
  
  double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
  double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
  double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
  double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
  
  double t3 = +2.0 * (q0 * q3 + q1 * q2);
  double t4 = +1.0 - 2.0 * (q2 * q2 + q3 * q3);
  yaw = fmod(-atan2(t3, t4) * RAD_TO_DEG + 450.0, 360.0);
  
  DebugSerialPrintf("Yaw actual: %.2f°\n", yaw);
  
  if ((data.header & DMP_header_bitmap_Accel) > 0) {
    float accX = (float)data.Raw_Accel.Data.X / conversionFactor;
    float accY = (float)data.Raw_Accel.Data.Y / conversionFactor;
    float accZ = (float)data.Raw_Accel.Data.Z / conversionFactor;
    
    imuGravity = sqrt(accX * accX + accY * accY + accZ * accZ);
    DebugSerialPrintf("Gravedad: %.2f mm/s² (esperado ~9806.65)\n", imuGravity);
  }
  
  imu.resetFIFO();
}