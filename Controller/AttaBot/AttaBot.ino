#include "utils.h"
#include <Adafruit_APDS9960.h> // Adafruit_APDS9960. v1.3.0
#include <EEPROM.h>
#include <Freenove_WS2812_Lib_for_ESP32.h> // Freenove WS2812 Lib for ESP32. v1.0.5
#include <ICM_20948.h> // SparkFun ICM-20948 Arduino Library. v1.2.12
#include <Servo.h> // ServoESP32. v1.03
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <deque>
#include <map>

// Descomentar solo para debug !!!
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

// Constantes para el WiFi
const char* ssid = "Atta-Bot";
const char* password = "attabot1234";
const unsigned int localPort = 6060;
char receivedPacket[255];

// Constantes del robot empleado
const float pulsesPerRev = 574;             // Cantidad de pulsos por revolucion
const float wheelCircumference = PI * 44.5;  // Diametro de la rueda = 44.5mm
const float millimetersPerPulse = wheelCircumference / (float)pulsesPerRev;
const float centerToWheelDistance = 41.5;  // Radio de giro del carro, es la distancia en mm entre el centro y una rueda.

// Muestreo velocidad
const unsigned int samplingTime = 10;  // en ms
const float samplingTimeS = (float)samplingTime * 0.001;
unsigned long currentMillis = millis();
unsigned long previousMillis = millis();
unsigned long previousMillisRW = 0;
const unsigned int SteadyStateTime = 800;
unsigned long SteadyStatePreviousMillis = millis();
int millisDifference;
volatile int leftPulseCount = 0;
volatile int rightPulseCount = 0;
int pastLeftPulseCount = 0;
int pastRightPulseCount = 0;
int pastLeftEncoder = 0;
int pastRightEncoder = 0;
float currentLeftSpeed = 0.0;
float currentRightSpeed = 0.0;
const float distanceOffset = 1 * millimetersPerPulse;

// Constantes para la implementación del control PID
const int pwmFrequency = 1000;
const int pwmResolution = 14;  // bits
const int maxPWMValue = (1 << pwmResolution) - 1;
const int minPWMValue = maxPWMValue * 0.20;
const float baseSpeed = millimetersPerPulse / samplingTimeS;
const float maxSpeed = baseSpeed * 5; // Unidades mm/s
const float minSpeed = 12;
const int speedReductionThreshold = 16; 
pidConstants pidSpeed(110, 375, 2);
kalmanFilter kfPID(6.0, 1.0, 1.0);
pidController leftControl(kfPID, pidSpeed, samplingTimeS, minPWMValue, maxPWMValue);
pidController rightControl(kfPID, pidSpeed, samplingTimeS, minPWMValue, maxPWMValue);

const int observationPeriod = 28800; //us
const int observationTime = 1600; //us
const int numberOfCycles = observationPeriod / observationTime;
const int lateralCycle = random(numberOfCycles);
const int centralCycle = (lateralCycle + random(1, numberOfCycles)) % numberOfCycles;
const unsigned minObstacleTime = 1350; //us
unsigned long currentMicros = micros();
unsigned long previousMicros = micros();
bool isLateralCycleActive = false;
bool isCentralCycleActive = false;
int cycleCounter = 0;
int microsDifference;
volatile unsigned long leftObsStartTime = 0;
volatile unsigned long rightObsStartTime = 0;
unsigned long centralObsStartTime = 0;
bool leftObstacle = false;
bool centralObstacle = false;
bool rightObstacle = false;
bool obstacleDetected = false;
int obstacleSensors;
int centralDistance;
const int reverseDistance = -40;  // mm

int debugUdp = 0;
int debugCounter = 0;
char direction = '+';
const char* debugMessage = "DEBUG: %d, ID: %s, Direccion: %c, val: Izq|Der, Encoder: %d|%d, Vel: %.2f|%.2f, Pwm: %d|%d, ErrorP: %.2f|%.2f, ErrorI: %.2f|%.2f, Dis: %.2f|%.2f, Tiempo: %d";

String robotID = "-1";
std::map<String, IPAddress> robots;

enum PossibleStates { WAIT = 0,
                      READ_INSTRUCTION,
                      MOVE,
                      TURN,
                      STOP,
                      REVERSE,
                      RANDOM_WALK,
                      MESSAGE_BASE,
                      IDENTIFY_OBSTACLE };

PossibleStates state = WAIT;
float instructionValue = 100;
bool movementReady = true;
const int instructionCompletedDelay = 400;
std::array<float, 2> fsmInstruction;
std::deque<std::array<float, 2>> instructionList;

const std::array<int, 7> possibleAngles = { 30, 45, 60, 75, 90, 135, 180 };
const std::array<int, 4> possibleAdvances = { 200, 250, 300, 350 };
enum possibleDirections { TURN_POS = 0,
                          MOVE_FORWARD,
                          TURN_NEG };

const float gravity = 9806.65;
const float conversionFactor = 8192.0;
float yaw;
float imuGravity;
biasStore store;

int maxBrightness = 140;

volatile unsigned long lowBatteryTime = 0;
int minLowBatteryTime = 200;

float robotDistanceMargin = 260;
float maxRobotAngleMargin = 80;
bool robotDetected = false;
String fromRobotID = "";
const int obstacleWaitTime = 600;
pose robotPose(0, 0, 0);

int countMessages = 0;
int sendMessages = 0;

Servo frontServo;
Adafruit_APDS9960 frontSensor;
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(1, ledPin);
WiFiUDP udp;
ICM_20948_I2C imu;


/*****************************************************************************************

 Función que cuenta los pulsos del encoder de la rueda izquierda y determina la dirección
 del movimiento de la rueda (horario o antihorario) basándose en el cambio de estados 
 de los pines del encoder. 

 Utiliza `IRAM_ATTR` para que la función se ejecute en una interrupción, lo cual 
 garantiza una actualización rápida y eficiente de los pulsos de la rueda.

*****************************************************************************************/
void IRAM_ATTR LeftWheelPulses() {
  int MSB = digitalRead(leftEncoderC2);
  int LSB = digitalRead(leftEncoderC1);
  int encoder = (MSB << 1) | LSB;
  int sum = (pastLeftEncoder << 2) | encoder;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    leftPulseCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    leftPulseCount--;
  }
  pastLeftEncoder = encoder;
}


/***************************************************************************************

 Función que cuenta los pulsos del encoder de la rueda derecha y determina la dirección
 del movimiento de la rueda (horario o antihorario) basándose en el cambio de estados 
 de los pines del encoder.

 Utiliza `IRAM_ATTR` para que la función se ejecute en una interrupción, lo cual 
 garantiza una actualización rápida y eficiente de los pulsos de la rueda.

***************************************************************************************/
void IRAM_ATTR RightWheelPulses() {
  int MSB = digitalRead(rightEncoderC1);
  int LSB = digitalRead(rightEncoderC2);
  int encoder = (MSB << 1) | LSB;
  int sum = (pastRightEncoder << 2) | encoder;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    rightPulseCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    rightPulseCount--;
  }
  pastRightEncoder = encoder;
}


/********************************************************************************************

 Función que detecta un obstáculo en la izquierda del robot utilizando un sensor infrarrojo.

 Utiliza `IRAM_ATTR` para que la función se ejecute en una interrupción, lo cual 
 garantiza una actualización rápida y eficiente del esado del sensor.

********************************************************************************************/
void IRAM_ATTR DetectLeftObstacle() {
  if (digitalRead(leftInfraredSensor) == LOW) {
    leftObsStartTime = micros();
  }
}


/******************************************************************************************

 Función que detecta un obstáculo en la derecha del robot utilizando un sensor infrarrojo.

 Utiliza `IRAM_ATTR` para que la función se ejecute en una interrupción, lo cual 
 garantiza una actualización rápida y eficiente del esado del sensor.

******************************************************************************************/
void IRAM_ATTR DetectRightObstacle() {
  if (digitalRead(rightInfraredSensor) == LOW) {
    rightObsStartTime = micros();
  }
}


/***************************************************************************************

 Función que verifica el estado de la batería y registra el tiempo desde que la 
 batería es baja.

***************************************************************************************/
void LowBattery() {
  if (digitalRead(batteryStatus) == LOW) {
    lowBatteryTime = millis();
  }
}


/***************************************************************************************

 Función de configuración del robot que se ejecuta una vez al inicio del programa.
 Esta función inicializa los componentes del robot, configura las interrupciones, 
 establece la comunicación Wi-Fi y el servidor UDP, y ajusta la configuración de 
 sensores y motores.

***************************************************************************************/
void setup() {
  #ifdef DebugSerial
    Serial.begin(115200);
  #endif

  analogWriteFrequency(pwmFrequency);
  analogWriteResolution(pwmResolution);

  WiFi.begin(ssid, password);
  strip.begin();
  WiFiStatus();
  ArduinoOTA.begin();
  udp.begin(localPort);
  DebugSerialPrintf("El servidor UDP se inició en el puerto: %u\n", localPort);
  Wire.begin();
  Wire.setClock(400000);
  // setupIMU();

  frontServo.attach(frontServoPin);
  frontServo.write(90);
  delay(500);

  pinMode(leftEncoderC1, INPUT_PULLUP);
  pinMode(leftEncoderC2, INPUT_PULLUP);
  pinMode(rightEncoderC1, INPUT_PULLUP);
  pinMode(rightEncoderC2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC1), LeftWheelPulses, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC2), LeftWheelPulses, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC1), RightWheelPulses, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC2), RightWheelPulses, CHANGE);

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

  SetupFrontSensor();
  delay(200);
}


/***************************************************************************************

 Función principal del programa que gestiona el ciclo de operación del robot.
 Esta función es responsable de controlar el flujo de acciones del robot, que incluye
 la lectura de paquetes UDP, la verificación de sensores y la gestión de estados 
 a través de una maquina de estados (FSM).

 Dependiendo del estado actual del robot, se ejecutan diferentes acciones:
 - WAIT: Espera instrucciones y reinicia el PID si es necesario.
 - MOVE: Mueve el robot una distancia especificada.
 - TURN: Realiza un giro a una distancia especificada.
 - RANDOM_WALK: Ejecuta el comportamiento de exploracion aleatoria
 - REVERSE: Retrocede si se detecta un obstáculo.
 - STOP: Detiene el movimiento y evalúa el siguiente estado.
 - READ_INSTRUCTION: Lee la siguiente instrucción de la lista de instrucciones.
 - MESSAGE_BASE: Envía un mensaje a la base con el estado del robot.
 - IDENTIFY_OBSTACLE: Identifica si el obstáculo detectado es otro robot o no.

***************************************************************************************/
void loop() {
  WiFiStatus();
  ReadUdpPackets();
  ReadSensors();

  switch (state) {
    case WAIT: {
      ArduinoOTA.handle();

      if ((millis() - previousMillis) >= instructionValue) {
        previousMillis = millis();
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
        MessageDebugf("DEBUG: -1, ID: %s, Movimiento completado", robotID);
        state = STOP;
      } else if (leftObstacle || centralObstacle || rightObstacle) {
        obstacleSensors = (leftObstacle << 2) | (centralObstacle << 1) | rightObstacle; 
        state = STOP;
      }

      break;
    }

    case TURN: {
      movementReady = MoveDistanceByWheel(instructionValue, -instructionValue);

      if (movementReady) {
        MessageDebugf("DEBUG: -1, ID: %s, Giro completado", robotID);
        state = STOP;
      } else if (leftObstacle || centralObstacle || rightObstacle) {
        obstacleSensors = (leftObstacle << 2) | (centralObstacle << 1) | rightObstacle;
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
        MessageDebugf("DEBUG: -1, ID: %s, Random Walk terminado", robotID);
      }

      // CommunicationTest();

      state = READ_INSTRUCTION;
      break;
    }

    case REVERSE: {
      movementReady = MoveDistanceByWheel(reverseDistance, reverseDistance);

      if (movementReady) {
        obstacleDetected = true;
        MessageDebugf("DEBUG: -1, ID: %s, Obstaculo evitado", robotID);
        state = STOP;
      }

      break;
    }

    case STOP: {
      ConfigureHBridge(0, 0);
      if (movementReady == true) {
        state = WAIT;
        instructionValue = instructionCompletedDelay;
      } else {
        SendPose();
        state = IDENTIFY_OBSTACLE;
      }
      
      break;
    }

    case READ_INSTRUCTION: {
      if (!instructionList.empty()) {
        fsmInstruction = instructionList.front();
        instructionList.pop_front();
        instructionValue = fsmInstruction[1];
        state = static_cast<PossibleStates>(fsmInstruction[0]);
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
      if (robotDetected) {
        state = WAIT;
        instructionValue = instructionCompletedDelay / 2;
        MessageDebugf("DEBUG: -1, ID: %s, Obstaculo encontrado es robot id: %s", robotID, fromRobotID);
      } else if ((currentMillis - previousMillis) >= obstacleWaitTime) {
        previousMillis = currentMillis;
        state = WAIT;
        instructionValue = 0;
        MessageDebugf("DEBUG: -1, ID: %s, Obstaculo encontrado no es un robot", robotID);
      }

      break;
    }
  }
}


/***************************************************************************************

 Función que verifica el estado de la conexión Wi-Fi y espera hasta que el robot esté 
 conectado a la red. Durante la conexión, se apagan los motores y se indica el estado 
 de conexión a través de un parpadeo en los LEDs del strip.

***************************************************************************************/
void WiFiStatus() {
  while (WiFi.status() != WL_CONNECTED) {
    ConfigureHBridge(0, 0);
    DebugSerialPrintln("Conectando WiFi ...");
    for (int brillo : { maxBrightness, 0 }) {
      strip.setBrightness(brillo);
      strip.setAllLedsColor(strip.Wheel(170));
      delay(250);
    }
  }
}


/***************************************************************************************

 Función que inicializa el sensor frontal APDS-9960 y asegura que esté funcionando 
 correctamente antes de continuar. Si la inicialización falla, muestra un mensaje de 
 error en el puerto serial y utiliza un parpadeo en los LEDs del strip para indicar el 
 estado de fallo hasta que el sensor responda correctamente.

***************************************************************************************/
void SetupFrontSensor() {
  while (!frontSensor.begin()) {
    DebugSerialPrintln("Fallo la inialización del sensor APDS-9960.");
    for (int brillo : { maxBrightness, 0 }) {
      strip.setBrightness(brillo);
      strip.setAllLedsColor(strip.Wheel(0));
      delay(250);
    }
  }
}


/*******************************************************************************************

 Función que envía un mensaje de prueba a cada robot registrado en la red (excepto 
 "Broadcast" y la "Base") para verificar la comunicación. Se limita a enviar un total de 500 
 mensajes durante la prueba. Una vez alcanzado el límite de mensajes, se indica el final de 
 la prueba al encender todos los LEDs del strip a su máximo brillo en color rojo durante 
 medio segundo.

*******************************************************************************************/
void CommunicationTest(){
  if (sendMessages < 500) {
    sendMessages ++;
    for (const auto& pair : robots) {
      if (pair.first != "Broadcast" && pair.first != "Base"){
        SendMessage(pair.second, "COUNT_MESSAGE");
      }
    }
  } else {
    strip.setBrightness(255);
    strip.setAllLedsColor(strip.Wheel(0));
    delay(500);
  }
}


/***************************************************************************************

 Función que separa un comando en substrings basados en un delimitador dado, y 
 almacena los resultados en un array de tamaño fijo de cinco elementos. Esta función 
 permite manejar hasta cinco partes de un comando. 

 @param command   La cadena de texto que contiene el comando a separar.
 @param delimiter El carácter delimitador que indica dónde dividir la cadena.
 @return          Un array de cinco elementos que contiene las partes separadas de la 
                  cadena; las posiciones restantes quedan vacías si el número de partes 
                  es menor que cinco.

***************************************************************************************/
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


/***************************************************************************************

 Función que envía un mensaje a una dirección IP específica mediante el protocolo UDP. 
 Utiliza el puerto definido localmente y asegura la conversión adecuada del mensaje para 
 el envío.

 @param host    La dirección IP del dispositivo receptor del mensaje.
 @param message El mensaje a enviar, en formato de cadena de caracteres.

***************************************************************************************/
void SendMessage(IPAddress host, const char* message) {
  udp.beginPacket(host, localPort);
  udp.write(reinterpret_cast<const uint8_t*>(message), strlen(message));
  udp.endPacket();
}


/***************************************************************************************

 Función que determina si un obstáculo detectado es otro robot en función de su posición,
 orientación y sensores activados. Calcula la distancia y el ángulo entre el robot actual 
 y el posible obstáculo.

 @param x2       La posición en x del posible robot-obstáculo.
 @param y2       La posición en y del posible robot-obstáculo.
 @param angle    El ángulo del posible robot-obstáculo.
 @param sensors  Sensores activados del posible robot-obstáculo.
 @param id       Identificación del posible robot-obstáculo.
 
 @return         Retorna `true` si el obstáculo es identificado como otro robot, `false`
                 en caso contrario.

***************************************************************************************/
bool IsRobotObstacle(float x2, float y2, float angle, int sensors, String id) {
  float deltaX = x2 - robotPose.x;
  float deltaY = y2 - robotPose.y;

  float distanceBetweenRobots = sqrt(deltaX * deltaX + deltaY * deltaY);
  if (distanceBetweenRobots > robotDistanceMargin) {
      return false;
  }

  float angleBetweenRobots = atan2f(deltaY, deltaX) * RAD_TO_DEG + 180;
  float angleDifference = angleBetweenRobots - angle;

  // Ajustar para que la diferencia angular esté en el rango [-180, 180]
  if (angleDifference > 180) {
    angleDifference -= 360;
  } else if (angleDifference < -180) {
    angleDifference += 360;
  }
  MessageDebugf("DEBUG: -1, ID: %s, From ID: %s, Distancia: %.1f, Angulo: %.1f, DifAngulo: %.1f, sensors: %d", robotID, id, distanceBetweenRobots, angleBetweenRobots, angleDifference, sensors);

  if (_abs(angleDifference) <= maxRobotAngleMargin) {
    if (sensors == 0b100 && angleDifference <= 0) {
      return true;
    } else if (sensors == 0b001 && angleDifference >= 0) {
      return true;
    } else if (sensors != 0b100 && sensors != 0b001){
      return true;
    }
  }

  return false;
}


/***************************************************************************************

 Función que lee los paquetes UDP recibidos y ejecuta diferentes comandos según el 
 contenido del mensaje. Permite configurar la red y los robots, mover el robot, ajustar 
 constantes de PID y filtros de Kalman, verificar la posición del robot, y detectar 
 obstáculos en el entorno. Cada comando se procesa y se actúa en consecuencia, 
 manteniendo la comunicación en tiempo real con otros robots o con la base.

***************************************************************************************/
void ReadUdpPackets() {
  int packetBytes = udp.parsePacket();
  if (!packetBytes) {
    return;
  }

  int len = udp.read(receivedPacket, sizeof(receivedPacket) - 1);
  if (len > 0) {
    receivedPacket[len] = 0;
  }
  DebugSerialPrintf("Recibidos %d bytes de %s: %s\n", packetBytes, udp.remoteIP().toString().c_str(), receivedPacket);

  String command(receivedPacket);
  std::array<String, 5> arguments = SeparateCommand(command, '|');
  command = arguments[0];

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
        DebugSerialPrintf("Nombre: %s, IP: %s\n", pair.first, pair.second.toString().c_str());
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

  } else if (command == "MOVE" || command == "TURN" || command == "WAIT" || command == "RANDOMW" || command == "MESSAGE_BASE") {
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

  } else if (command == "RESET") {
    ESP.restart();

  } else if (command == "SERVO") {
    int servoAngle = constrain(arguments[1].toInt(), 5, 175);
    DebugSerialPrintf("Servo: %d°\n", servoAngle);
    frontServo.write(servoAngle);

  } else if (command == "PID") {
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

  } else if (command == "KFPID") {
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

  } else if (command == "POSE") {
    robotPose.x = arguments[1].toFloat();
    robotPose.y = arguments[2].toFloat();
    robotPose.angle = arguments[3].toFloat();

  } else if (command == "CHECK_OBSTACLE") {
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
      snprintf(buffer, sizeof(buffer), "OBSTACLE_DETECTED|%s", robotID);
      delayMicroseconds(800);
      SendMessage(robots[id], buffer);
      SendMessage(robots[id], buffer);
    }

  } else if (command == "OBSTACLE_DETECTED") {
    fromRobotID = arguments[1];
    robotDetected = true;

  } else if (command == "COUNT_MESSAGE") {
    countMessages ++;

  } else if (command == "SEND_COUNT_MESSAGE") {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Robot ID: %s, Total messages: %d", robotID, countMessages);
    SendMessage(robots["Base"], buffer);
  }
}


/***************************************************************************************

 Función que lee el estado de los sensores de proximidad y la batería. En cada ciclo, 
 activa los sensores infrarrojos laterales y el sensor frontal de proximidad de acuerdo 
 con el ciclo de lectura configurado. Verifica la presencia de obstáculos en los tres 
 sensores y ajusta el estado de los LEDs para indicar si hay una detección de obstáculos 
 o batería baja.

***************************************************************************************/
void ReadSensors() {
  if (((millis() - previousMillis) <= samplingTime - 2) || isLateralCycleActive || isCentralCycleActive || (debugUdp == 3)) {
    currentMicros = micros();
    if (currentMicros - previousMicros >= observationTime) {
      previousMicros = currentMicros;
      cycleCounter = (cycleCounter + 1) % numberOfCycles;

      isLateralCycleActive = (lateralCycle == cycleCounter);
      digitalWrite(enableLeftInfraredSensor, isLateralCycleActive);
      digitalWrite(enableRightInfraredSensor, isLateralCycleActive);

      isCentralCycleActive = (centralCycle == cycleCounter);
      frontSensor.enableProximity(isCentralCycleActive);
    }

    if (isLateralCycleActive) {     
      leftObstacle = (digitalRead(leftInfraredSensor) == LOW) && ((micros() - leftObsStartTime) >= minObstacleTime);
      rightObstacle = (digitalRead(rightInfraredSensor) == LOW) && ((micros() - rightObsStartTime) >= minObstacleTime);
    }

    if (isCentralCycleActive) {
      centralDistance = frontSensor.readProximity();
      if (centralDistance > 2) {
        centralObstacle = (micros() - centralObsStartTime) >= minObstacleTime / 2;
      } else {
        centralObsStartTime = micros();
        centralObstacle = false;
      }
    }
  }

  if (debugUdp == 3) {
    if (leftObstacle || centralObstacle || rightObstacle) {
      strip.setBrightness(maxBrightness);
      strip.setAllLedsColor(strip.Wheel(200));
    } else {
      strip.setBrightness(0);
      strip.setAllLedsColor(strip.Wheel(200));
    }
  } else {
    if ((digitalRead(batteryStatus) == LOW) && ((millis() - lowBatteryTime) >= minLowBatteryTime)) {
      strip.setBrightness(255);
      strip.setAllLedsColor(strip.Wheel(120));
    }
  }
}


/***************************************************************************************

 Función que reinicia los controladores PID y las variables de conteo de pulsos para 
 ambos motores. Se utiliza para restablecer el estado de los controladores y las 
 métricas relacionadas al control de movimiento antes de iniciar un nuevo ciclo.

***************************************************************************************/
void ResetPID() {
  leftControl.Reset();
  rightControl.Reset();
  debugCounter = 0;
  leftPulseCount = 0;
  rightPulseCount = 0;
  pastLeftPulseCount = 0;
  pastRightPulseCount = 0;
}


/***************************************************************************************

 Configura el puente H para controlar la dirección y velocidad de los motores de las ruedas.
 Ajusta los pines de PWM para los motores izquierdo y derecho según el valor del PWM 
 proporcionado. Si el valor de PWM es positivo, el motor gira hacia adelante; si es 
 negativo, hacia atrás.

 @param leftWheelPWM Valor de PWM para el motor de la rueda izquierda.
 @param rightWheelPWM Valor de PWM para el motor de la rueda derecha.

***************************************************************************************/
void ConfigureHBridge(int leftWheelPWM, int rightWheelPWM) {
  if (leftWheelPWM >= 0) {
    analogWrite(leftMotorBackward, 0);
    analogWrite(leftMotorForward, leftWheelPWM);
  } else if (leftWheelPWM < 0) {
    analogWrite(leftMotorForward, 0);
    analogWrite(leftMotorBackward, _abs(leftWheelPWM));
  }

  if (rightWheelPWM >= 0) {
    analogWrite(rightMotorBackward, 0);
    analogWrite(rightMotorForward, rightWheelPWM);
  } else if (rightWheelPWM < 0) {
    analogWrite(rightMotorForward, 0);
    analogWrite(rightMotorBackward, _abs(rightWheelPWM));
  }
}


/***************************************************************************************

 Calcula la velocidad deseada en función de la distancia restante.
 Si la distancia restante es menor que el umbral de reducción de velocidad, ajusta la 
 velocidad deseada utilizando un mapeo lineal entre la velocidad mínima y máxima.

 @param distance Distancia total que se desea recorrer.
 @param wheelDistance Distancia que ya se ha recorrido por las ruedas.

 @return Velocidad deseada, que puede ser positiva o negativa según la distancia restante.

***************************************************************************************/
float DesiredSpeed(float distance, float wheelDistance) {
  float remainingDistance = distance - wheelDistance;
  float desiredSpeed = maxSpeed;
  if (_abs(remainingDistance) < speedReductionThreshold) {
    desiredSpeed = map(_abs(remainingDistance), 0, speedReductionThreshold, minSpeed, maxSpeed);
  }

  return (remainingDistance < 0) ? -desiredSpeed : desiredSpeed;
}


/***************************************************************************************

 Determina si el robot está en estado estacionario basado en la velocidad actual de las ruedas
 y la distancia recorrida. Si la velocidad de ambas ruedas es cero y han recorrido una 
 distancia significativa en un tiempo dado, se considera que el robot está estacionario.

 @param currentLeftSpeed Velocidad actual de la rueda izquierda.
 @param currentRightSpeed Velocidad actual de la rueda derecha.
 @param leftWheelDistance Distancia recorrida por la rueda izquierda.
 @param rightWheelDistance Distancia recorrida por la rueda derecha.

 @return true si el robot está en estado estacionario, false en caso contrario.

***************************************************************************************/
bool IsStationary(float currentLeftSpeed, float currentRightSpeed, float leftWheelDistance, float rightWheelDistance) {
  bool speedsAtZero = (static_cast<int>(_abs(currentLeftSpeed) + _abs(currentRightSpeed)) == 0);
  bool wheelsHaveMoved = (static_cast<int>(_abs(leftWheelDistance) + _abs(rightWheelDistance)) != 0);
  if (!speedsAtZero) {
    SteadyStatePreviousMillis = millis();
  } else if ((millis() - SteadyStatePreviousMillis >= SteadyStateTime) && wheelsHaveMoved) {
    return true;
  }

  return false;
}


/***************************************************************************************

 Envía un mensaje de depuración formateado a la consola y si el modo de depuración 
 está activado, también lo envía a la base. La función permite el uso de 
 especificadores de formato similares a printf.

 @param format Formato del mensaje a imprimir, similar a printf.
 @param ... Parámetros adicionales que se utilizarán en el formato.

***************************************************************************************/
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


/***************************************************************************************

 Controla el movimiento del robot para que recorra una distancia específica con cada 
 rueda. Calcula la velocidad actual de las ruedas basándose en la cantidad de pulsos 
 de cada rueda desde la última llamada. Luego, determina la velocidad deseada y 
 ajusta el PWM para controlar el puente H. La función retorna verdadero si se ha 
 alcanzado la distancia deseada o si el robot está estacionario.

 @param leftDistance Distancia que la rueda izquierda debe recorrer en milímetros.
 @param rightDistance Distancia que la rueda derecha debe recorrer en milímetros.

 @return true si el movimiento ha terminado, false en caso contrario.

***************************************************************************************/
bool MoveDistanceByWheel(float leftDistance, float rightDistance) {
  currentMillis = millis();
  millisDifference = currentMillis - previousMillis;
  if (millisDifference < samplingTime) {
    return false;
  }

  previousMillis = currentMillis;

  // Se calculan las velocidades de ambas ruedas
  currentLeftSpeed = ((leftPulseCount - pastLeftPulseCount) * millimetersPerPulse) / samplingTimeS;  // velocidad en mm/s
  pastLeftPulseCount = leftPulseCount;
  currentRightSpeed = ((rightPulseCount - pastRightPulseCount) * millimetersPerPulse) / samplingTimeS;  // velocidad en mm/s
  pastRightPulseCount = rightPulseCount;

  float leftWheelDistance = pastLeftPulseCount * millimetersPerPulse;
  float desiredLeftSpeed = DesiredSpeed(leftDistance, leftWheelDistance);
  int leftWheelPWM = leftControl.Calculate(desiredLeftSpeed, currentLeftSpeed);

  float rightWheelDistance = pastRightPulseCount * millimetersPerPulse;
  float desiredRightSpeed = DesiredSpeed(rightDistance, rightWheelDistance);
  int rightWheelPWM = rightControl.Calculate(desiredRightSpeed, currentRightSpeed);

  ConfigureHBridge(leftWheelPWM, rightWheelPWM);
  bool IsMoveFinished = ((_abs(leftWheelDistance) + distanceOffset) >= _abs(leftDistance)) &&
                     ((_abs(rightWheelDistance) + distanceOffset) >= _abs(rightDistance));
  IsMoveFinished = IsMoveFinished || IsStationary(currentLeftSpeed, currentRightSpeed, leftWheelDistance, rightWheelDistance);
  
  if (debugUdp >= 2){
    MessageDebugf(debugMessage, debugCounter, robotID, direction, leftPulseCount, rightPulseCount, currentLeftSpeed, currentRightSpeed, leftWheelPWM, rightWheelPWM,
                  leftControl.error, rightControl.error, leftControl.sumError, rightControl.sumError, leftWheelDistance, rightWheelDistance, millisDifference);
  }

  return IsMoveFinished;
}


/***************************************************************************************

 Envía la posición actual del robot junto con el estado de los sensores de obstáculos 
 a otros robots en la red.

***************************************************************************************/
void SendPose() {
  robotDetected = false;
  const char* message = "CHECK_OBSTACLE|%d|%.1f|%.1f|%.1f";
  char buffer[40];
  snprintf(buffer, sizeof(buffer), message, obstacleSensors, robotPose.x, robotPose.y, robotPose.angle);
  SendMessage(robots["Broadcast"], buffer);
  previousMillis - millis();
}


/***************************************************************************************

 Selecciona aleatoriamente el siguiente movimiento que el robot realizará, considerando
 si hay obstáculos detectados. El robot puede girar a la derecha, avanzar hacia adelante
 o girar a la izquierda. La selección se basa en probabilidades predefinidas y se 
 almacena la instrucción correspondiente en una deque.

***************************************************************************************/
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
  // imu.enableDebugging();
  bool success = false;
  while (!success) {
    imu.begin(Wire, AD0_VAL);
    if (imu.status != ICM_20948_Stat_Ok) {
      DebugSerialPrintln("Fallo inicializando la IMU, provando de nuevo...");
      for (int brillo : { maxBrightness, 0 }) {
        strip.setBrightness(brillo);
        strip.setAllLedsColor(strip.Wheel(85));
        delay(250);
      }
    } else {
      success = true;
    }
  }

  success &= (imu.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);

  // E.g. For a 225Hz ODR rate when DMP is running at 225Hz, value = (225/112.5) - 1 = 1.
  success &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 1) == ICM_20948_Stat_Ok);
  success &= (imu.setDMPODRrate(DMP_ODR_Reg_Accel, 1) == ICM_20948_Stat_Ok);

  success &= (imu.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (imu.enableDMP() == ICM_20948_Stat_Ok);
  success &= (imu.resetDMP() == ICM_20948_Stat_Ok);
  success &= (imu.resetFIFO() == ICM_20948_Stat_Ok);

  if (!success) {
    DebugSerialPrintln("Fallo la inicializacion del DPM.");
    DebugSerialPrintln("Verifique que la linea 29 (#define ICM_20948_USE_DMP) de ICM_20948_C.h este sin comentar.");
    strip.setBrightness(maxBrightness);
    strip.setAllLedsColor(strip.Wheel(85));
  }

  if (!EEPROM.begin(128)) {
    DebugSerialPrintln("Fallo la inicializacion de la EEPROM.");
    strip.setBrightness(maxBrightness);
    strip.setAllLedsColor(strip.Wheel(0));
  }

  EEPROM.get(0, store);
  if (StoreValido(&store)) {
    success &= (imu.setBiasGyroX(store.biasGyroX) == ICM_20948_Stat_Ok);
    success &= (imu.setBiasGyroY(store.biasGyroY) == ICM_20948_Stat_Ok);
    success &= (imu.setBiasGyroZ(store.biasGyroZ) == ICM_20948_Stat_Ok);
    success &= (imu.setBiasAccelX(store.biasAccelX) == ICM_20948_Stat_Ok);
    success &= (imu.setBiasAccelY(store.biasAccelY) == ICM_20948_Stat_Ok);
    success &= (imu.setBiasAccelZ(store.biasAccelZ) == ICM_20948_Stat_Ok);
    success &= (imu.setBiasCPassX(store.biasCPassX) == ICM_20948_Stat_Ok);
    success &= (imu.setBiasCPassY(store.biasCPassY) == ICM_20948_Stat_Ok);
    success &= (imu.setBiasCPassZ(store.biasCPassZ) == ICM_20948_Stat_Ok);

    if (!success) {
      DebugSerialPrintln("No se pudo restaurar la calibracion de la IMU.");
      strip.setBrightness(maxBrightness);
      strip.setAllLedsColor(strip.Wheel(0));
    }
  } else {
    DebugSerialPrintln("No se encontro una calibracion valida para la IMU.");
    strip.setBrightness(maxBrightness);
    strip.setAllLedsColor(strip.Wheel(0));
  }
}

void LeerYaw() {
  icm_20948_DMP_data_t data;
  imu.readDMPdataFromFIFO(&data);
  if (((imu.status == ICM_20948_Stat_Ok) || (imu.status == ICM_20948_Stat_FIFOMoreDataAvail)) && true) {
    if ((data.header & DMP_header_bitmap_Quat9) > 0) {
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2 * q2 + q3 * q3);
      yaw = fmod(-atan2(t3, t4) * RAD_TO_DEG + 450.0, 360.0);
    }
  }

  imu.resetFIFO();
}

ICM_20948_Status_e ICM_20948::initializeDMP(void) {
  ICM_20948_Status_e result = ICM_20948_Stat_Ok;
  ICM_20948_Status_e worstResult = ICM_20948_Stat_Ok;
  result = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true);
  if (result > worstResult) worstResult = result;
  result = i2cControllerConfigurePeripheral(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single);
  if (result > worstResult) worstResult = result;
  result = setBank(3);
  if (result > worstResult) worstResult = result;
  uint8_t mstODRconfig = 0x04;
  result = write(AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1);
  if (result > worstResult) worstResult = result;
  result = setClockSource(ICM_20948_Clock_Auto);
  if (result > worstResult) worstResult = result;
  result = setBank(0);
  if (result > worstResult) worstResult = result;
  uint8_t pwrMgmt2 = 0x40;
  result = write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1);
  if (result > worstResult) worstResult = result;
  result = setSampleMode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled);
  if (result > worstResult) worstResult = result;
  result = enableFIFO(false);
  if (result > worstResult) worstResult = result;
  result = enableDMP(false);
  if (result > worstResult) worstResult = result;

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS;
  myFSS.a = gpm4;     // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                      // gpm2
                      // gpm4
                      // gpm8
                      // gpm16
  myFSS.g = dps2000;  // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                      // dps250
                      // dps500
                      // dps1000
                      // dps2000
  result = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (result > worstResult) worstResult = result;

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;   // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d5bw7_n8bw3;  // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                 // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                 // acc_d111bw4_n136bw
                                 // acc_d50bw4_n68bw8
                                 // acc_d23bw9_n34bw4
                                 // acc_d11bw5_n17bw
                                 // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                 // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d5bw7_n8bw9;  // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                 // gyr_d196bw6_n229bw8
                                 // gyr_d151bw8_n187bw6
                                 // gyr_d119bw5_n154bw3
                                 // gyr_d51bw2_n73bw3
                                 // gyr_d23bw9_n35bw9
                                 // gyr_d11bw6_n17bw8
                                 // gyr_d5bw7_n8bw9
                                 // gyr_d361bw4_n376bw5

  result = setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  if (result > worstResult) worstResult = result;
  result = enableDLPF(ICM_20948_Internal_Acc, true);
  if (result > worstResult) worstResult = result;
  result = enableDLPF(ICM_20948_Internal_Gyr, true);
  if (result > worstResult) worstResult = result;
  result = setBank(0);
  if (result > worstResult) worstResult = result;
  uint8_t zero = 0;
  result = write(AGB0_REG_FIFO_EN_1, &zero, 1);
  if (result > worstResult) worstResult = result;
  result = write(AGB0_REG_FIFO_EN_2, &zero, 1);
  if (result > worstResult) worstResult = result;
  result = intEnableRawDataReady(false);
  if (result > worstResult) worstResult = result;
  result = resetFIFO();
  if (result > worstResult) worstResult = result;

  ICM_20948_smplrt_t mySmplrt;
  //mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  //mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.g = 4;  // 225Hz
  mySmplrt.a = 4;  // 225Hz
  //mySmplrt.g = 8; // 112Hz
  //mySmplrt.a = 8; // 112Hz
  result = setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt);
  if (result > worstResult) worstResult = result;
  result = setDMPstartAddress();
  if (result > worstResult) worstResult = result;
  result = loadDMPFirmware();
  if (result > worstResult) worstResult = result;
  result = setDMPstartAddress();
  if (result > worstResult) worstResult = result;
  result = setBank(0);
  if (result > worstResult) worstResult = result;
  uint8_t fix = 0x48;
  result = write(AGB0_REG_HW_FIX_DISABLE, &fix, 1);
  if (result > worstResult) worstResult = result;
  result = setBank(0);
  if (result > worstResult) worstResult = result;
  uint8_t fifoPrio = 0xE4;
  result = write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1);
  if (result > worstResult) worstResult = result;

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const unsigned char accScale[4] = { 0x04, 0x00, 0x00, 0x00 };
  result = writeDMPmems(ACC_SCALE, 4, &accScale[0]);
  if (result > worstResult) worstResult = result;  // Write accScale to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const unsigned char accScale2[4] = { 0x00, 0x04, 0x00, 0x00 };
  result = writeDMPmems(ACC_SCALE2, 4, &accScale2[0]);
  if (result > worstResult) worstResult = result;  // Write accScale2 to ACC_SCALE2 DMP register


  const unsigned char mountMultiplierZero[4] = { 0x00, 0x00, 0x00, 0x00 };
  const unsigned char mountMultiplierPlus[4] = { 0x09, 0x99, 0x99, 0x99 };
  const unsigned char mountMultiplierMinus[4] = { 0xF6, 0x66, 0x66, 0x67 };
  result = writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]);
  if (result > worstResult) worstResult = result;
  const unsigned char b2sMountMultiplierZero[4] = { 0x00, 0x00, 0x00, 0x00 };
  const unsigned char b2sMountMultiplierPlus[4] = { 0x40, 0x00, 0x00, 0x00 };  // Value taken from InvenSense Nucleo example
  result = writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]);
  if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]);
  if (result > worstResult) worstResult = result;

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  result = setGyroSF(4, 3);
  if (result > worstResult) worstResult = result;  // 4 = 225Hz (see above), 3 = 2000dps (see above)

  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const unsigned char gyroFullScale[4] = { 0x10, 0x00, 0x00, 0x00 };
  result = writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]);
  if (result > worstResult) worstResult = result;

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  //const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  const unsigned char accelOnlyGain[4] = { 0x00, 0xE8, 0xBA, 0x2E };  // 225Hz
  //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
  result = writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]);
  if (result > worstResult) worstResult = result;

  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  //const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  const unsigned char accelAlphaVar[4] = { 0x3D, 0x27, 0xD2, 0x7D };  // 225Hz
  //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
  result = writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]);
  if (result > worstResult) worstResult = result;

  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  //const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  const unsigned char accelAVar[4] = { 0x02, 0xD8, 0x2D, 0x83 };  // 225Hz
  //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
  result = writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]);
  if (result > worstResult) worstResult = result;

  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = { 0x00, 0x00 };  // Value taken from InvenSense Nucleo example
  result = writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]);
  if (result > worstResult) worstResult = result;

  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  // const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
  const unsigned char compassRate[2] = { 0x00, 0xE1 };  // 225Hz
  result = writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]);
  if (result > worstResult) worstResult = result;

  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  //result = intEnableDMP(true); if (result > worstResult) worstResult = result;

  return worstResult;
}
