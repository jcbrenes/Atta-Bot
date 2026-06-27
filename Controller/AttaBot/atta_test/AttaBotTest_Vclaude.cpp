// ============================================================================
// AttaBotTest.ino — Sketch de diagnóstico independiente para AttaBot V2
// ============================================================================
// Permite verificar motores, encoders, sensores IR, servo y sensor frontal
// sin necesidad de conexión WiFi ni sistema de visión externo.
//
// INSTRUCCIONES DE USO:
//   1. Abre el Serial Monitor a 115200 baudios
//   2. El robot arranca en MODO MENÚ (LED azul parpadeando)
//   3. Escribe el número de prueba y presiona Enter
//   4. Observa resultados en Serial Monitor + comportamiento del LED
//
// MENÚ DE PRUEBAS:
//   1  → TEST MOTORES (adelante/atrás/izq/der, verificación encoders)
//   2  → TEST ENCODERS (cuenta pulsos en tiempo real, calcula mm/pulso)
//   3  → TEST SENSORES IR (izquierdo y derecho en bucle)
//   4  → TEST SENSOR FRONTAL APDS9960 (proximidad en tiempo real)
//   5  → TEST SERVO (barrido 0→90→180→90)
//   6  → TEST IMU (yaw en tiempo real)
//   7  → TEST BATERÍA (lee pin batteryStatus)
//   8  → CALIBRACIÓN PPR (mide pulsesPerRev con movimiento real)
//   9  → TEST COMPLETO (ejecuta 1-7 en secuencia)
//   0  → VOLVER AL MENÚ
// ============================================================================

#include "utils.h"
#include <Adafruit_APDS9960.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <FastLED.h>
#include <ICM_20948.h>
#include <Preferences.h>
#include <Wire.h>

// ============================================================================
// PINES (copiados de AttaBot.ino principal)
// ============================================================================

#define leftMotorForward  12
#define leftMotorBackward 14
#define rightMotorForward 13
#define rightMotorBackward 15

#define leftEncoderC1  32
#define leftEncoderC2  35
#define rightEncoderC1 23
#define rightEncoderC2 25

#define enableLeftInfraredSensor  5
#define leftInfraredSensor        33
#define frontServoPin             26
#define enableRightInfraredSensor 18
#define rightInfraredSensor       27

#define batteryStatus 19
#define ledPin        2
#define AD0_VAL       1
#define NUM_LEDS      1

#define pwm_freq       1000
#define pwm_resolution 14

// ============================================================================
// CONSTANTES
// ============================================================================

const int    maxPWMValue      = (1 << pwm_resolution) - 1;
const int    testPWM          = maxPWMValue * 0.35;   // ~35% para pruebas
const int    lowPWM           = maxPWMValue * 0.22;   // mínimo para mover
float        pulsesPerRev     = 574.0f;
const float  wheelCircumference = PI * 44.5f;
float        millimetersPerPulse = wheelCircumference / pulsesPerRev;
const float  centerToWheelDistance = 41.5f;

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================

volatile long leftPulseCount  = 0;
volatile long rightPulseCount = 0;
int pastLeft  = 0;
int pastRight = 0;

CRGB        leds[NUM_LEDS];
Servo       frontServo;
Adafruit_APDS9960 frontSensor;
ICM_20948_I2C imu;
Preferences preferences;

bool imuAvailable          = false;
bool frontSensorAvailable  = false;
int  currentTest           = -1;   // -1 = menú
unsigned long testStartMs  = 0;

// ============================================================================
// ISR — ENCODERS (idéntica lógica al código principal)
// ============================================================================

void IRAM_ATTR LeftWheelPulses() {
  int MSB = digitalRead(leftEncoderC2);
  int LSB = digitalRead(leftEncoderC1);
  int encoder = (MSB << 1) | LSB;
  int sum = (pastLeft << 2) | encoder;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    leftPulseCount++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    leftPulseCount--;
  pastLeft = encoder;
}

void IRAM_ATTR RightWheelPulses() {
  int MSB = digitalRead(rightEncoderC1);
  int LSB = digitalRead(rightEncoderC2);
  int encoder = (MSB << 1) | LSB;
  int sum = (pastRight << 2) | encoder;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    rightPulseCount++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    rightPulseCount--;
  pastRight = encoder;
}

// ============================================================================
// HELPERS DE LED
// ============================================================================

void ledSolid(uint8_t r, uint8_t g, uint8_t b) {
  leds[0] = CRGB(r, g, b);
  FastLED.setBrightness(120);
  FastLED.show();
}

void ledOff() {
  FastLED.setBrightness(0);
  FastLED.show();
}

// ============================================================================
// HELPERS DE MOTOR
// ============================================================================

void motorStop() {
  ledcWrite(leftMotorForward,  0);
  ledcWrite(leftMotorBackward, 0);
  ledcWrite(rightMotorForward,  0);
  ledcWrite(rightMotorBackward, 0);
}

void motorSet(int leftPWM, int rightPWM) {
  // Positivo = adelante, negativo = atrás
  if (leftPWM >= 0) {
    ledcWrite(leftMotorBackward, 0);
    ledcWrite(leftMotorForward,  leftPWM);
  } else {
    ledcWrite(leftMotorForward,  0);
    ledcWrite(leftMotorBackward, abs(leftPWM));
  }
  if (rightPWM >= 0) {
    ledcWrite(rightMotorBackward, 0);
    ledcWrite(rightMotorForward,  rightPWM);
  } else {
    ledcWrite(rightMotorForward,  0);
    ledcWrite(rightMotorBackward, abs(rightPWM));
  }
}

// ============================================================================
// HELPER: mover durante N milisegundos con pausa posterior
// ============================================================================

void motorRunMs(int leftPWM, int rightPWM, unsigned long ms, const char* label) {
  noInterrupts(); leftPulseCount = 0; rightPulseCount = 0; interrupts();
  Serial.printf("  → %s (izq=%d, der=%d) por %lums...\n", label, leftPWM, rightPWM, ms);
  motorSet(leftPWM, rightPWM);
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { delay(20); }
  motorStop();
  long L = leftPulseCount;
  long R = rightPulseCount;
  Serial.printf("     Pulsos: Izq=%ld  Der=%ld\n", L, R);
  Serial.printf("     Distancia aprox: Izq=%.1fmm  Der=%.1fmm\n",
                L * millimetersPerPulse, R * millimetersPerPulse);
  delay(400);
}

// ============================================================================
// IMPRESIÓN DE MENÚ
// ============================================================================

void printMenu() {
  Serial.println("\n========================================");
  Serial.println("   AttaBotTest — MENÚ DE DIAGNÓSTICO");
  Serial.println("========================================");
  Serial.printf("  PPR actual: %.2f  |  mm/pulso: %.4f\n",
                pulsesPerRev, millimetersPerPulse);
  Serial.printf("  IMU: %s  |  Sensor frontal: %s\n",
                imuAvailable ? "OK" : "NO DETECTADA",
                frontSensorAvailable ? "OK" : "NO DETECTADA");
  Serial.println("----------------------------------------");
  Serial.println("  1 → Test Motores");
  Serial.println("  2 → Test Encoders (tiempo real)");
  Serial.println("  3 → Test Sensores IR laterales");
  Serial.println("  4 → Test Sensor Frontal (APDS9960)");
  Serial.println("  5 → Test Servo");
  Serial.println("  6 → Test IMU (yaw tiempo real)");
  Serial.println("  7 → Test Batería");
  Serial.println("  8 → Calibración PPR");
  Serial.println("  9 → Test Completo (1→7)");
  Serial.println("  0 → Volver al menú");
  Serial.println("========================================");
  Serial.println("Escribe el número y presiona Enter:");
  ledSolid(0, 0, 255);   // Azul = esperando input
}

// ============================================================================
// TEST 1 — MOTORES
// ============================================================================

void testMotores() {
  Serial.println("\n[TEST 1] MOTORES");
  Serial.println("  Se moverá: adelante → atrás → giro izq → giro der");
  Serial.println("  Verifica que los movimientos sean correctos y simétricos.");
  delay(1500);

  ledSolid(0, 255, 0);
  motorRunMs( testPWM,  testPWM, 800, "ADELANTE");
  delay(300);

  ledSolid(255, 80, 0);
  motorRunMs(-testPWM, -testPWM, 800, "ATRAS");
  delay(300);

  ledSolid(255, 0, 255);
  motorRunMs(-testPWM,  testPWM, 700, "GIRO IZQUIERDA");
  delay(300);

  ledSolid(0, 255, 255);
  motorRunMs( testPWM, -testPWM, 700, "GIRO DERECHA");
  delay(300);

  // Prueba asimetría: solo motor izquierdo
  Serial.println("  → Solo motor IZQUIERDO adelante 600ms:");
  motorRunMs(testPWM, 0, 600, "SOLO IZQ");
  delay(300);

  // Solo motor derecho
  Serial.println("  → Solo motor DERECHO adelante 600ms:");
  motorRunMs(0, testPWM, 600, "SOLO DER");
  delay(300);

  ledSolid(0, 255, 0);
  Serial.println("[TEST 1] COMPLETADO ✓");
  Serial.println("  Verifica: ¿los movimientos fueron suaves y simétricos?");
  Serial.println("  Si detectas diferencia L/R en pulsos, ajusta el PPR con prueba 8.");
}

// ============================================================================
// TEST 2 — ENCODERS EN TIEMPO REAL
// ============================================================================

void testEncoders() {
  Serial.println("\n[TEST 2] ENCODERS — Tiempo real (15 segundos)");
  Serial.println("  Mueve el robot a mano o déjalo quieto para ver el conteo.");
  Serial.println("  El LED cambia de color según si hay pulsos.");

  noInterrupts(); leftPulseCount = 0; rightPulseCount = 0; interrupts();

  unsigned long t0 = millis();
  unsigned long lastPrint = 0;
  long prevL = 0, prevR = 0;

  while (millis() - t0 < 15000) {
    long L = leftPulseCount;
    long R = rightPulseCount;

    if (millis() - lastPrint > 300) {
      lastPrint = millis();
      long dL = L - prevL;
      long dR = R - prevR;
      prevL = L; prevR = R;

      float distL = L * millimetersPerPulse;
      float distR = R * millimetersPerPulse;

      Serial.printf("  L=%5ld (%.1fmm) | R=%5ld (%.1fmm) | dL=%ld dR=%ld\n",
                    L, distL, R, distR, dL, dR);

      if (abs(dL) > 0 || abs(dR) > 0)
        ledSolid(0, 255, 0);
      else
        ledSolid(20, 20, 80);
    }
    delay(20);
  }

  Serial.printf("\n  Pulsos totales: Izq=%ld | Der=%ld\n",
                leftPulseCount, rightPulseCount);
  Serial.println("[TEST 2] COMPLETADO ✓");
}

// ============================================================================
// TEST 3 — SENSORES IR LATERALES
// ============================================================================

void testSensoresIR() {
  Serial.println("\n[TEST 3] SENSORES IR LATERALES (15 segundos)");
  Serial.println("  Pasa la mano frente a cada sensor.");
  Serial.println("  LED ROJO = obstáculo | LED VERDE = libre");

  // Habilitar sensores
  digitalWrite(enableLeftInfraredSensor,  HIGH);
  digitalWrite(enableRightInfraredSensor, HIGH);
  delay(50);

  unsigned long t0      = millis();
  unsigned long lastPrint = 0;
  int leftCount = 0, rightCount = 0;
  bool prevLeft = false, prevRight = false;

  while (millis() - t0 < 15000) {
    bool leftDetected  = (digitalRead(leftInfraredSensor)  == LOW);
    bool rightDetected = (digitalRead(rightInfraredSensor) == LOW);

    if (leftDetected  && !prevLeft)  leftCount++;
    if (rightDetected && !prevRight) rightCount++;
    prevLeft  = leftDetected;
    prevRight = rightDetected;

    if (millis() - lastPrint > 200) {
      lastPrint = millis();
      Serial.printf("  IZQ: %s (%d)  |  DER: %s (%d)\n",
                    leftDetected  ? "OBSTÁCULO" : "libre",
                    leftCount,
                    rightDetected ? "OBSTÁCULO" : "libre",
                    rightCount);

      if (leftDetected && rightDetected)
        ledSolid(255, 0, 0);
      else if (leftDetected)
        ledSolid(255, 100, 0);
      else if (rightDetected)
        ledSolid(255, 0, 100);
      else
        ledSolid(0, 255, 0);
    }
    delay(20);
  }

  // Deshabilitar
  digitalWrite(enableLeftInfraredSensor,  LOW);
  digitalWrite(enableRightInfraredSensor, LOW);

  Serial.printf("\n  Detecciones: Izq=%d | Der=%d\n", leftCount, rightCount);
  Serial.println("[TEST 3] COMPLETADO ✓");
  Serial.println("  Si no detectó nada con la mano cerca, revisa el cableado/enable pin.");
}

// ============================================================================
// TEST 4 — SENSOR FRONTAL APDS9960
// ============================================================================

void testSensorFrontal() {
  Serial.println("\n[TEST 4] SENSOR FRONTAL APDS9960 (15 segundos)");

  if (!frontSensorAvailable) {
    Serial.println("  ⚠ Sensor no detectado en setup. Reintentando...");
    if (frontSensor.begin()) {
      frontSensorAvailable = true;
      Serial.println("  ✓ Sensor inicializado en este intento.");
    } else {
      Serial.println("  ✗ No se puede inicializar. Verifica I2C / Qwiic.");
      ledSolid(255, 0, 0);
      delay(2000);
      return;
    }
  }

  frontSensor.enableProximity(true);
  Serial.println("  Acerca y aleja la mano frente al sensor frontal.");

  unsigned long t0 = millis();
  unsigned long lastPrint = 0;
  int maxProx = 0;

  while (millis() - t0 < 15000) {
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      uint8_t prox = frontSensor.readProximity();
      if (prox > maxProx) maxProx = prox;

      // Barra visual en serial
      int bars = prox / 10;
      char bar[27]; memset(bar, '|', bars); bar[bars] = '\0';
      Serial.printf("  Proximidad: %3d  [%-26s]\n", prox, bar);

      // LED: verde=libre, amarillo=cerca, rojo=muy cerca
      if (prox < 5)
        ledSolid(0, 255, 0);
      else if (prox < 50)
        ledSolid(255, 200, 0);
      else
        ledSolid(255, 0, 0);
    }
    delay(50);
  }

  frontSensor.enableProximity(false);
  Serial.printf("\n  Proximidad máxima detectada: %d\n", maxProx);
  Serial.println("[TEST 4] COMPLETADO ✓");
}

// ============================================================================
// TEST 5 — SERVO
// ============================================================================

void testServo() {
  Serial.println("\n[TEST 5] SERVO FRONTAL");
  Serial.println("  Barrido: 90° → 0° → 180° → 90°");

  struct { int angle; const char* label; } positions[] = {
    {90,  "centro  (90°)"},
    {0,   "derecha  (0°)"},
    {90,  "centro  (90°)"},
    {180, "izquierda (180°)"},
    {90,  "centro  (90°)"}
  };

  for (auto& p : positions) {
    Serial.printf("  → Moviendo a %s\n", p.label);
    ledSolid(0, 180, 255);
    frontServo.write(p.angle);
    delay(1000);
  }

  Serial.println("[TEST 5] COMPLETADO ✓");
  Serial.println("  Verifica: ¿el servo llegó a las posiciones sin vibrar?");
}

// ============================================================================
// TEST 6 — IMU (YAW en tiempo real)
// ============================================================================

void testIMU() {
  Serial.println("\n[TEST 6] IMU ICM-20948 — Yaw en tiempo real (20 segundos)");

  if (!imuAvailable) {
    Serial.println("  ⚠ IMU no detectada en setup.");
    ledSolid(255, 0, 0);
    delay(2000);
    return;
  }

  unsigned long t0 = millis();
  unsigned long lastPrint = 0;

  while (millis() - t0 < 20000) {
    icm_20948_DMP_data_t data;
    imu.readDMPdataFromFIFO(&data);

    if ((imu.status == ICM_20948_Stat_Ok ||
         imu.status == ICM_20948_Stat_FIFOMoreDataAvail) &&
        (data.header & DMP_header_bitmap_Quat9)) {

      double q1 = data.Quat9.Data.Q1 / 1073741824.0;
      double q2 = data.Quat9.Data.Q2 / 1073741824.0;
      double q3 = data.Quat9.Data.Q3 / 1073741824.0;
      double q0 = sqrt(1.0 - (q1*q1 + q2*q2 + q3*q3));

      double t3 = 2.0 * (q0*q3 + q1*q2);
      double t4 = 1.0 - 2.0 * (q2*q2 + q3*q3);
      float yaw = fmod(-atan2(t3, t4) * RAD_TO_DEG + 450.0, 360.0);

      float gravity = 0;
      if (data.header & DMP_header_bitmap_Accel) {
        const float cf = 8192.0;
        float ax = data.Raw_Accel.Data.X / cf;
        float ay = data.Raw_Accel.Data.Y / cf;
        float az = data.Raw_Accel.Data.Z / cf;
        gravity = sqrt(ax*ax + ay*ay + az*az);
      }

      if (millis() - lastPrint > 200) {
        lastPrint = millis();
        Serial.printf("  Yaw: %6.1f°  |  Gravedad: %.3f g\n", yaw, gravity);
        // LED azul varía con el yaw
        uint8_t hue = (uint8_t)(yaw * 255.0f / 360.0f);
        leds[0] = CHSV(hue, 255, 200);
        FastLED.setBrightness(120);
        FastLED.show();
      }
      imu.resetFIFO();
    }
    delay(20);
  }

  Serial.println("[TEST 6] COMPLETADO ✓");
  Serial.println("  Verifica: ¿el yaw cambia al girar el robot y se estabiliza?");
}

// ============================================================================
// TEST 7 — BATERÍA
// ============================================================================

void testBateria() {
  Serial.println("\n[TEST 7] BATERÍA (5 segundos de lectura)");
  Serial.println("  Pin batteryStatus LOW = batería baja");

  unsigned long t0 = millis();
  int lowCount = 0;

  while (millis() - t0 < 5000) {
    bool low = (digitalRead(batteryStatus) == LOW);
    if (low) lowCount++;

    Serial.printf("  Estado: %s\n", low ? "⚠ BAJA" : "OK");
    if (low)
      ledSolid(255, 255, 0);
    else
      ledSolid(0, 255, 0);
    delay(500);
  }

  Serial.printf("\n  Lecturas LOW: %d / 10\n", lowCount);
  if (lowCount > 3)
    Serial.println("  ⚠ ADVERTENCIA: La batería puede estar baja.");
  else
    Serial.println("  Batería: nivel normal.");
  Serial.println("[TEST 7] COMPLETADO ✓");
}

// ============================================================================
// TEST 8 — CALIBRACIÓN PPR
// ============================================================================
// Mueve el robot exactamente 1 vuelta de rueda (circunferencia conocida),
// mide los pulsos y calcula el PPR real.

void calibracionPPR() {
  Serial.println("\n[TEST 8] CALIBRACIÓN PPR");
  Serial.println("  El robot avanzará a velocidad baja durante 2 segundos.");
  Serial.printf("  PPR actual: %.2f | mm/pulso: %.4f\n",
                pulsesPerRev, millimetersPerPulse);
  Serial.println("  Coloca el robot en línea recta. Presiona Enter para comenzar...");

  while (!Serial.available()) delay(50);
  while (Serial.available()) Serial.read();

  // Mover 2 segundos hacia adelante a velocidad baja
  noInterrupts(); leftPulseCount = 0; rightPulseCount = 0; interrupts();

  ledSolid(255, 180, 0);
  motorSet(lowPWM, lowPWM);
  delay(2000);
  motorStop();

  long L = leftPulseCount;
  long R = rightPulseCount;

  Serial.printf("\n  Pulsos medidos: Izq=%ld | Der=%ld\n", L, R);

  // Pide al usuario la distancia real recorrida
  Serial.println("  Mide con regla la distancia recorrida en mm y escríbela:");
  Serial.println("  (Escribe el número y presiona Enter)");

  while (!Serial.available()) delay(50);
  String input = Serial.readStringUntil('\n');
  input.trim();
  float distanciaReal = input.toFloat();

  if (distanciaReal < 10 || distanciaReal > 2000) {
    Serial.println("  ✗ Distancia inválida. Cancelando calibración.");
    return;
  }

  long avgPulses = (L + R) / 2;
  if (avgPulses < 10) {
    Serial.println("  ✗ Muy pocos pulsos. Verifica encoders con test 2.");
    return;
  }

  // Calcula mm/pulso y PPR nuevos
  float nuevoMmPorPulso = distanciaReal / avgPulses;
  float nuevoPPR        = wheelCircumference / nuevoMmPorPulso;

  Serial.printf("\n  Resultado:\n");
  Serial.printf("    Distancia real:    %.1f mm\n", distanciaReal);
  Serial.printf("    Pulsos promedio:   %ld\n", avgPulses);
  Serial.printf("    Nuevo mm/pulso:    %.4f\n", nuevoMmPorPulso);
  Serial.printf("    Nuevo PPR:         %.2f  (anterior: %.2f)\n",
                nuevoPPR, pulsesPerRev);

  Serial.println("\n  ¿Guardar nuevo PPR en flash? (s/n)");
  while (!Serial.available()) delay(50);
  String resp = Serial.readStringUntil('\n');
  resp.trim();
  resp.toLowerCase();

  if (resp == "s" || resp == "si" || resp == "y" || resp == "yes") {
    preferences.begin("attabot-config", false);
    preferences.putFloat("ppr", nuevoPPR);
    preferences.end();

    pulsesPerRev       = nuevoPPR;
    millimetersPerPulse = nuevoMmPorPulso;

    Serial.println("  ✓ PPR guardado en flash Preferences.");
    Serial.println("  El código principal cargará este valor automáticamente.");
    ledSolid(0, 255, 0);
  } else {
    Serial.println("  Cancelado. PPR no guardado.");
  }

  Serial.println("[TEST 8] COMPLETADO ✓");
}

// ============================================================================
// TEST 9 — SECUENCIA COMPLETA
// ============================================================================

void testCompleto() {
  Serial.println("\n[TEST 9] SECUENCIA COMPLETA");
  Serial.println("  Ejecutando tests 1→7 con pausa de 2s entre cada uno.");
  Serial.println("  Presiona Enter para comenzar...");
  while (!Serial.available()) delay(50);
  while (Serial.available()) Serial.read();

  testMotores();   delay(2000);
  testEncoders();  delay(2000);
  testSensoresIR();delay(2000);
  testSensorFrontal(); delay(2000);
  testServo();     delay(2000);
  testIMU();       delay(2000);
  testBateria();

  ledSolid(0, 255, 0);
  Serial.println("\n========================================");
  Serial.println("  TEST COMPLETO FINALIZADO");
  Serial.println("========================================");
}

// ============================================================================
// SETUP DE IMU (versión simplificada de setupIMU del código principal)
// ============================================================================

bool setupIMUTest() {
  imu.begin(Wire, AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.printf("  IMU no detectada (status=%d)\n", imu.status);
    return false;
  }

  bool ok = true;
  ok &= (imu.initializeDMP()                                             == ICM_20948_Stat_Ok);
  ok &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR)       == ICM_20948_Stat_Ok);
  ok &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER)         == ICM_20948_Stat_Ok);
  ok &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 1)                       == ICM_20948_Stat_Ok);
  ok &= (imu.setDMPODRrate(DMP_ODR_Reg_Accel, 1)                       == ICM_20948_Stat_Ok);
  ok &= (imu.enableFIFO()  == ICM_20948_Stat_Ok);
  ok &= (imu.enableDMP()   == ICM_20948_Stat_Ok);
  ok &= (imu.resetDMP()    == ICM_20948_Stat_Ok);
  ok &= (imu.resetFIFO()   == ICM_20948_Stat_Ok);

  return ok;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println("\n\n=== AttaBotTest — Iniciando ===");

  // PPR desde flash
  preferences.begin("attabot-config", true);
  float stored = preferences.getFloat("ppr", 0);
  preferences.end();
  if (stored > 100) {
    pulsesPerRev        = stored;
    millimetersPerPulse = wheelCircumference / pulsesPerRev;
    Serial.printf("PPR cargado desde flash: %.2f\n", pulsesPerRev);
  } else {
    Serial.printf("Usando PPR por defecto: %.2f\n", pulsesPerRev);
  }

  // Servo
  frontServo.setPeriodHertz(50);
  frontServo.attach(frontServoPin, 1000, 2000);
  frontServo.write(90);
  Serial.println("[OK] Servo");

  // PWM motores
  ledcAttach(leftMotorForward,  pwm_freq, pwm_resolution);
  ledcAttach(leftMotorBackward, pwm_freq, pwm_resolution);
  ledcAttach(rightMotorForward, pwm_freq, pwm_resolution);
  ledcAttach(rightMotorBackward,pwm_freq, pwm_resolution);
  motorStop();
  Serial.println("[OK] Motores PWM");

  // I2C
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("[OK] I2C");

  // LED
  FastLED.addLeds<WS2812, ledPin, GRB>(leds, NUM_LEDS);
  FastLED.setMaxRefreshRate(120);
  ledSolid(0, 0, 100);
  Serial.println("[OK] LED");

  // Encoders
  pinMode(leftEncoderC1,  INPUT_PULLUP);
  pinMode(leftEncoderC2,  INPUT);
  pinMode(rightEncoderC1, INPUT_PULLUP);
  pinMode(rightEncoderC2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC1),  LeftWheelPulses,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC2),  LeftWheelPulses,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC1), RightWheelPulses, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC2), RightWheelPulses, CHANGE);
  Serial.println("[OK] Encoders");

  // Sensores IR
  pinMode(enableLeftInfraredSensor,  OUTPUT);
  pinMode(enableRightInfraredSensor, OUTPUT);
  digitalWrite(enableLeftInfraredSensor,  LOW);
  digitalWrite(enableRightInfraredSensor, LOW);
  pinMode(leftInfraredSensor,  INPUT);
  pinMode(rightInfraredSensor, INPUT);
  Serial.println("[OK] Sensores IR");

  // Batería
  pinMode(batteryStatus, INPUT);
  Serial.println("[OK] Pin batería");

  // Sensor frontal APDS9960
  if (frontSensor.begin()) {
    frontSensorAvailable = true;
    Serial.println("[OK] APDS9960");
  } else {
    Serial.println("[--] APDS9960 no detectado");
  }

  // IMU
  Serial.println("Inicializando IMU...");
  if (setupIMUTest()) {
    imuAvailable = true;
    Serial.println("[OK] IMU ICM-20948");
  } else {
    Serial.println("[--] IMU no disponible");
  }

  Serial.println("\n=== Setup completo ===");
  printMenu();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) {
      printMenu();
      return;
    }

    int choice = input.toInt();

    switch (choice) {
      case 1: testMotores();       break;
      case 2: testEncoders();      break;
      case 3: testSensoresIR();    break;
      case 4: testSensorFrontal(); break;
      case 5: testServo();         break;
      case 6: testIMU();           break;
      case 7: testBateria();       break;
      case 8: calibracionPPR();    break;
      case 9: testCompleto();      break;
      case 0: break;
      default:
        Serial.printf("  Opción '%s' no reconocida.\n", input.c_str());
        break;
    }

    delay(500);
    printMenu();
  }

  // Latido lento para indicar que el sketch está corriendo
  static unsigned long lastHeartbeat = 0;
  static bool ledState = false;
  if (millis() - lastHeartbeat > 1200) {
    lastHeartbeat = millis();
    ledState = !ledState;
    if (ledState) ledSolid(0, 0, 200);
    else          ledOff();
  }
}
