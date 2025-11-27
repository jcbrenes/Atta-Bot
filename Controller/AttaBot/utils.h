/***************************************************************************************
 * utils.h - VERSIÓN COMPLETA REFACTORIZADA
 * 
 * Este archivo contiene TODAS las estructuras de datos y clases auxiliares
 * que no dependen directamente de hardware (motores, sensores, WiFi)
 * 
 * Proyecto: AttaBot - Sistema de Robot Enjambre
 * Autor: [Tu nombre]
 * Fecha: 2025
 ***************************************************************************************/

#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <deque>
#include <map>
#include <array>

// ============================================================================
// ESTRUCTURAS DE DATOS BÁSICAS
// ============================================================================

/***************************************************************************************
 * Estructura que representa la pose de un robot en un espacio bidimensional.
 ***************************************************************************************/
struct pose {
  float x, y, angle;

  pose(float x, float y, float angle) : x(x), y(y), angle(angle) {}
};


/***************************************************************************************
 * Estructura que representa los parámetros del controlador PID.
 ***************************************************************************************/
struct pidConstants {
  float kp, ki, kd;

  pidConstants(float p, float i, float d) : kp(p), ki(i), kd(d) {}
};


/***************************************************************************************
 * Estructura que representa un filtro de Kalman.
 ***************************************************************************************/
struct kalmanFilter {
  float R; // Valor R filtro de Kalman
  float H; // Valor de ganancia filtro de Kalman
  float Q; // Valor Q filtro de Kalman
  float P; // Valor de ajuste de Kalman
  float K; // Valor ganancia K de Kalman
  float predictedValue; // Valor predicho

  kalmanFilter(float r, float h, float q)
      : R(r), H(h), Q(q), P(0), K(0), predictedValue(0) {}

  void Reset() {
    P = 0;
    K = 0;
    predictedValue = 0;
  }

  float Calculate(float val) {
    K = (P * H) / (H * P * H + R);
    predictedValue = predictedValue + K * (val - H * predictedValue);
    P = (1 - K * H) * P + Q;
    return predictedValue;
  }
};


/***************************************************************************************
 * Estructura para un controlador PID con filtro de Kalman.
 ***************************************************************************************/
struct pidController {
  kalmanFilter kf;
  pidConstants pidConst;
  const float samplingTime;
  const float minWorkCycleLimit;
  const float maxWorkCycleLimit;
  float offsetSumError;
  float sumError;
  float previousError;
  float error;
  float differentialError;

  pidController(kalmanFilter k, pidConstants p, float tiempo, float limiteMin, float limiteMax)
      : kf(k), pidConst(p), sumError(0), previousError(0), error(0), offsetSumError(0),
        samplingTime(tiempo), minWorkCycleLimit(limiteMin), maxWorkCycleLimit(limiteMax) {}

  void Reset() {
    sumError = 0;
    previousError = 0;
    kf.Reset();
  }

  int Calculate(const float reference, const float currentValue) {
    error = reference - currentValue;
    sumError += error * samplingTime;
    sumError = constrain(sumError, -maxWorkCycleLimit, maxWorkCycleLimit);
    float pidPwm = (pidConst.kp * error) + (pidConst.ki * sumError);
    differentialError = (abs(pidPwm) < minWorkCycleLimit) ? (error - previousError) / samplingTime : 0;
    pidPwm += pidConst.kd * differentialError;
    pidPwm = kf.Calculate(pidPwm);
    pidPwm = constrain(pidPwm, -maxWorkCycleLimit, maxWorkCycleLimit);
    previousError = error;
    return static_cast<int>(pidPwm);
  }
};


/***************************************************************************************
 * Estructura que almacena los valores de bias para sensores IMU.
 ***************************************************************************************/
struct biasStore {
  int32_t header = 0x42;
  int32_t biasGyroX = 0;
  int32_t biasGyroY = 0;
  int32_t biasGyroZ = 0;
  int32_t biasAccelX = 0;
  int32_t biasAccelY = 0;
  int32_t biasAccelZ = 0;
  int32_t biasCPassX = 0;
  int32_t biasCPassY = 0;
  int32_t biasCPassZ = 0;
  int32_t sum = 0;
};


// ============================================================================
// ENUMERACIONES
// ============================================================================

/***************************************************************************************
 * Enumeración: RobotState
 * Define todos los estados posibles de la máquina de estados del robot.
 ***************************************************************************************/
enum RobotState {
    WAIT = 0,
    READ_INSTRUCTION,
    MOVE,
    TURN,
    STOP,
    REVERSE,
    RANDOM_WALK,
    MESSAGE_BASE,
    IDENTIFY_OBSTACLE,
    ACTIVE_EVASION,
    REQUEST_POSITION,
    RESUME_AFTER_EVASION
};


// ============================================================================
// ESTRUCTURAS DE CONTROL Y NAVEGACIÓN
// ============================================================================

/***************************************************************************************
 * Estructura: NavigationTarget
 * 
 * Controla la navegación iterativa hacia un objetivo específico.
 * Sistema dinámico que avanza por segmentos hasta alcanzar el destino.
 ***************************************************************************************/
struct NavigationTarget {
    float targetX = 0;
    float targetY = 0;
    bool isActive = false;
    
    // Control de segmentos dinámico
    float segmentDistance = 250;
    float arrivalThreshold = 80;
    float minSegmentDistance = 50;
    float maxSegmentDistance = 350;
    
    // Contador de intentos
    int maxIterations = 50;
    int currentIteration = 0;
    
    // NUEVO: Detección de loops y progreso
    struct PositionHistory {
        float x, y;
        unsigned long timestamp;
    };
    std::deque<PositionHistory> positionHistory;  // Últimas 10 posiciones
    const int maxHistorySize = 10;
    float lastDistance = 999999.0;  // Distancia en iteración anterior
    int iterationsWithoutProgress = 0;
    const int maxIterationsWithoutProgress = 3;
    unsigned long navigationStartTime = 0;
    const unsigned long maxNavigationTime = 300000;  // 5 minutos
    
    void Reset() {
        isActive = false;
        currentIteration = 0;
        targetX = 0;
        targetY = 0;
        positionHistory.clear();
        lastDistance = 999999.0;
        iterationsWithoutProgress = 0;
        navigationStartTime = 0;
    }
    
    void StartNavigation() {
        isActive = true;
        currentIteration = 0;
        positionHistory.clear();
        lastDistance = 999999.0;
        iterationsWithoutProgress = 0;
        navigationStartTime = millis();
    }
    
    bool HasExceededMaxIterations() {
        return currentIteration >= maxIterations;
    }
    
    bool HasTimedOut() {
        return (millis() - navigationStartTime) > maxNavigationTime;
    }
    
    bool IsInLoop(float currentX, float currentY) {
        // Verificar si volvimos a una posición visitada recientemente
        const float loopThreshold = 100.0;  // 10cm
        
        for (const auto& pos : positionHistory) {
            float dx = currentX - pos.x;
            float dy = currentY - pos.y;
            float distance = sqrt(dx * dx + dy * dy);
            
            if (distance < loopThreshold) {
                unsigned long timeSince = millis() - pos.timestamp;
                // Si volvimos a una posición en menos de 30 segundos, es un loop
                if (timeSince < 30000) {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    void RecordPosition(float x, float y) {
        PositionHistory pos = {x, y, millis()};
        positionHistory.push_back(pos);
        
        if (positionHistory.size() > maxHistorySize) {
            positionHistory.pop_front();
        }
    }
    
    bool IsMakingProgress(float currentDistance) {
        const float progressThreshold = 20.0;  // 2cm mínimo de progreso
        
        if (currentDistance >= lastDistance - progressThreshold) {
            iterationsWithoutProgress++;
        } else {
            iterationsWithoutProgress = 0;
        }
        
        lastDistance = currentDistance;
        
        return iterationsWithoutProgress < maxIterationsWithoutProgress;
    }
    
    float GetDistanceToTarget(float currentX, float currentY) {
        float deltaX = targetX - currentX;
        float deltaY = targetY - currentY;
        return sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    
    bool HasReachedTarget(float currentX, float currentY) {
        return GetDistanceToTarget(currentX, currentY) < arrivalThreshold;
    }
};


/***************************************************************************************
 * Estructura: InterruptionContext
 * 
 * Almacena el contexto de movimiento cuando el robot es interrumpido por obstáculos.
 * Permite reanudar el movimiento después de evasión.
 ***************************************************************************************/
struct InterruptionContext {
    bool wasInterrupted = false;
    RobotState previousState = WAIT;
    float remainingValue = 0;
    int leftPulsesBeforeStop = 0;
    int rightPulsesBeforeStop = 0;
    
    void Clear() {
        wasInterrupted = false;
        previousState = WAIT;
        remainingValue = 0;
        leftPulsesBeforeStop = 0;
        rightPulsesBeforeStop = 0;
    }
    
    bool HasRemainingMovement(float minimumDistance = 20) {
        return wasInterrupted && abs(remainingValue) > minimumDistance;
    }
};


/***************************************************************************************
 * Estructura: EvasionTracker
 * 
 * Rastrea las evasiones consecutivas para detectar situaciones de bloqueo
 * y activar comportamientos de escape más agresivos.
 ***************************************************************************************/
struct EvasionTracker {
    int consecutiveEvasions = 0;
    unsigned long lastEvasionTime = 0;
    const int maxConsecutiveEvasions = 3;
    const unsigned long evasionResetTime = 5000;  // 5 segundos
    bool forceRetreat = false;
    
    void RecordEvasion() {
        unsigned long now = millis();
        
        // Si han pasado más de 5 segundos, resetear contador
        if (now - lastEvasionTime > evasionResetTime) {
            consecutiveEvasions = 0;
        }
        
        consecutiveEvasions++;
        lastEvasionTime = now;
        
        // Activar retroceso forzado si superamos el límite
        if (consecutiveEvasions >= maxConsecutiveEvasions) {
            forceRetreat = true;
        }
    }
    
    void Reset() {
        consecutiveEvasions = 0;
        forceRetreat = false;
        lastEvasionTime = millis();
    }
    
    bool ShouldRetreat() {
        return forceRetreat;
    }
    
    bool IsInCriticalState() {
        return consecutiveEvasions >= maxConsecutiveEvasions - 1;
    }
};


/***************************************************************************************
 * Estructura: CongregationState
 * 
 * Maneja el estado de congregación del robot.
 * Consolida todas las variables relacionadas con el comportamiento de congregación.
 ***************************************************************************************/
struct CongregationState {
    String leaderID = "-1";
    bool isLeader = false;
    bool positionReceived = false;
    bool hasGlobalTarget = false;
    float globalTargetX = 0;
    float globalTargetY = 0;
    unsigned long lastRequestTime = 0;
    bool waitingForResponse = false;
    const unsigned long requestTimeout = 5000;
    
    void Reset() {
        leaderID = "-1";
        isLeader = false;
        positionReceived = false;
        hasGlobalTarget = false;
        globalTargetX = 0;
        globalTargetY = 0;
        lastRequestTime = 0;
        waitingForResponse = false;
    }
    
    bool IsActive() {
        return leaderID != "-1";
    }
    
    bool HasTimedOut() {
        return waitingForResponse && (millis() - lastRequestTime > requestTimeout);
    }
    
    void StartRequest() {
        lastRequestTime = millis();
        waitingForResponse = true;
    }
    
    void CompleteRequest() {
        waitingForResponse = false;
        lastRequestTime = 0;
    }
};


/***************************************************************************************
 * Estructura: ObstacleState
 * 
 * Consolida el estado de todos los sensores de obstáculos.
 ***************************************************************************************/
struct ObstacleState {
    bool leftObstacle = false;
    bool centralObstacle = false;
    bool rightObstacle = false;
    bool robotDetected = false;
    String fromRobotID = "";
    int obstacleSensors = 0;  // Bitmap: [left][central][right]
    
    void Clear() {
        leftObstacle = false;
        centralObstacle = false;
        rightObstacle = false;
        robotDetected = false;
        fromRobotID = "";
        obstacleSensors = 0;
    }
    
    bool HasAnyObstacle() {
        return leftObstacle || centralObstacle || rightObstacle;
    }
    
    void UpdateBitmap() {
        obstacleSensors = (leftObstacle << 2) | (centralObstacle << 1) | rightObstacle;
    }
    
    bool IsFrontalObstacle() {
        return centralObstacle || (leftObstacle && rightObstacle);
    }
    
    String GetObstaclePattern() {
        if (obstacleSensors == 0b100) return "LEFT";
        if (obstacleSensors == 0b010) return "CENTER";
        if (obstacleSensors == 0b001) return "RIGHT";
        if (obstacleSensors == 0b110) return "LEFT+CENTER";
        if (obstacleSensors == 0b011) return "CENTER+RIGHT";
        if (obstacleSensors == 0b111) return "ALL";
        return "NONE";
    }
};


/***************************************************************************************
 * Estructura: MovementMetrics
 * 
 * Agrupa todas las métricas relacionadas con el movimiento del robot.
 ***************************************************************************************/
struct MovementMetrics {
    volatile int leftPulseCount = 0;
    volatile int rightPulseCount = 0;
    int pastLeftPulseCount = 0;
    int pastRightPulseCount = 0;
    float currentLeftSpeed = 0.0;
    float currentRightSpeed = 0.0;
    unsigned long previousMillis = 0;
    unsigned long steadyStatePreviousMillis = 0;
    
    void Reset() {
        leftPulseCount = 0;
        rightPulseCount = 0;
        pastLeftPulseCount = 0;
        pastRightPulseCount = 0;
        currentLeftSpeed = 0.0;
        currentRightSpeed = 0.0;
    }
    
    float GetAverageSpeed() {
        return (currentLeftSpeed + currentRightSpeed) / 2.0;
    }
    
    float GetAverageDistance(float mmPerPulse) {
        return ((pastLeftPulseCount + pastRightPulseCount) / 2.0) * mmPerPulse;
    }
    
    bool IsStationary() {
        return abs(currentLeftSpeed) < 0.1 && abs(currentRightSpeed) < 0.1;
    }
};


/***************************************************************************************
 * Estructura: LedController
 * 
 * Control no bloqueante de LEDs WS2812.
 * Permite animaciones sin usar delay().
 ***************************************************************************************/
struct LedController {
    enum State { OFF, SOLID, BLINKING };
    State currentState = OFF;
    uint8_t red = 0, green = 0, blue = 0, brightness = 0;
    unsigned long lastUpdate = 0, interval = 500;
    bool blinkState = false;
    
    void setSolid(uint8_t r, uint8_t g, uint8_t b, uint8_t bright = 255) {
        red = r;
        green = g;
        blue = b;
        brightness = bright;
        currentState = SOLID;
    }
    
    void setBlink(uint8_t r, uint8_t g, uint8_t b, uint8_t bright = 255, unsigned long intervalMs = 250) {
        red = r;
        green = g;
        blue = b;
        brightness = bright;
        interval = intervalMs;
        currentState = BLINKING;
        lastUpdate = 0;
    }
    
    void setOff() {
        currentState = OFF;
    }
    
    bool IsBlinking() {
        return currentState == BLINKING;
    }
    
    bool IsSolid() {
        return currentState == SOLID;
    }

    void update();
};


// ============================================================================
// FUNCIONES AUXILIARES INLINE
// ============================================================================

/***************************************************************************************
 * Normaliza ángulos al rango [-180, 180]
 ***************************************************************************************/
inline float NormalizeAngle(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

/***************************************************************************************
 * Calcula distancia euclidiana entre dos puntos
 ***************************************************************************************/
inline float CalculateDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

/***************************************************************************************
 * Calcula ángulo hacia un objetivo
 ***************************************************************************************/
inline float CalculateAngleToTarget(float x1, float y1, float x2, float y2) {
    return atan2(y2 - y1, x2 - x1) * RAD_TO_DEG;
}

/***************************************************************************************
 * Verifica si un valor está en un rango
 ***************************************************************************************/
inline bool InRange(float value, float min, float max) {
    return value >= min && value <= max;
}

#endif // UTILS_H