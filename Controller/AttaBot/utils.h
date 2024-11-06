/***************************************************************************************

 Estructura que representa la pose de un robot en un espacio bidimensional. Contiene las 
 coordenadas x e y, así como el ángulo de orientación.

***************************************************************************************/
struct pose {
  float x, y, angle;


  /*************************************************************************************

   Constructor para inicializar los valores de la estructura pose.

   @param x     Coordenada X de la pose.
   @param y     Coordenada Y de la pose.
   @param angle Ángulo de orientación en radianes.

  *************************************************************************************/
  pose(float x, float y, float angle)
      : x(x), y(y), angle(angle) {}
};


/***************************************************************************************

 Estructura que representa los parámetros del controlador PID. Contiene las constantes 
 proporcional (kp), integral (ki) y derivativa (kd) utilizadas en el cálculo del control 
 PID.

***************************************************************************************/
struct pidConstants {
  float kp, ki, kd;


  /*************************************************************************************

   Constructor para inicializar los valores de las constantes PID.

   @param p Coeficiente proporcional.
   @param i Coeficiente integral.
   @param d Coeficiente derivativo.

  *************************************************************************************/
  pidConstants(float p, float i, float d)
      : kp(p), ki(i), kd(d) {}
};


/***************************************************************************************

 Estructura que representa un filtro de Kalman, utilizado para la estimación y 
 predicción de valores a partir de mediciones ruidosas. Incluye parámetros como R, H, Q, 
 así como valores internos como P, K y el valor predicho.

***************************************************************************************/
struct kalmanFilter{
  float R; // Valor R filtro de Kalman
  float H; // Valor de ganancia filtro de Kalman
  float Q; // Valor Q filtro de Kalman
  float P; // Valor de ajuste de Kalman
  float K; // Valor ganancia K de Kalman
  float predictedValue; // Valor predicho


  /*************************************************************************************

   Constructor que permite establecer valores para R, H y Q. Los valores de P, K y 
   predictedValue se inicializan a cero.

   @param r Valor R del filtro de Kalman.
   @param h Valor de ganancia del filtro de Kalman.
   @param q Valor Q del filtro de Kalman.

  *************************************************************************************/
  kalmanFilter(float r, float h, float q)
      : R(r), H(h), Q(q), P(0), K(0), predictedValue(0) {}


  /*************************************************************************************

   Método para restablecer los valores no constantes del filtro de Kalman a cero.

  *************************************************************************************/
  void Reset() {
    P = 0;
    K = 0;
    predictedValue = 0;
  }


  /*************************************************************************************

   Método para aplicar el filtro de Kalman.

   @param val Valor de entrada (medición actual).
   @return Valor estimado después de aplicar el filtro de Kalman.

  *************************************************************************************/
  float Calculate(float val) {
    K = (P * H) / (H * P * H + R); // Ganancia de Kalman
    predictedValue = predictedValue + K * (val - H * predictedValue);
    P = (1 - K * H) * P + Q; // Actualizar error covarianza
    return predictedValue;
  }
};


/***************************************************************************************

 Estructura para un controlador PID con filtro de Kalman. Controla la respuesta de salida 
 ajustando el ciclo de trabajo (PWM) en función de las constantes PID proporcionadas y 
 limita la salida en función de valores predefinidos. Incluye filtrado de Kalman para 
 suavizar la señal de salida.

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


  /*************************************************************************************
   Constructor que inicializa el controlador PID y establece las constantes de tiempo, 
   límites de ciclo de trabajo y reinicia valores de error.
   
   @param k          Filtro de Kalman para la salida del controlador.
   @param p          Estructura de constantes PID.
   @param tiempo     Tiempo de muestreo en segundos.
   @param limiteMin  Límite mínimo para el ciclo de trabajo.
   @param limiteMax  Límite máximo para el ciclo de trabajo.
  *************************************************************************************/
  pidController(kalmanFilter k, pidConstants p, float tiempo, float limiteMin, float limiteMax)
      : kf(k), pidConst(p), sumError(0), previousError(0), error(0), offsetSumError(0), samplingTime(tiempo),
        minWorkCycleLimit(limiteMin), maxWorkCycleLimit(limiteMax) {}



  /*************************************************************************************

   Restablece el error acumulado y el filtro de Kalman.

  *************************************************************************************/
  void Reset() {
    sumError = 0;
    previousError = 0;
    kf.Reset();
  }


  /*************************************************************************************

   Calcula la acción de control usando un controlador PID.

   @param reference: Valor de referencia deseado
   @param currentValue: Valor actual
   @return: Valor PWM después de aplicar la acción de control, limitado por el filtro 
            de Kalman y los valores máximos y mínimos permitidos
            
  *************************************************************************************/
  int Calculate(const float reference, const float currentValue) {
    error = reference - currentValue; // Se actualiza el error actual

    // Se actualiza el error integral y se restringe en un rango
    sumError += error * samplingTime;
    sumError = constrain(sumError, -maxWorkCycleLimit, maxWorkCycleLimit);

    // Ecuación de control PI
    float pidPwm = (pidConst.kp * error) + (pidConst.ki * sumError);

    // Error derivativo (diferencial)
    differentialError = (_abs(pidPwm) < minWorkCycleLimit) ? (error - previousError) / samplingTime : 0;

    // Ecuación de control D
    pidPwm += pidConst.kd * differentialError;

    // Se limita los valores máximos y mínimos de la acción de control
    pidPwm = kf.Calculate(pidPwm);
    pidPwm = constrain(pidPwm, -maxWorkCycleLimit, maxWorkCycleLimit);

    // Actualiza el valor del error para el siguiente ciclo
    previousError = error;
    return static_cast<int>(pidPwm);
  }
};


/***************************************************************************************

 Estructura que almacena los valores de bias (sesgo) para sensores de giroscopio, 
 acelerómetro y campo magnético (CPass), así como un encabezado y una suma de verificación. 

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