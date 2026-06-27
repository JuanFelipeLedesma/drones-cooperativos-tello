/*
Prueba -> Si esta es la última cargada en el drone

A la espera de prueba en vuelo !!!

Doble núcleo con vuelo estable y lectura del sensor
+ Promedio móvil de altura
+ PID de altura activado al cruzar 50 cm por primera vez
+ LED GPIO21 encendido mientras el PID está activo
+ PID más suave:
  - trabaja en cm
  - usa solo altura filtrada
  - zona muerta
  - salida suavizada
  - límite de cambio por muestra
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ======================================================
// PWM OUTPUT
// ======================================================
const uint32_t PWM_FREQ = 400;

// ======================================================
// ENTRADAS FC
// ======================================================
const uint8_t FC_M1 = D0;
const uint8_t FC_M2 = D1;
const uint8_t FC_M3 = D2;
const uint8_t FC_M4 = D3;

// ======================================================
// SALIDAS GPIO HACIA ESC
// ======================================================
const uint8_t ESC_M1 = 7;
const uint8_t ESC_M2 = 8;
const uint8_t ESC_M3 = 9;
const uint8_t ESC_M4 = 41;

// ======================================================
// LED DE ESTADO
// ======================================================
const uint8_t STATUS_LED = 21;

// ======================================================
// RANGO REAL FUNCIONAL
// ======================================================
const uint8_t PWM_MIN = 100;
const uint8_t PWM_MAX = 200;

// ======================================================
// CAPTURA PWM
// ======================================================
volatile uint32_t riseTime[4]   = {0, 0, 0, 0};
volatile uint16_t pulseWidth[4] = {1000, 1000, 1000, 1000};
volatile uint32_t lastEdge[4]   = {0, 0, 0, 0};

// ======================================================
// SENSORES
// ======================================================
Adafruit_VL53L0X lox;
VL53L0X_RangingMeasurementData_t measure;

volatile uint16_t distanceRawMM = 0;
volatile uint16_t distanceFilteredMM = 0;
volatile float distanceFilteredCM = 0.0f;
volatile bool distanceValid = false;

portMUX_TYPE distMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t sensorTaskHandle = nullptr;

// ======================================================
// PROMEDIO MÓVIL
// ======================================================
const uint8_t DIST_WINDOW = 20;
uint16_t distBuffer[DIST_WINDOW] = {0};
uint32_t distSum = 0;
uint8_t distIndex = 0;
uint8_t distCount = 0;

// ======================================================
// PID DE ALTURA
// ======================================================
const float ALT_TARGET_CM = 50.0f;       // 50 cm
const uint32_t PID_DURATION_MS = 30000;  // 30 s

float Kp = 1.00f;
float Ki = 0.015f;
float Kd = 0.12f;

float Rango = 50.0f;

const float MOTOR_CORR_GAIN_UP   = 3.0f;
const float MOTOR_CORR_GAIN_DOWN = 3.0f;
const float MOTOR_CORR_LIMIT_US  = 180.0f;

const float CORR_SMOOTH_ALPHA_UP   = 0.18f;
const float CORR_SMOOTH_ALPHA_DOWN = 0.45f;

const float MAX_CORR_STEP_UP_US   = 5.0f;
const float MAX_CORR_STEP_DOWN_US = 12.0f;

const float ALT_DEADBAND_CM = 0.8f;

// Corrección aplicada al throttle general
volatile float altitudeCorrectionUs = 0.0f;
volatile bool altitudePidActive = false;
volatile bool altitudePidDone = false;
volatile uint32_t altitudePidStartMs = 0;

// ======================================================
// CONVERSIÓN FC -> analogWrite
// 1000us -> 100
// 2000us -> 200
// ======================================================
uint8_t pwmToAnalog(uint16_t us)
{
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;

    return map(us, 1000, 2000, PWM_MIN, PWM_MAX);
}

// ======================================================
// ESCRITURA MOTOR
// ======================================================
void writeMotor(uint8_t pin, uint16_t us)
{
    analogWrite(pin, pwmToAnalog(us));
}

// ======================================================
// SENSOR TASK
// ======================================================
void sensorTask(void *pvParameters)
{
    (void)pvParameters;

    const uint32_t SENSOR_PERIOD_MS = 10;
    const uint32_t PID_UPDATE_MS = 30;

    uint32_t lastSensorRead = 0;
    uint32_t lastPidUpdate = 0;

    static float integral = 0.0f;
    static float prevErrorCm = 0.0f;
    static float filteredCorrectionUs = 0.0f;
    static float prevFilteredCm = 0.0f;

    for (;;)
    {
        if (millis() - lastSensorRead >= SENSOR_PERIOD_MS)
        {
            lastSensorRead = millis();

            lox.rangingTest(&measure, false);

            bool valid = (measure.RangeStatus != 4) &&
                         (measure.RangeMilliMeter != 65535) &&
                         (measure.RangeMilliMeter >= 50) &&
                         (measure.RangeMilliMeter <= 2000);

            if (valid)
            {
                uint16_t newDist = measure.RangeMilliMeter;

                // Guardar lectura cruda
                portENTER_CRITICAL(&distMux);
                distanceRawMM = newDist;
                portEXIT_CRITICAL(&distMux);

                // Actualizar promedio móvil
                if (distCount < DIST_WINDOW)
                {
                    distBuffer[distIndex] = newDist;
                    distSum += newDist;
                    distCount++;
                }
                else
                {
                    distSum -= distBuffer[distIndex];
                    distBuffer[distIndex] = newDist;
                    distSum += newDist;
                }

                distIndex++;
                if (distIndex >= DIST_WINDOW) distIndex = 0;

                uint16_t avgMM = distSum / distCount;
                float avgCM = avgMM / 10.0f;

                portENTER_CRITICAL(&distMux);
                distanceFilteredMM = avgMM;
                distanceFilteredCM = avgCM;
                distanceValid = true;
                portEXIT_CRITICAL(&distMux);

                // ==================================================
                // Activación del PID:
                // Se arma solo cuando la altura filtrada cruza 50 cm
                // por primera vez desde abajo hacia arriba.
                // ==================================================
                if (!altitudePidActive && !altitudePidDone)
                {
                    if (prevFilteredCm < ALT_TARGET_CM && avgCM >= ALT_TARGET_CM)
                    {
                        altitudePidActive = true;
                        altitudePidStartMs = millis();
                        integral = 0.0f;
                        prevErrorCm = 0.0f;
                        filteredCorrectionUs = 0.0f;
                        altitudeCorrectionUs = 0.0f;

                        digitalWrite(STATUS_LED, HIGH);
                    }
                }

                // ==================================================
                // PID activo por 30 s
                // ==================================================
                if (altitudePidActive && (millis() - lastPidUpdate >= PID_UPDATE_MS))
                {
                    lastPidUpdate = millis();

                    float dt = PID_UPDATE_MS / 1000.0f;
                    float errorCm = ALT_TARGET_CM - avgCM;

                    // Zona muerta pequeña para evitar micro-oscilaciones
                    if (errorCm > -ALT_DEADBAND_CM && errorCm < ALT_DEADBAND_CM)
                    {
                        errorCm = 0.0f;
                    }

                    integral += errorCm * dt;

                    // anti-windup
                    if (integral > 300.0f) integral = 300.0f;
                    if (integral < -300.0f) integral = -300.0f;

                    float derivative = (errorCm - prevErrorCm) / dt;

                    float rawOutputUs = (Kp * errorCm) +
                    (Ki * integral) +
                    (Kd * derivative);

                    // Limitar corrección máxima del PID
                    if (rawOutputUs > Rango) rawOutputUs = Rango;
                    if (rawOutputUs < -Rango) rawOutputUs = -Rango;

                    // Suavizado asimétrico:
                    // cuando necesita bajar RPM (corrección negativa), responde más rápido
                    float alpha = (rawOutputUs < filteredCorrectionUs) ? CORR_SMOOTH_ALPHA_DOWN
                                                                    : CORR_SMOOTH_ALPHA_UP;
                    filteredCorrectionUs += (rawOutputUs - filteredCorrectionUs) * alpha;

                    // Límite de cambio asimétrico:
                    // bajar RPM más rápido que subirlas
                    float step = filteredCorrectionUs - altitudeCorrectionUs;
                    if (step > MAX_CORR_STEP_UP_US) step = MAX_CORR_STEP_UP_US;
                    if (step < -MAX_CORR_STEP_DOWN_US) step = -MAX_CORR_STEP_DOWN_US;

                    altitudeCorrectionUs += step;

                    prevErrorCm = errorCm;
                }
                else if (!altitudePidActive)
                {
                    altitudeCorrectionUs = 0.0f;
                }

                prevFilteredCm = avgCM;
            }
            else
            {
                portENTER_CRITICAL(&distMux);
                distanceValid = false;
                altitudeCorrectionUs = 0.0f;
                portEXIT_CRITICAL(&distMux);
            }

            // ==================================================
            // Desactivación automática después de 30 s
            // ==================================================
            if (altitudePidActive && (millis() - altitudePidStartMs >= PID_DURATION_MS))
            {
                altitudePidActive = false;
                altitudePidDone = true;
                altitudeCorrectionUs = 0.0f;
                digitalWrite(STATUS_LED, LOW);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ======================================================
// ISR M1
// ======================================================
void IRAM_ATTR isrM1()
{
    uint32_t now = micros();

    if (digitalRead(FC_M1))
    {
        riseTime[0] = now;
    }
    else
    {
        uint32_t width = now - riseTime[0];

        if (width >= 800 && width <= 2200)
        {
            pulseWidth[0] = width;
            lastEdge[0] = now;
        }
    }
}

// ======================================================
// ISR M2
// ======================================================
void IRAM_ATTR isrM2()
{
    uint32_t now = micros();

    if (digitalRead(FC_M2))
    {
        riseTime[1] = now;
    }
    else
    {
        uint32_t width = now - riseTime[1];

        if (width >= 800 && width <= 2200)
        {
            pulseWidth[1] = width;
            lastEdge[1] = now;
        }
    }
}

// ======================================================
// ISR M3
// ======================================================
void IRAM_ATTR isrM3()
{
    uint32_t now = micros();

    if (digitalRead(FC_M3))
    {
        riseTime[2] = now;
    }
    else
    {
        uint32_t width = now - riseTime[2];

        if (width >= 800 && width <= 2200)
        {
            pulseWidth[2] = width;
            lastEdge[2] = now;
        }
    }
}

// ======================================================
// ISR M4
// ======================================================
void IRAM_ATTR isrM4()
{
    uint32_t now = micros();

    if (digitalRead(FC_M4))
    {
        riseTime[3] = now;
    }
    else
    {
        uint32_t width = now - riseTime[3];

        if (width >= 800 && width <= 2200)
        {
            pulseWidth[3] = width;
            lastEdge[3] = now;
        }
    }
}

// ======================================================
// SETUP
// ======================================================
void setup()
{
    Serial.begin(115200);

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    // INPUTS FC
    pinMode(FC_M1, INPUT);
    pinMode(FC_M2, INPUT);
    pinMode(FC_M3, INPUT);
    pinMode(FC_M4, INPUT);

    // INTERRUPTS
    attachInterrupt(digitalPinToInterrupt(FC_M1), isrM1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FC_M2), isrM2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FC_M3), isrM3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FC_M4), isrM4, CHANGE);

    // PWM OUTPUTS
    analogWriteFrequency(ESC_M1, PWM_FREQ);
    analogWriteFrequency(ESC_M2, PWM_FREQ);
    analogWriteFrequency(ESC_M3, PWM_FREQ);
    analogWriteFrequency(ESC_M4, PWM_FREQ);

    // ARM ESC
    analogWrite(ESC_M1, PWM_MIN);
    analogWrite(ESC_M2, PWM_MIN);
    analogWrite(ESC_M3, PWM_MIN);
    analogWrite(ESC_M4, PWM_MIN);

    // I2C + VL53L0X
    Wire.begin();
    Wire.setClock(400000);

    if (!lox.begin())
    {
        Serial.println("ERROR: VL53L0X no responde.");
        while (true) { delay(1000); }
    }

    xTaskCreatePinnedToCore(
        sensorTask,
        "sensorTask",
        4096,
        nullptr,
        1,
        &sensorTaskHandle,
        0
    );

    Serial.println("Armando ESC...");
    delay(5000);

    Serial.println("Sistema iniciado");
}

// ======================================================
// LOOP
// ======================================================
void loop()
{
    static uint32_t lastPrint = 0;

    uint16_t m1, m2, m3, m4;
    bool pidState;
    bool dvalid;
    uint16_t dRaw, dFilt;
    float dFiltCm;
    float corrUs = 0.0f;
    float corrAppliedUs = 0.0f;

    noInterrupts();
    m1 = pulseWidth[0];
    m2 = pulseWidth[1];
    m3 = pulseWidth[2];
    m4 = pulseWidth[3];
    interrupts();

    uint32_t now = micros();

    if (now - lastEdge[0] > 10000) m1 = 1000;
    if (now - lastEdge[1] > 10000) m2 = 1000;
    if (now - lastEdge[2] > 10000) m3 = 1000;
    if (now - lastEdge[3] > 10000) m4 = 1000;

    portENTER_CRITICAL(&distMux);
    corrUs = altitudeCorrectionUs;
    pidState = altitudePidActive;
    dvalid = distanceValid;
    dRaw = distanceRawMM;
    dFilt = distanceFilteredMM;
    dFiltCm = distanceFilteredCM;
    portEXIT_CRITICAL(&distMux);

    // Aplicar corrección colectiva SOLO si el PID está activo
    if (pidState){

        float motorGain = (corrUs >= 0.0f) ? MOTOR_CORR_GAIN_UP : MOTOR_CORR_GAIN_DOWN;
        corrAppliedUs = corrUs * motorGain;

        if (corrAppliedUs > MOTOR_CORR_LIMIT_US)
            corrAppliedUs = MOTOR_CORR_LIMIT_US;

        if (corrAppliedUs < -MOTOR_CORR_LIMIT_US)
            corrAppliedUs = -MOTOR_CORR_LIMIT_US;

        int16_t corrInt =
            (int16_t)(corrAppliedUs >= 0.0f ?
            corrAppliedUs + 0.5f :
            corrAppliedUs - 0.5f);

        m1 = constrain((int32_t)m1 + corrInt, 1000, 2000);
        m2 = constrain((int32_t)m2 + corrInt, 1000, 2000);
        m3 = constrain((int32_t)m3 + corrInt, 1000, 2000);
        m4 = constrain((int32_t)m4 + corrInt, 1000, 2000);
    }

    writeMotor(ESC_M1, m1);
    writeMotor(ESC_M2, m2);
    writeMotor(ESC_M3, m3);
    writeMotor(ESC_M4, m4);

    if (millis() - lastPrint > 50)
    {
        lastPrint = millis();

        Serial.print("M1:");
        Serial.print(pwmToAnalog(m1));

        Serial.print(" M2:");
        Serial.print(pwmToAnalog(m2));

        Serial.print(" M3:");
        Serial.print(pwmToAnalog(m3));

        Serial.print(" M4:");
        Serial.print(pwmToAnalog(m4));

        Serial.print(" | DistCm:");
        Serial.print(dFiltCm, 1);

        Serial.print(" | PID:");
        Serial.print(pidState ? "ON" : "OFF");

        Serial.print(" | CorrUs:");
        Serial.print(corrUs, 1);

        Serial.print(" | CorrMotor:");
        Serial.print(corrAppliedUs, 1);

        Serial.print(" | LED:");
        Serial.print(digitalRead(STATUS_LED) ? "ON" : "OFF");

        Serial.println();
    }

    delayMicroseconds(20);
}