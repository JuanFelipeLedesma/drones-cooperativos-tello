/*
Código funcional !!!

Doble núcleo con vuelo estable y lectura del sensor

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
// RANGO REAL FUNCIONAL
// ======================================================
const uint8_t PWM_MIN = 100;
const uint8_t PWM_MAX = 200;

// ======================================================
// CAPTURA PWM
// ======================================================
volatile uint32_t riseTime[4]   = {0,0,0,0};
volatile uint16_t pulseWidth[4] = {1000,1000,1000,1000};
volatile uint32_t lastEdge[4]   = {0,0,0,0};

// ======================================================
// SENSORES
// ======================================================
Adafruit_VL53L0X lox;
VL53L0X_RangingMeasurementData_t measure;
volatile uint16_t distanceMM = 0;
volatile bool distanceValid = false;

portMUX_TYPE distMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t sensorTaskHandle = nullptr;

// ======================================================
// CONVERSIÓN FC -> analogWrite
// 1000us -> 100
// 2000us -> 255
// ======================================================
uint8_t pwmToAnalog(uint16_t us){
    if(us < 1000) us = 1000;
    if(us > 2000) us = 2000;

    return map(us, 1000, 2000, PWM_MIN, PWM_MAX);
}

// ======================================================
// ESCRITURA MOTOR
// ======================================================
void writeMotor(uint8_t pin, uint16_t us){
    analogWrite(pin, pwmToAnalog(us));
}

// ======================================================
// SENSOR TASK
// ======================================================
void sensorTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        lox.rangingTest(&measure, false);

        portENTER_CRITICAL(&distMux);
        if (measure.RangeStatus != 4) {
            distanceMM = measure.RangeMilliMeter;
            distanceValid = true;
        } else {
            distanceMM = 0;
            distanceValid = false;
        }
        portEXIT_CRITICAL(&distMux);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ======================================================
// ISR M1
// ======================================================
void IRAM_ATTR isrM1(){
    uint32_t now = micros();

    if(digitalRead(FC_M1))
    {
        riseTime[0] = now;
    }
    else
    {
        uint32_t width = now - riseTime[0];

        if(width >= 800 && width <= 2200)
        {
            pulseWidth[0] = width;
            lastEdge[0] = now;
        }
    }
}

// ======================================================
// ISR M2
// ======================================================
void IRAM_ATTR isrM2(){
    uint32_t now = micros();

    if(digitalRead(FC_M2))
    {
        riseTime[1] = now;
    }
    else
    {
        uint32_t width = now - riseTime[1];

        if(width >= 800 && width <= 2200)
        {
            pulseWidth[1] = width;
            lastEdge[1] = now;
        }
    }
}

// ======================================================
// ISR M3
// ======================================================
void IRAM_ATTR isrM3(){
    uint32_t now = micros();

    if(digitalRead(FC_M3))
    {
        riseTime[2] = now;
    }
    else
    {
        uint32_t width = now - riseTime[2];

        if(width >= 800 && width <= 2200)
        {
            pulseWidth[2] = width;
            lastEdge[2] = now;
        }
    }
}

// ======================================================
// ISR M4
// ======================================================
void IRAM_ATTR isrM4(){
    uint32_t now = micros();

    if(digitalRead(FC_M4))
    {
        riseTime[3] = now;
    }
    else
    {
        uint32_t width = now - riseTime[3];

        if(width >= 800 && width <= 2200)
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

    if (!lox.begin()) {
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
void loop(){
    static uint32_t lastPrint = 0;

    uint16_t m1, m2, m3, m4;

    noInterrupts();
    m1 = pulseWidth[0];
    m2 = pulseWidth[1];
    m3 = pulseWidth[2];
    m4 = pulseWidth[3];
    interrupts();

    // FAILSAFE SIMPLE
    uint32_t now = micros();

    if(now - lastEdge[0] > 10000) m1 = 1000;
    if(now - lastEdge[1] > 10000) m2 = 1000;
    if(now - lastEdge[2] > 10000) m3 = 1000;
    if(now - lastEdge[3] > 10000) m4 = 1000;

    // ESCRIBIR MOTORES
    writeMotor(ESC_M1, m1);
    writeMotor(ESC_M2, m2);
    writeMotor(ESC_M3, m3);
    writeMotor(ESC_M4, m4);

    // DEBUG
    if(millis() - lastPrint > 200)
    {
        lastPrint = millis();

        uint16_t dmm;
        bool dvalid;

        portENTER_CRITICAL(&distMux);
        dmm = distanceMM;
        dvalid = distanceValid;
        portEXIT_CRITICAL(&distMux);

        Serial.print("M1:");
        Serial.print(pwmToAnalog(m1));

        Serial.print(" M2:");
        Serial.print(pwmToAnalog(m2));

        Serial.print(" M3:");
        Serial.print(pwmToAnalog(m3));

        Serial.print(" M4:");
        Serial.print(pwmToAnalog(m4));

        Serial.print(" | Dist:");
        if (dvalid) {
            Serial.print(dmm);
            Serial.print(" mm");
        } else {
            Serial.print(" out");
        }

        Serial.println();
    }

    delayMicroseconds(20);
}