/*
 * High-Speed Balancing Robot for ESP32
 * Original: Wouter Klop
 * License: CC BY-SA 4.0 (acknowledge original author and publish modifications)
 */

#include <Arduino.h>
#include <FlySkyIBus.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Streaming.h>
#include <MPU6050.h>
#include <PID.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>
#include <fastStepper.h>
#include <Preferences.h>
#include <Ps3Controller.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

// ---------------- Input Configuration ----------------
#define INPUT_IBUS       // FlySky IBUS receiver
#define INPUT_PS3        // PS3 controller over Bluetooth
#define STEPPER_DRIVER_A4988

float speedFactor = 0.7;
float steerFactor = 1.0;
float speedFilterConstant = 0.9;
float steerFilterConstant = 0.9;

// ---------------- Type Definitions ----------------
typedef union {
    struct { float val; uint8_t cmd; uint8_t checksum; };
    uint8_t array[6];
} command;

typedef union {
    uint8_t arr[6];
    struct {
        uint8_t grp;
        uint8_t cmd;
        union { float val; uint8_t valU8[4]; };
    } __attribute__((packed));
} cmd;

struct {
    boolean enable = 0;
    uint8_t prescaler = 4;
} plot;

struct {
    float speed = 0;
    float steer = 0;
    float speedGain = 0.7;
    float steerGain = 0.6;
    float speedOffset = 0.0;
    bool selfRight = 0;
    bool disableControl = 0;
    bool override = 0;
} remoteControl;

#define FORMAT_SPIFFS_IF_FAILED true

// ---------------- Pin Definitions ----------------
#define PIN_LED         32
#define PIN_LED_LEFT    33
#define PIN_LED_RIGHT   26
#define PIN_MOTOR_CURRENT 25

#define motEnablePin    27
#define motUStepPin1    14
#define motUStepPin2    12
#define motUStepPin3    13

// ---------------- Stepper Motors ----------------
fastStepper motLeft(5, 4, 0, [](){});
fastStepper motRight(2, 15, 1, [](){});
uint8_t microStep = 16;
uint8_t motorCurrent = 150;
float maxStepSpeed = 1500;

// ---------------- PID Controllers ----------------
#define dT_MICROSECONDS 5000
#define dT dT_MICROSECONDS/1000000.0

#define PID_ANGLE 0
#define PID_POS 1
#define PID_SPEED 2

#define PID_ANGLE_MAX 12
PID pidAngle(cPID, dT, PID_ANGLE_MAX, -PID_ANGLE_MAX);
#define PID_POS_MAX 35
PID pidPos(cPD, dT, PID_POS_MAX, -PID_POS_MAX);
PID pidSpeed(cP, dT, PID_POS_MAX, -PID_POS_MAX);

uint8_t controlMode = 1;

// ---------------- IMU ----------------
MPU6050 imu;
#define GYRO_SENSITIVITY 65.5
int16_t gyroOffset[3];
float accAngle = 0;
float filterAngle = 0;
float angleOffset = 2.0;
float gyroFilterConstant = 0.996;
float gyroGain = 1.0;

// ---------------- WiFi & Web ----------------
const char* http_username = "admin";
const char* http_password = "admin";
AsyncWebServer httpServer(80);
WebSocketsServer wsServer(81);
Preferences preferences;

char robotName[63] = "balancingrobot";
char BTaddress[20] = "00:00:00:00:00:00";

// ---------------- ADC ----------------
#define ADC_CHANNEL_BATTERY_VOLTAGE ADC1_CHANNEL_6
#define BATTERY_VOLTAGE_SCALING_FACTOR (100+3.3)/3.3
#define BATTERY_VOLTAGE_FILTER_COEFFICIENT 0.99
esp_adc_cal_characteristics_t adc_chars;

// ---------------- Function Prototypes ----------------
void setMotorCurrent();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void readSensor();
void parseSerial();
void parseCommand(char* data, uint8_t length);
void calculateGyroOffset(uint8_t nSample);
void sendWifiList();

// ---------------- Setup ----------------
void setup() {
    Serial.begin(115200);

    #ifdef INPUT_IBUS
    IBus.begin(Serial2);
    #endif

    preferences.begin("settings", false);

    pinMode(motEnablePin, OUTPUT);
    pinMode(motUStepPin1, OUTPUT);
    pinMode(motUStepPin2, OUTPUT);
    pinMode(motUStepPin3, OUTPUT);
    digitalWrite(motEnablePin, 1); // Disable motors

    motLeft.init();
    motRight.init();
    motLeft.microStep = microStep;
    motRight.microStep = microStep;

    // LEDs
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_LED_LEFT, OUTPUT);
    pinMode(PIN_LED_RIGHT, OUTPUT);
    digitalWrite(PIN_LED_LEFT, 1);

    // SPIFFS
    if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
        Serial.println("SPIFFS mount failed");
        return;
    }

    // IMU
    Wire.begin(21, 22, 400000UL);
    imu.initialize();
    imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    initSensor(50);

    // WiFi/AP setup omitted for brevity
    // OTA setup omitted for brevity

    pidAngle.setParameters(0.65,1.0,0.075,15);
    pidPos.setParameters(1,0,1.2,50);
    pidSpeed.setParameters(6,5,0,20);

    setMotorCurrent();
    Serial.println("Ready");
}

// ---------------- Main Loop ----------------
void loop() {
    static unsigned long tLast = 0;
    unsigned long tNow = micros();

    if (tNow - tLast > dT_MICROSECONDS) {
        readSensor();
        parseSerial();
        motLeft.update();
        motRight.update();
        tLast = tNow;
    }
}

// ---------------- Utility Functions ----------------
void setMotorCurrent() {
    dacWrite(PIN_MOTOR_CURRENT, motorCurrent);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ---------------- Sensor & PID ----------------
void readSensor() {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // Angle computation simplified for brevity
}

// ---------------- Serial Commands ----------------
void parseSerial() {
    static char serialBuf[63];
    static uint8_t pos = 0;
    while(Serial.available()) {
        char c = Serial.read();
        serialBuf[pos++] = c;
        if (c == 'x') {
            parseCommand(serialBuf, pos);
            pos = 0;
            memset(serialBuf, 0, sizeof(serialBuf));
        }
    }
}

void parseCommand(char* data, uint8_t length) {
    if(length < 3) return;
    switch(data[0]) {
        case 'c': // PID params
        case 'a': // angle offset
        case 'v': // motor current
        // other cases omitted for brevity
        default: break;
    }
}

void calculateGyroOffset(uint8_t nSample) {
    int32_t sumX=0,sumY=0,sumZ=0;
    int16_t x,y,z;
    for(uint8_t i=0;i<nSample;i++){
        imu.getRotation(&x,&y,&z);
        sumX+=x; sumY+=y; sumZ+=z;
        delay(5);
    }
    gyroOffset[0]=sumX/nSample;
    gyroOffset[1]=sumY/nSample;
    gyroOffset[2]=sumZ/nSample;
}

void sendWifiList(void) {
    char wBuf[200]; wBuf[0]='w'; wBuf[1]='l';
    uint8_t n = WiFi.scanNetworks(); if(n>5)n=5;
    uint16_t pos=2;
    for(uint8_t i=0;i<n;i++) pos += sprintf(wBuf+pos,"%s,",WiFi.SSID(i).c_str());
    wBuf[pos-1]=0;
    wsServer.sendTXT(0,wBuf);
}

