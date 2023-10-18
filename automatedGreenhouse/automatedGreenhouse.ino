#define BLYNK_TEMPLATE_ID "TMPL4GEf_iJ-o"
#define BLYNK_TEMPLATE_NAME "PG5501 Exam Greenhouse Monitoring System"
#define BLYNK_AUTH_TOKEN "Mb_EAhRPd1Eh0Yg7unsr2Uas91RmxYuG"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Stepper.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_LTR329_LTR303.h"
#include "time.h"

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "your_ssid_here";
char pass[] = "your_password_here";

// DHT11 Temperature & Humidity sensor
#define DHTPIN 5
#define DHTTYPE DHT11

// DS18B20 Temperature sensor
const int DS18B20TemperatureSensorPin = 17;

// HW-038 Water Level Sensor
const int waterLevelSensorPin = A0;

// 74HS595 Shift Register
const int shiftRegisterLatchPin = 38;
const int shiftRegisterClockPin = 39;
const int ShiftRegisterDataPin = 13;

// DC Motor
const int DCMotorSpeedControlPin = 36;
const int DCMotorDirectionControlPin1 = 35;
const int DCMotorDirectionControlPin2 = 37;

// SD90 Servo motor
const int servoPin = 12;

// ULN2003 Step Motor Driver Pins
const int stepMotorPin1 = 6;   // -> IN1
const int stepMotorPin2 = 10;  // -> IN2
const int stepMotorPin3 = 9;   // -> IN3
const int stepMotorPin4 = 11;  // -> IN4

// Clock
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;
struct tm timeinfo;

uint16_t lightLevelVisibleAndInfrared, lightLevelInfrared;

const int windowClosedServoPosition = 180;
const int stepsPerRevolution = 2048;

int pineappleSoilHumidity = 0;
int overrideDCMotor = 0;
int manualDCMotorSpeed = 0;
int overrideServo = 0;
int manualServoPosition = 0;

byte ledControlByte = 0;

float dhtHumidity = 0;
float dhtInsideTemperature = 0;
float shtOutsideTemperature = 0;
float shtHumidity = 0;

bool isPlantsAwake = false;
bool isLightsOn = false;

enum {
  FLOWERING,
  VEGETATIVE,
  SEEDLING
};
unsigned char plantLifeStage = SEEDLING;

Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_LTR329 ltr = Adafruit_LTR329();
BlynkTimer timer;
Servo servo;
DHT dht(DHTPIN, DHTTYPE);
Stepper stepMotor(stepsPerRevolution, stepMotorPin1, stepMotorPin3, stepMotorPin2, stepMotorPin4);
OneWire oneWire(DS18B20TemperatureSensorPin);
DallasTemperature DS18B20TemperatureSensor(&oneWire);

void setup() {
  Serial.begin(115200);
  initPinModes();
  initComponents();
  timer.setInterval(5000L, checkSensorsForAnomalies);
}

void loop() {
  Blynk.run();
  timer.run();
}

void checkSensorsForAnomalies() {
  printLocalTime();
  checkDHTAndSHTToCompareInsideAndOutsideTemperature();
  checkLtrSensorIfLightsAreNeeded();
  checkDS18B20TemperatureSensorIfDistanceBetweenLightsAndPlantsAreGood();
  readPineappleSoilHumidity();
}

void checkDHTAndSHTToCompareInsideAndOutsideTemperature() {
  readDhtData();
  readShtData();
  if (overrideServo == 1) {
    servo.write(manualServoPosition);
  }
  if (overrideDCMotor == 1) {
    // Value-range between -50 and 50 to give user some leeway and ease of use,
    // and because motor will not start doing work before value > ~70 || < ~-80
    if (manualDCMotorSpeed < 50 && manualDCMotorSpeed > -50) {
      stopDCMotor();
    } else if (manualDCMotorSpeed >= 50) {
      setSpeedAndDirectionOfDCMotor(true, manualDCMotorSpeed);
    } else if (manualDCMotorSpeed <= -50) {
      setSpeedAndDirectionOfDCMotor(false, abs(manualDCMotorSpeed));
    }
  } else {
    /*
      If temperature inside < outside, we have to use the AC to stay around
      optimal growth temperature (in this case signalized by DC motor blowing
      air into the greenhouse and servo to adjust how open windows should be). 
      If opposite is true, we can use the same motor in reverse to suck air 
      from outside into the greenhouse.
    */
    if (dhtInsideTemperature < shtOutsideTemperature) {
      if (dhtInsideTemperature > 28) {
        // Operation on significantly high temperature
        setSpeedAndDirectionOfDCMotor(true, 255);
        autoServo(0);
      } else if (dhtInsideTemperature > 26) {
        // Operation on slightly high temperature
        setSpeedAndDirectionOfDCMotor(true, 150);
        autoServo(60);
      } else {
        // Normal daily operation on ideal temperature
        setDCAndServoInNormalOperationMode();
      }
    } else {
      if (dhtInsideTemperature > 28) {
        // Operation on significantly high temperature
        setSpeedAndDirectionOfDCMotor(false, 255);
        autoServo(0);
      } else if (dhtInsideTemperature > 26) {
        // Operation on slightly high temperature
        setSpeedAndDirectionOfDCMotor(false, 150);
        autoServo(60);
      } else {
        // Normal daily operation on ideal temperature
        setDCAndServoInNormalOperationMode();
      }
    }
  }
}

// DS18B20; Measure Temperature at the top of the plant
void checkDS18B20TemperatureSensorIfDistanceBetweenLightsAndPlantsAreGood() {
  DS18B20TemperatureSensor.requestTemperatures();
  float DS18B20TemperatureInCelsius = DS18B20TemperatureSensor.getTempCByIndex(0);
  /*
    Check if plants are awake, if lights are on, and if temperature at the top of the
    plant is too high/low. If all is true, then adjust the distance between LEDs and
    top of plant at an increment of 1 step per interval. Since plants grow relatively
    slowly, adjusting with 1 step per interval will suffice to avoid burns on leaves.
    At the same time, if the temperature is too low, it will lessen the distance to
    help with achieving optimal growing conditions.
  */
  if (isPlantsAwake && isLightsOn) {
    if (DS18B20TemperatureInCelsius > 26.5) {
      stepMotor.step(1);
    } else if (DS18B20TemperatureInCelsius < 23.00) {
      stepMotor.step(-1);
    }
  }
  Blynk.virtualWrite(V13, DS18B20TemperatureInCelsius);
  Serial.print("DS18B20 Temperature: ");
  Serial.print(DS18B20TemperatureInCelsius);
  Serial.println("ºC");
}

void checkLtrSensorIfLightsAreNeeded() {
  uint16_t lightLevel = checkLightLevel();
  // Check if plants are awake (18 hours for seedling/vegetative, 12 hours for flowering)
  if (plantLifeStage == FLOWERING) {
    if (timeinfo.tm_hour > 8 && timeinfo.tm_hour < 20) {
      isPlantsAwake = true;
      // Check if sun is providing enough light for the plants
      if (lightLevel < 10000) {
        setLightLevel(lightLevel);
      } else {
        printLightLevel(lightLevel);
        turnOffLights();
      }
    }
  } else if (timeinfo.tm_hour > 5 && timeinfo.tm_hour < 23) {
    isPlantsAwake = true;
    // Check if sun is providing enough light for the plants
    if (lightLevel < 10000) {
      setLightLevel(lightLevel);
    } else {
      printLightLevel(lightLevel);
      turnOffLights();
    }
  } else {
    isPlantsAwake = false;
    turnOffLights();
  }
}

uint16_t checkLightLevel() {
  ltr.readBothChannels(lightLevelVisibleAndInfrared, lightLevelInfrared);
  Blynk.virtualWrite(V7, lightLevelVisibleAndInfrared);
  return lightLevelVisibleAndInfrared;
}

void setLightLevel(uint16_t lightLevel) {
  printLightLevel(lightLevel);
  switch (plantLifeStage) {
    case SEEDLING:
      startFloweringLights();
      break;
    case VEGETATIVE:
      startVegetativeLights();
      break;
    case FLOWERING:
      startSeedlingLights();
      break;
    default:
      break;
  }
  isLightsOn = true;
}

void readPineappleSoilHumidity() {
  pineappleSoilHumidity = analogRead(waterLevelSensorPin);
  Serial.print("Pineapple soil humidity: ");
  Serial.println(pineappleSoilHumidity);
  Blynk.virtualWrite(V10, pineappleSoilHumidity);
}

void readDhtData() {
  dhtHumidity = dht.readHumidity();
  dhtInsideTemperature = dht.readTemperature();

  if (isnan(dhtHumidity) || isnan(dhtInsideTemperature)) {
    Serial.println("Failed to read from DHT sensor");
    return;
  }

  Blynk.virtualWrite(V0, dhtInsideTemperature);
  Blynk.virtualWrite(V1, dhtHumidity);
  Serial.print("DHT Temperature : ");
  Serial.print(dhtInsideTemperature);
  Serial.print("ºC");
  Serial.print("\tDHT Humidity : ");
  Serial.println(dhtHumidity);
}

void readShtData() {
  shtOutsideTemperature = sht31.readTemperature();
  shtHumidity = sht31.readHumidity();

  if (isnan(shtHumidity) || isnan(shtOutsideTemperature)) {
    Serial.println("Failed to read from SHT sensor");
    return;
  }

  Blynk.virtualWrite(V2, shtOutsideTemperature);
  Blynk.virtualWrite(V3, shtHumidity);
  Serial.print("SHT Temperature : ");
  Serial.print(shtOutsideTemperature);
  Serial.print("ºC");
  Serial.print("\tSHT Humidity : ");
  Serial.println(shtHumidity);
}

void printLightLevel(uint16_t lightLevel) {
  Serial.print("Current light level: ");
  Serial.print(lightLevelVisibleAndInfrared);
  Serial.println(" lux.");
}

void turnOffLights() {
  ledControlByte = 0;
  updateShiftRegister();
  isLightsOn = false;
}

void startFloweringLights() {
  turnOffLights();
  bitSet(ledControlByte, 1);
  bitSet(ledControlByte, 4);
  bitSet(ledControlByte, 7);
  updateShiftRegister();
}

void startVegetativeLights() {
  turnOffLights();
  bitSet(ledControlByte, 0);
  bitSet(ledControlByte, 2);
  bitSet(ledControlByte, 3);
  bitSet(ledControlByte, 5);
  bitSet(ledControlByte, 6);
  updateShiftRegister();
}

void startSeedlingLights() {
  turnOffLights();
  bitSet(ledControlByte, 2);
  bitSet(ledControlByte, 5);
  updateShiftRegister();
}

void autoServo(int servoPosition) {
  if (overrideServo == 0) {
    servo.write(servoPosition);
  }
}

void setDCAndServoInNormalOperationMode() {
  setSpeedAndDirectionOfDCMotor(true, 80);
  autoServo(120);
}

void setSpeedAndDirectionOfDCMotor(bool direction, int speed) {
  switch (direction) {
    case true:
      digitalWrite(DCMotorDirectionControlPin1, HIGH);
      digitalWrite(DCMotorDirectionControlPin2, LOW);
      break;
    case false:
      digitalWrite(DCMotorDirectionControlPin1, LOW);
      digitalWrite(DCMotorDirectionControlPin2, HIGH);
      break;
    default:
      break;
  }
  analogWrite(DCMotorSpeedControlPin, speed);
}

void stopDCMotor() {
  digitalWrite(DCMotorDirectionControlPin1, LOW);
  digitalWrite(DCMotorDirectionControlPin2, LOW);
}

void updateShiftRegister() {
  digitalWrite(shiftRegisterLatchPin, LOW);
  shiftOut(ShiftRegisterDataPin, shiftRegisterClockPin, LSBFIRST, ledControlByte);
  digitalWrite(shiftRegisterLatchPin, HIGH);
}

void getLocalTimeAndDate() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
}

void printLocalTime() {
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%H:%M:%S");
}

void initComponents() {
  Blynk.begin(auth, ssid, pass);
  Wire.begin(3, 4);

  if (!sht31.begin(0x44)) {
    Serial.println("SHT31 not found");
    while (1) delay(10);
  }

  if (!ltr.begin()) {
    Serial.println("LTR not found");
    while (1) delay(10);
  }

  dht.begin();
  DS18B20TemperatureSensor.begin();

  stepMotor.setSpeed(5);

  ltr.setGain(LTR3XX_GAIN_4);
  ltr.setIntegrationTime(LTR3XX_INTEGTIME_50);
  ltr.setMeasurementRate(LTR3XX_MEASRATE_50);

  servo.attach(servoPin);
  servo.write(windowClosedServoPosition);

  turnOffLights();
  getLocalTimeAndDate();
}

void initPinModes() {
  pinMode(shiftRegisterLatchPin, OUTPUT);
  pinMode(ShiftRegisterDataPin, OUTPUT);
  pinMode(shiftRegisterClockPin, OUTPUT);
  pinMode(DCMotorSpeedControlPin, OUTPUT);
  pinMode(DCMotorDirectionControlPin1, OUTPUT);
  pinMode(DCMotorDirectionControlPin2, OUTPUT);

  digitalWrite(DCMotorDirectionControlPin1, LOW);
  digitalWrite(DCMotorDirectionControlPin2, LOW);
}

BLYNK_WRITE(V4) {
  plantLifeStage = param.asInt();
  Serial.print("Plant life stage: ");
  Serial.println(plantLifeStage);
}

BLYNK_WRITE(V5) {
  overrideDCMotor = param.asInt();
  Serial.print("DC Motor override: ");
  Serial.println(overrideDCMotor);
}

BLYNK_WRITE(V6) {
  manualDCMotorSpeed = param.asInt();
  Serial.print("Manual motor speed: ");
  Serial.println(manualDCMotorSpeed);
}

BLYNK_WRITE(V8) {
  overrideServo = param.asInt();
  Serial.print("Servo motor override: ");
  Serial.println(overrideServo);
}

BLYNK_WRITE(V9) {
  manualServoPosition = param.asInt();
  Serial.print("Manual servo position: ");
  Serial.println(manualServoPosition);
}