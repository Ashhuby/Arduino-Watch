#include <Wire.h>
#include <MPU6050.h>
#include <EEPROM.h>  // Add EEPROM library

MPU6050 mpu;
int stepCount = 0;
int XStep = 0;
int ZStep = 0;
float previousX = 0;
float previousZ = 0;
bool stepDetected = false;

// EEPROM address for storing step count
#define STEP_ADDR 0
// Save interval (steps) to reduce EEPROM wear
#define SAVE_INTERVAL 10
int lastSavedCount = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  //writeIntToEEPROM(STEP_ADDR, 0); // Reset Step Count

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Read saved step count from EEPROM
  stepCount = readIntFromEEPROM(STEP_ADDR);
  lastSavedCount = stepCount;
  Serial.print("Last saved step count: ");
  Serial.println(stepCount);
  
  Serial.println("MPU6050 initialized");
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float accelX = ax / 16384.0; // Convert to g's
  float accelZ = az / 16384.0; // Convert to g's
  float Zthreshold = 0.3;
  float Xthreshold = 0.3;

  if (!stepDetected && (accelX - previousX) > Zthreshold) {
    XStep++; 
    stepCount++;
    stepDetected = true;
    Serial.println(stepCount);
    saveStepCount(); // Check if we need to save
  }

  if ((accelX - previousX) < 0.2) {
    stepDetected = false;
  }

  if (!stepDetected && (accelZ - previousZ) > Xthreshold) {
    ZStep++; 
    stepCount++;
    stepDetected = true;
    Serial.println(stepCount);
    saveStepCount(); // Check if we need to save
  }

  if ((accelZ - previousZ) < 0.2) {
    stepDetected = false;
  }

  previousX = accelX;
  previousZ = accelZ;
  delay(200); // adjust based on sampling needs
}

// Save step count to EEPROM periodically
void saveStepCount() {
  if (stepCount - lastSavedCount >= SAVE_INTERVAL) {
    writeIntToEEPROM(STEP_ADDR, stepCount);
    lastSavedCount = stepCount;
    Serial.println("Saved to EEPROM!");
  }
}

// Write a 2-byte integer to EEPROM
void writeIntToEEPROM(int address, int value) {
  EEPROM.write(address, highByte(value));
  EEPROM.write(address + 1, lowByte(value));
}

// Read a 2-byte integer from EEPROM
int readIntFromEEPROM(int address) {
  byte high = EEPROM.read(address);
  byte low = EEPROM.read(address + 1);
  return (high << 8) | low;
}