#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RTClib.h"
#include "MAX30105.h"
#include "heartRate.h"
#include <MPU6050.h>

// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

RTC_DS3231 rtc; // Declares a Real Time Clock object
MAX30105 particleSensor; // Declares the heart rate sensor object
MPU6050 mpu(0x69); // Declares the MPU6050 sensor object with address 0x69

const byte RATE_SIZE = 6; // Sets the size of the array used to average the heart rate
byte rates[RATE_SIZE]; // Array to hold last few heart rate samples
byte rateSpot = 0; // Index for the rates array
long lastBeat = 0; // Tracks the time of the last heartbeat

float beatsPerMinute; // Stores the instantaneous heart rate
int beatAvg; // Stores the average heart rate

int stepCount = 0; // Counter for total steps detected
float previousX = 0; // Stores the previous X-axis acceleration value
bool stepDetected = false; // Flag to prevent multiple step counts for one motion

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // OLED Init (Commented out until ready)
  // if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
  //   Serial.println(F("OLED allocation failed"));
  //   while (true); // Halts if OLED initialization fails
  // }
  // display.clearDisplay();
  // display.display();

  // RTC Init
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (true); // Halts if RTC initialization fails
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time...");
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Uncomment to set time
  }

  // MAX30105 Init
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found");
    while (true); // Halts if MAX30105 initialization fails
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0); // Turns off the green LED

  // MPU6050 Init
  mpu.initialize(); // Initializes the MPU6050 sensor
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1); // Halts if MPU6050 connection fails
  }
  Serial.println("MPU6050 initialized");

  delay(1500);
}

void loop() {
  DateTime now = rtc.now(); // Gets the current time from the RTC
  long irValue = particleSensor.getIR(); // Gets the infrared value from the heart rate sensor

  // Heart rate calculation logic
  if (checkForBeat(irValue)) { // Checks if a heartbeat is detected
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0); // Calculates BPM

    if (beatsPerMinute > 40 && beatsPerMinute < 210) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE; // Wraps around the array index

      beatAvg = 0;
      for (byte i = 0; i < RATE_SIZE; i++) {
        beatAvg += rates[i]; // Sums the heart rate values
      }
      beatAvg /= RATE_SIZE; // Calculates the average
    }
  }

  // MPU6050 step counting logic
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az); // Reads raw acceleration data
  float accelX = ax / 16384.0; // Converts X-axis acceleration to g's
 
  // Step detection logic based on acceleration changes
  if (!stepDetected && (accelX - previousX) > 0.3) // Threshold for X-axis change (0.3)
  {
    stepCount++; // Increments the step count
    stepDetected = true;
  }
  if ((accelX - previousX) < 0.2) {
    stepDetected = false; // Resets the detection flag
  }

  previousX = accelX;

  // Serial Monitor output for debugging
  Serial.print("Time=");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  Serial.print(", Steps=");
  Serial.print(stepCount);
  if (irValue < 50000) Serial.print("  (No finger?)");
  Serial.println();

  if(irValue < 1200) delay(9000);
  else delay(50);
}