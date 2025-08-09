#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "RTClib.h"
#include "MAX30105.h"
#include "heartRate.h"
#include <MPU6050.h>

// Reduced display size and buffer
#define SCREEN_WIDTH 64
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// RTC - using minimal DateTime
RTC_DS3231 rtc;

// Heart rate sensor with reduced buffer
MAX30105 particleSensor;
const byte RATE_SIZE = 6;  // Reduced from 6
byte rates[RATE_SIZE];
byte rateSpot = 0;
unsigned long lastBeat = 0;
byte beatAvg = 0;  // Changed from int
byte bpm;

// Step counter with optimized variables
volatile uint16_t stepCount = 0;  // Using uint16_t instead of int
float previousY = 0;
bool stepDetected = false;
MPU6050 mpu;

// Memory tracking
int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void setup() {
  // Start with minimal Serial
  Serial.begin(9600);  // Reduced from 115200 to save memory
  Serial.print(F("Init... Free RAM: "));
  Serial.println(freeRam());

  // Initialize I2C bus
  Wire.begin();

  // OLED with reduced buffer
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED failed"));
    while(1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // RTC - minimal initialization
  if(!rtc.begin()) {
    Serial.println(F("RTC failed"));
    while(1);
  }

  // Heart rate sensor
  if(!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {  // Reduced speed
    Serial.println(F("MAX30105 failed"));
    while(1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  Serial.print(F("Sensors OK. Free RAM: "));
  Serial.println(freeRam());
}

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  // Process sensors only every 50ms
  if(now - lastUpdate >= 50) {
    lastUpdate = now;
    
    // Get time (minimal processing)
    DateTime time = rtc.now();
    
    // Heart rate monitoring
    long irValue = particleSensor.getIR();
    if(checkForBeat(irValue)) {
      unsigned long delta = now - lastBeat;
      lastBeat = now;
      bpm = 60 / (delta / 1000.0);
      if(bpm > 40 && bpm < 210) {
        rates[rateSpot++] = bpm;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for(byte i=0; i<RATE_SIZE; i++) beatAvg += rates[i];
        beatAvg /= RATE_SIZE;
      }
    }

    // Step counting (simplified)
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    float accelY = ay / 16384.0; // Convert to g's
    float threshold = 0.5;

    if (!stepDetected && (accelY - previousY) > threshold) {
      stepCount++;
      stepDetected = true;
      Serial.print("Step Count: ");
      Serial.println(stepCount);
    }

    // Update display (minimal)
    display.clearDisplay();
    display.setCursor(0,0);
    //display.print(F("BPM:"));
    display.print(irValue < 50000 ? F("--") : String(beatAvg));
    
    display.setCursor(0,10);
    //display.print(F("Steps:"));
    display.print(stepCount);
    
    display.setCursor(0,22);
    if(time.hour() < 10) display.print('0');
    display.print(time.hour());
    display.print(':');
    if(time.minute() < 10) display.print('0');
    display.print(time.minute());
    
    display.display();
  }

  // Minimal serial output
  static unsigned long lastSerial = 0;
  if(now - lastSerial > 1000) {
    lastSerial = now;
    //Serial.print(F("RAM:"));
    //Serial.print(freeRam());
    Serial.print(F(" BPM:"));
    Serial.print(beatAvg);
    Serial.print("       ");
    Serial.print(bpm);
    Serial.print(F(" Steps:"));
    Serial.println(stepCount);
  }
}