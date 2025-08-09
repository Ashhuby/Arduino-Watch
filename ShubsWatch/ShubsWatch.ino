#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RTClib.h"
#include "MAX30105.h"
#include "heartRate.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

RTC_DS3231 rtc;
MAX30105 particleSensor;

const byte RATE_SIZE = 6;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute;
int beatAvg;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // OLED Init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED allocation failed"));
    while (true);
  }
  display.clearDisplay();
  display.display();

  // RTC Init
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (true);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time...");
    // Uncomment once to set RTC to current compile time:
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // MAX30105 Init
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found");
    while (true);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0); // Turn off green LED

  delay(1500);
}

void loop() {
  DateTime now = rtc.now();

  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute > 40 && beatsPerMinute < 210) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte i = 0; i < RATE_SIZE; i++) {
        beatAvg += rates[i];
      }
      beatAvg /= RATE_SIZE;
    }
  }

  char timeBuffer[10];
  sprintf(timeBuffer, "%02d:%02d", now.hour(), now.minute());

  char bpmBuffer[10];
  if (irValue < 50000) {
    strcpy(bpmBuffer, "--");
  } else {
    sprintf(bpmBuffer, "%d", beatAvg);
  }

  display.clearDisplay();

  // Draw only on left half
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(bpmBuffer);
  
  
  display.setCursor(0, 20);
  display.print("Step Count");
  
  display.setCursor(0, 50);
  display.print(timeBuffer);

  display.display();

  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  if (irValue < 50000) Serial.print("          Dont see finger");
  Serial.println();
}
