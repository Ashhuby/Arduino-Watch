#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RTClib.h"
#include "MAX30105.h"
#include "heartRate.h"
#include <MPU6050.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

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

// --- Alarm ---
#define BUZZER 6
#define BUTTON 4

bool alarmTriggered = false; 

// ---- Note Definitions (only used notes) ----
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_CS4 277
#define NOTE_CS5 554
#define NOTE_D4 294
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E4 330
#define NOTE_E5 659
#define NOTE_F4 349
#define REST 0

// ---- Extended Melody Snippet ----
int melody[] = {
  NOTE_FS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_FS4,8,
  NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, REST,4, REST,8, NOTE_CS4,8,
  NOTE_D4,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
  NOTE_E5,-4, NOTE_DS5,8, NOTE_D5,8, REST,8, REST,4,
  NOTE_GS4,8, REST,8, NOTE_CS5,8, NOTE_FS4,8, REST,8, NOTE_CS5,8, REST,8, NOTE_GS4,8,
  REST,8, NOTE_CS5,8, NOTE_G4,8, NOTE_FS4,8, REST,8, NOTE_E4,8, REST,8,
  NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, REST,4, NOTE_E4,8, NOTE_E4,8
};

int tempo = 114;
int notesCount = sizeof(melody)/sizeof(melody[0])/2;
int wholenote = (60000 * 4) / tempo;

// --- Nix Symbol ---
// 'NixIcon', 128x32px
const unsigned char epd_bitmap_NixIcon [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x0a, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01, 0x30, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x21, 0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x22, 0x45, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x3e, 0xfd, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x3e, 0xb9, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x1d, 0xd8, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0xb8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0xd0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 528)
const int epd_bitmap_allArray_LEN = 1;
const unsigned char* epd_bitmap_allArray[1] = {
	epd_bitmap_NixIcon
};



void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  // OLED Init (Commented out until ready)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED allocation failed"));
    while (true); // Halts if OLED initialization fails
  }
  display.clearDisplay();
  display.display();

  // RTC Init
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (true); // Halts if RTC initialization fails
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Uncomment to set time
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


  // --- Alarm ---
  pinMode(BUZZER, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  delay(1500);
}

void loop() {
  DateTime now = rtc.now(); // Gets the current time from the RTC
  long irValue = particleSensor.getIR(); // Gets the infrared value from the heart rate sensor

  //display.clearDisplay();
  //display.display();

  // Check alarm time
  if (now.hour() == 7 && now.minute() == 30 && !alarmTriggered) {
    playSong();
    alarmTriggered = true;   // prevent replay
  }

  // Reset the flag when the minute changes
  if (!(now.hour() == 7 && now.minute() == 30)) {
    alarmTriggered = false;
  }

  // Reset steps every day
  if (now.hour() == 0 && now.minute() == 0 && now.second() == 0) {
    stepCount = 0;
  }


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

/*
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
*/

  // Oled output
  display.clearDisplay();

  // Small text so it fits
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Steps (top)
  display.setCursor(12, 0);
  display.println(stepCount);

  // Heart rate (middle)
  display.setCursor(12, 10);
  if(irValue >= 1200)
  {
    display.print(beatAvg);
    display.println(" bpm");
  }
  else
  {
    display.println("--");
  }
 
 
  // Time (bottom)
  display.setCursor(12, 25);
  if (now.hour() < 10) display.print('0'); // pad with 0
  display.print(now.hour());
  display.print(':');
  if (now.minute() < 10) display.print('0');
  display.println(now.minute());

  display.display();

  if(irValue < 1200) delay(9000);
  else delay(50);
}

void playSong()
{
  // Snow nix icon
  display.clearDisplay();
  display.drawBitmap(0,0, epd_bitmap_NixIcon, 128, 32, SSD1306_WHITE);
  display.display();

  for (int i = 0; i < notesCount * 2; i += 2) {
      // Stop alarm if button is pressed
      if (digitalRead(BUTTON) == LOW) {
        noTone(BUZZER);
        Serial.println("Alarm stopped!");
        return;
      }

      int note = melody[i];
      int divider = melody[i + 1];
      int noteDuration = 0;

      if (divider > 0) noteDuration = wholenote / divider;
      else if (divider < 0) noteDuration = (wholenote / abs(divider)) * 1.5;

      if (note == REST) noTone(BUZZER);
      else {
        tone(BUZZER, note);
        Serial.print("Playing note: ");
        Serial.print(note);
        Serial.print(" Hz for ");
        Serial.print(noteDuration);
        Serial.println(" ms");
      }

      delay(noteDuration);
      noTone(BUZZER);
      delay(50);
    }
}

