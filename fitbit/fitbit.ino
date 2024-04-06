// Basic demo for accelerometer readings from Adafruit MPU6050

// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

#include <MAX30105.h>
#include <heartRate.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "Adafruit_MAX1704X.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "pitches.h"

// Display Pins
#define TFT_CS        7  // Not used in this setup, SPI CS is used automatically
#define TFT_RESET     40  // Not used
#define TFT_DC        39  // Data Command 
#define TFT_I2C_POWER 21  // Power control pin for the TFT display
#define TFT_BACKLIGHT 45

#define PIN 33 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 16 // Popular NeoPixel ring size

// Initialize Adafruit ST7789 (Display)
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RESET);

// Neopixel init
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Initialize MAX30102 (Heart Rate Sensor) and MPU6050 (IMU) and  MAX17048 (Battery)
Adafruit_MAX17048 maxlipo;
Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

const byte RATE_SIZE = 1; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

// moving average variables
#define QUEUE_SIZE 10

// threshold for counting a step
double UPPER_STEP_THRESH = 10.0;
bool isBelowThresh = true;
int n_steps = 0;

// struct for moving average
struct MovingAverage {
  double array[QUEUE_SIZE];  
  int currentIndex;
  int count;                 
  double sum;
};

// moving average helper functions
void initializeMovingAverage(MovingAverage &ma) {
  ma.currentIndex = 0;
  ma.count = 0;
  ma.sum = 0.0;
  for (int i = 0; i < QUEUE_SIZE; ++i) {
    ma.array[i] = 0.0;  // Initialize all elements to 0
  }
}

double addToMovingAverage(MovingAverage& ma, double value) {
  // If the circular array is at full capacity, subtract the oldest value from the sum
  if (ma.count >= QUEUE_SIZE) {
    ma.sum -= ma.array[ma.currentIndex];
  } else {
    // Increment the count if not at full capacity
    ma.count++;
  }
  // Add the new value to the sum
  ma.sum += value;
  // Add the new value to the circular array and update the index (circular)
  ma.array[ma.currentIndex] = value;
  ma.currentIndex = (ma.currentIndex + 1) % QUEUE_SIZE;
  // Return the updated average
  return ma.sum / ma.count;
}

double accelMagnitude;
double accelAvgMagnitude;
MovingAverage accelMovingAvg;

// Neopixel statemachine
#define LOWER_ACTIVITY_THRESH 5
#define UPPER_ACTIVITY_THRESH 14

enum ActivityLevel {
  ACTIVITY_LOW,
  MODERATE,
  ACTIVITY_HIGH
};

void pulseBlueLight() {
  static uint8_t brightness = 0;
  static int fadeAmount = 5;

  for(int i=0; i<NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, brightness));
  }
  pixels.show();

  brightness += fadeAmount;
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount; // reverse direction
  }
  delay(30);
}

void pulseRedLight() {
  static uint8_t brightness = 0;
  static int fadeAmount = 5;

  for(int i=0; i<NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(brightness, 0, 0));
  }
  pixels.show();

  brightness += fadeAmount;
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount; // reverse direction
  }
  delay(30);
}

void pulsePurpleLight() {
  static uint8_t brightness = 0;
  static int fadeAmount = 5;

  for(int i=0; i<NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(brightness, 0, brightness));
  }
  pixels.show();

  brightness += fadeAmount;
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount; // reverse direction
  }
  delay(30);
}

void pulseGreenLight() {
  static uint8_t brightness = 0;
  static int fadeAmount = 5;

  for(int i=0; i<NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, brightness, 0));
  }
  pixels.show();

  brightness += fadeAmount;
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount; // reverse direction
  }
  delay(30);
}

ActivityLevel currentActivity = ACTIVITY_LOW;

// sounds setup
int notif_sound[] = {
  NOTE_C4,
  NOTE_E4,
  NOTE_G4,
  END
};
int notif_durations[] = { 
  4, 4, 8
};

int goal_complete[] = {
  NOTE_C5,
  NOTE_E5,
  NOTE_G5,
  NOTE_C6,
  NOTE_G5,
  NOTE_E5,
  NOTE_C5,
  NOTE_E5,
  NOTE_G5,
  NOTE_C6,
  NOTE_G5,
  NOTE_E5,
  NOTE_C5,
  NOTE_D5,
  NOTE_B4,
  NOTE_C5
};
int goal_complete_durations[] = {
  4, 4, 4, 4, 4, 4, 4, 4,
  4, 4, 4, 4, 4, 4, 4, 8
};

int note_speed=90;

#define STEP_GOAL 20
#define CHIME_EVERY 10
#define SPEAKER_PIN 11
int lastPlayed = 0; // keeps track of the number of steps at which the sound was last played

void setup(void) {
  Serial.begin(9600);
  Serial.println("Initializing...");
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Begin Neopixel
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  
  // Initialize display power
  pinMode(TFT_I2C_POWER, OUTPUT);
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH); // Power on the display
  digitalWrite(TFT_BACKLIGHT, HIGH);
  
  // Initialize TFT display
  tft.init(135, 240); // Initialize display with its resolution
  tft.setRotation(1); // Set the rotation as needed
  tft.fillScreen(ST77XX_BLACK); // Clear display to black

  while (!maxlipo.begin()) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    delay(2000);
  }
  Serial.print(F("Found MAX17048"));
  Serial.print(F(" with Chip ID: 0x")); 
  Serial.println(maxlipo.getChipID(), HEX);

  // Initialize IMU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Initialize Heart Sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found");
    while (1);
  }
  Serial.println("Place your index finger on the heart sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  Serial.println("MPU6050 and MAX30102 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  // initialize moving average
  initializeMovingAverage(accelMovingAvg);

  Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  // long irValue = particleSensor.getIR();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  long irValue = particleSensor.getIR();

  // 3 successive loops to update bpm faster
  int count = 0;
  while (count < 2){
      if (checkForBeat(irValue) == true) {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      // if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      //   rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      //   rateSpot %= RATE_SIZE; //Wrap variable

      //   //Take average of readings
      //   beatAvg = 0;
      //   for (byte x = 0 ; x < RATE_SIZE ; x++)
      //     beatAvg += rates[x];
      //   beatAvg /= RATE_SIZE;
      // }
    }
    count ++;
  }

  float cellVoltage = maxlipo.cellVoltage();
  if (isnan(cellVoltage)) {
    Serial.println("Failed to read cell voltage, check battery is connected!");
    delay(2000);
    return;
  }
  // Serial.print(F("Batt Voltage: ")); Serial.print(cellVoltage, 3); Serial.println(" V");
  // Serial.print(F("Batt Percent: ")); Serial.print(maxlipo.cellPercent(), 1); Serial.println(" %");
  // Serial.println();

  /* Print out the values */
  // Serial.print("IR=");
  // Serial.print(irValue);
  // Serial.print(", BPM=");
  // Serial.print(beatsPerMinute);
  // Serial.println("");
  
  // Fill display with black
  tft.fillScreen(ST77XX_BLACK);

  // Setting up text properties
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE);  
  tft.setTextSize(2);
  
  tft.println("BPM: " + String(beatsPerMinute));
  tft.println("Avg BPM: " + String(beatAvg));
  tft.println("Number of Steps: " + String(n_steps));
  tft.println("Batt Voltage:" + String(maxlipo.cellVoltage(), 3) + " V");
  tft.println("Batt Percent:" + String(maxlipo.cellPercent(), 1) + " %");

  // tft.println("IR: " + String(irValue));
  // tft.println("Accel X: " + String(a.acceleration.x) + " m/s^2");
  // tft.println("Accel Y: " + String(a.acceleration.y) + " m/s^2");
  // tft.println("Accel Z: " + String(a.acceleration.z) + " m/s^2");
  // tft.println("Rot X: " + String(g.gyro.x) + " m/s^2");
  // tft.println("Rot Y: " + String(g.gyro.x) + " m/s^2");
  // tft.println("Rot Z: " + String(g.gyro.x) + " m/s^2");

  // Serial.print(", Avg BPM=");
  // Serial.print(beatAvg);

  // if (irValue < 50000)
    // Serial.print(" No finger?");
  // Serial.println("");

  // add y to moving average
  accelMagnitude = sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.x, 2));
  accelAvgMagnitude = addToMovingAverage(accelMovingAvg, accelMagnitude);
  // Serial.println(accelAvgMagnitude);

  if (accelAvgMagnitude < UPPER_STEP_THRESH){
    isBelowThresh = true;
  }

  if (accelAvgMagnitude > UPPER_STEP_THRESH && isBelowThresh == true){
    isBelowThresh = false;
    n_steps ++;
    Serial.print("Number of Steps: ");
    Serial.println(n_steps);
  }

  if (n_steps == STEP_GOAL && n_steps != lastPlayed){
    Serial.println("Goal reached!");
    for (int currNote = 0; goal_complete[currNote]!=-1; currNote++) {
      int noteDuration = note_speed*goal_complete_durations[currNote];
      tone(SPEAKER_PIN, goal_complete[currNote],noteDuration*.95);
      delay(noteDuration);
      noTone(SPEAKER_PIN);
    }
    lastPlayed = n_steps; // update lastplayed to avoid repeated chimes
  }
  else if (n_steps>0 && n_steps % CHIME_EVERY == 0 && n_steps != lastPlayed){
    Serial.println("Playing sound");
    for (int currNote = 0; notif_sound[currNote]!=-1; currNote++) {
      int noteDuration = note_speed*notif_durations[currNote];
      tone(SPEAKER_PIN, notif_sound[currNote],noteDuration*.95);
      delay(noteDuration);
      noTone(SPEAKER_PIN);
    }
    lastPlayed = n_steps; // update lastplayed to avoid repeated chimes
  }

  // Determine the current activity level based on accelAvgMagnitude
  if (accelAvgMagnitude < LOWER_ACTIVITY_THRESH) {
    currentActivity = ACTIVITY_LOW;
  } else if (accelAvgMagnitude >= LOWER_ACTIVITY_THRESH && accelAvgMagnitude < UPPER_ACTIVITY_THRESH) {
    currentActivity = MODERATE;
  } else if (accelAvgMagnitude >= UPPER_ACTIVITY_THRESH) {
    currentActivity = ACTIVITY_HIGH;
  }

    // Update NeoPixel effects based on the current activity level (state machine)
  switch (currentActivity) {
    case ACTIVITY_LOW:
      pulseBlueLight();
      break;
    case MODERATE:
      pulsePurpleLight();
      break;
    case ACTIVITY_HIGH:
      pulseGreenLight();
      break;
  }
  
  // Serial.print("Acceleration X: ");
  // Serial.print(a.acceleration.x);
  // Serial.print(",");s
  // // Serial.print(", Y: ");
  // Serial.print(a.acceleration.y);
  // Serial.print(",");
  // // Serial.print(", Z: ");
  // Serial.print(a.acceleration.z);
  // Serial.println("");
  // Serial.println(" m/s^2");

  // Serial.print("Rotation X: ");
  // Serial.print(g.gyro.x);
  // Serial.print(", Y: ");
  // Serial.print(g.gyro.y);
  // Serial.print(", Z: ");
  // Serial.print(g.gyro.z);
  // Serial.println(" rad/s");

  // Serial.print("Temperature: ");
  // Serial.print(temp.temperature);
  // Serial.println(" degC");

  // Serial.println("");
  // delay(50);
}