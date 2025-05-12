#include <TFT_eSPI.h>
#include "NotoSansBold36.h"
#include <EEPROM.h>
#define AA_FONT_LARGE NotoSansBold36

#define DIAL_CENTRE_X 120
#define DIAL_CENTRE_Y 125 // Move dial lower (was 120)
#define DIAL_RADIUS 110   // Widen dial (was 100)
#define NEEDLE_LENGTH 80  // Optionally, make needle longer for bigger dial
#define NEEDLE_WIDTH 7    // Optionally, make needle wider for bigger dial

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite needle = TFT_eSprite(&tft);
TFT_eSprite spr = TFT_eSprite(&tft);

uint16_t spr_width = 0;
uint16_t bg_color = TFT_BLACK;

// Sensor pin
const uint8_t sensorPin = D1; // Use the actual pin connected to your Hall sensor
const uint8_t buttonPin = D2; // Connect push button here

const int EEPROM_ADDR = 0; // EEPROM address to store calibrationFactor

const float SLIP_FACTOR = 1.05; // Adjust this value based on your calibration
const float CM_PER_REV = (3 * 7.5) * SLIP_FACTOR; // = approx 236 mm per revolution, adjust for your wheel size
const int PULSES_PER_REV = 3;     // Adjust if your sensor gives a different number of pulses per revolution
const float METERS_PER_REV = CM_PER_REV / 100.0;
const float KNOTS_PER_MPS = 1.0 / 0.51444;
const float BASE_CONVERSION_FACTOR = (METERS_PER_REV / PULSES_PER_REV) * KNOTS_PER_MPS;

volatile unsigned long pulseCount = 0;
unsigned long lastPulseCount = 0;
unsigned long lastCalcTime = 0;

float calibrationFactor = 1.0;
float displayedKnots = 0.0;
float displayedRPM = 0.0;
const float smoothingFactor = 0.2;

bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

unsigned long calibrationDisplayMillis = 0;
bool showCalibration = false;

unsigned long lastDisplayUpdateTime = 0;

unsigned long lastPulseTime = 0;
const unsigned long ZERO_TIMEOUT = 2000; // 2 seconds without pulses = zero speed

void IRAM_ATTR onPulse()
{
  pulseCount++;
  // Serial.print("Pulse count: ");
  // Serial.println(pulseCount);
}

void loadCalibration()
{
  EEPROM.begin(512);
  EEPROM.get(EEPROM_ADDR, calibrationFactor);
  if (isnan(calibrationFactor) || calibrationFactor < 0.5 || calibrationFactor > 1.5)
  {
    calibrationFactor = 1.0; // Reset to default if corrupted
  }
}

void saveCalibration()
{
  EEPROM.put(EEPROM_ADDR, calibrationFactor);
  EEPROM.commit();
  // Serial.print("Calibration saved: ");
  // Serial.println(calibrationFactor, 3);
}

void drawDial()
{
  // Draw dial background
  tft.fillScreen(bg_color);
  tft.fillCircle(DIAL_CENTRE_X, DIAL_CENTRE_Y, DIAL_RADIUS, TFT_DARKGREY);
  tft.fillCircle(DIAL_CENTRE_X, DIAL_CENTRE_Y, DIAL_RADIUS - 8, TFT_BLACK);

  // Draw ticks and numbers
  for (int i = 0; i <= 15; ++i)
  {
    float angle = (225.0 - (i * 270.0 / 15.0)) * DEG_TO_RAD;
    int x0 = DIAL_CENTRE_X + cos(angle) * (DIAL_RADIUS - 10);
    int y0 = DIAL_CENTRE_Y - sin(angle) * (DIAL_RADIUS - 10);
    int x1 = DIAL_CENTRE_X + cos(angle) * (DIAL_RADIUS - 2);
    int y1 = DIAL_CENTRE_Y - sin(angle) * (DIAL_RADIUS - 2);
    tft.drawLine(x0, y0, x1, y1, TFT_WHITE);

    // Draw numbers
    char buf[4];
    sprintf(buf, "%d", i);
    int tx = DIAL_CENTRE_X + cos(angle) * (DIAL_RADIUS - 25);
    int ty = DIAL_CENTRE_Y - sin(angle) * (DIAL_RADIUS - 25);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(buf, tx, ty, 2);
  }

  // Draw units, moved lower (e.g. +60 pixels)
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("knots", DIAL_CENTRE_X, DIAL_CENTRE_Y + DIAL_RADIUS / 2 + 40, 2);
}

void createNeedle()
{
  needle.setColorDepth(16);
  needle.createSprite(NEEDLE_WIDTH, NEEDLE_LENGTH);
  needle.fillSprite(TFT_BLACK);
  needle.setPivot(NEEDLE_WIDTH / 2, NEEDLE_LENGTH - 2);
  needle.fillRect(0, 0, NEEDLE_WIDTH, NEEDLE_LENGTH, TFT_RED);
}

void plotNeedle(float value)
{
  static float old_value = -1;

  // Erase old needle by overdrawing with dial center color
  if (old_value >= 0)
  {
    float old_angle = (225.0 - (old_value * 270.0 / 15.0)) * DEG_TO_RAD;
    // Draw over old needle with dial center color (TFT_BLACK)
    float dx = cos(old_angle);
    float dy = -sin(old_angle);
    int x0 = DIAL_CENTRE_X + int(-dy * 7); // wider base
    int y0 = DIAL_CENTRE_Y + int(dx * 7);
    int x1 = DIAL_CENTRE_X + int(dy * 7);
    int y1 = DIAL_CENTRE_Y - int(dx * 7);
    int x2 = DIAL_CENTRE_X + int(dx * (NEEDLE_LENGTH));
    int y2 = DIAL_CENTRE_Y + int(dy * (NEEDLE_LENGTH));
    tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_BLACK);
  }

  // Draw new needle as a filled triangle (cooler look)
  float angle = (225.0 - (value * 270.0 / 15.0)) * DEG_TO_RAD;
  float dx = cos(angle);
  float dy = -sin(angle);
  int x0 = DIAL_CENTRE_X + int(-dy * 7); // wider base
  int y0 = DIAL_CENTRE_Y + int(dx * 7);
  int x1 = DIAL_CENTRE_X + int(dy * 7);
  int y1 = DIAL_CENTRE_Y - int(dx * 7);
  int x2 = DIAL_CENTRE_X + int(dx * (NEEDLE_LENGTH));
  int y2 = DIAL_CENTRE_Y + int(dy * (NEEDLE_LENGTH));
  tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_RED);
  // Optional: add a black outline for extra style
  tft.drawTriangle(x0, y0, x1, y1, x2, y2, TFT_BLACK);

  // Draw digital value (with one decimal)
  spr.fillSprite(bg_color);
  spr.setTextColor(TFT_WHITE, TFT_TRANSPARENT, true);
  char buf[8];
  dtostrf(value, 4, 1, buf); // width=4, 1 decimal
  spr.setTextDatum(MC_DATUM);
  spr.drawString(buf, spr_width / 2 - 2, spr.fontHeight() / 2 + 5);                         // shift left, lower for better centering
  spr.pushSprite(DIAL_CENTRE_X - spr_width / 2, DIAL_CENTRE_Y - spr.fontHeight() / 2 + 66); // was +54

  old_value = value;
}

void printOtherValues(float rpm, float kmh, float mph)
{
  static TFT_eSprite valueSpr = TFT_eSprite(&tft);
  static int valueSprWidth = 120;
  static int valueSprHeight = 32;
  static bool sprInitialized = false;

  if (!sprInitialized)
  {
    valueSpr.setColorDepth(16);
    valueSpr.createSprite(valueSprWidth, valueSprHeight);
    sprInitialized = true;
  }

  // Center X
  int x = DIAL_CENTRE_X - valueSprWidth / 2;
  // Y: halfway between dial center and "knots" label, minus a bit for spacing
  int y = DIAL_CENTRE_Y + (DIAL_RADIUS / 4) - 3;

  valueSpr.fillSprite(bg_color);
  valueSpr.setTextColor(TFT_WHITE, TFT_TRANSPARENT, true);
  valueSpr.setTextDatum(MC_DATUM);
  valueSpr.drawString("RPM", valueSprWidth / 2, valueSprHeight / 2 - 10, 2);
  valueSpr.pushSprite(x, y - 10);

  // Draw km/h and mph below RPM
  valueSpr.fillSprite(bg_color);
  valueSpr.setTextColor(TFT_WHITE, TFT_TRANSPARENT, true);
  valueSpr.setTextDatum(MC_DATUM);
  valueSpr.drawString(String(rpm, 0), valueSprWidth / 2, valueSprHeight / 2, 2);
  valueSpr.pushSprite(x, y);
  valueSpr.fillSprite(bg_color);
}

void showCalibrationFactor()
{
  static TFT_eSprite calSpr = TFT_eSprite(&tft);
  static int calSprWidth = 100;
  static int calSprHeight = 40;
  static bool calSprInitialized = false;

  if (!calSprInitialized)
  {
    calSpr.setColorDepth(16);
    calSpr.createSprite(calSprWidth, calSprHeight);
    calSprInitialized = true;
  }

  calSpr.fillSprite(bg_color);
  calSpr.setTextColor(TFT_YELLOW, TFT_TRANSPARENT, true);
  calSpr.setTextDatum(MC_DATUM);

  char buf[32];
  sprintf(buf, "Cal: %.2f", calibrationFactor);

  calSpr.drawString(buf, calSprWidth / 2, calSprHeight / 2, 2);

  // Center horizontally, and vertically between origo and top
  int x = DIAL_CENTRE_X - calSprWidth / 2;
  int y = (DIAL_CENTRE_Y / 2 - calSprHeight / 2) + 20; // Adjusted for better placement
  calSpr.pushSprite(x, y);
}

void setup()
{
  pinMode(sensorPin, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(sensorPin), onPulse, FALLING); // or RISING depending on your sensor

  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(bg_color);

  drawDial();

  spr.loadFont(AA_FONT_LARGE);
  spr_width = spr.textWidth("15.0") + 18;            // More horizontal padding
  spr.createSprite(spr_width, spr.fontHeight() + 2); // More vertical padding
  spr.fillSprite(bg_color);

  createNeedle();
  plotNeedle(0.0);

  Serial.begin(115200);
  loadCalibration();
  showCalibration = true;
}

bool buttonPressed = false;

void loop()
{
  unsigned long currentMillis = millis();

  static bool lastButtonReading = HIGH;
  static unsigned long lastDebounceTime = 0;

  bool reading = digitalRead(buttonPin);

  if (reading != lastButtonReading)
  {
    lastDebounceTime = currentMillis;
    lastButtonReading = reading;
  }

  if ((currentMillis - lastDebounceTime) > debounceDelay && reading == LOW)
  {
    if (!buttonPressed)
    {
      // First time we detect the press
      calibrationFactor += 0.01;
      if (calibrationFactor > 1.5)
        calibrationFactor = 0.5;
      saveCalibration();
      buttonPressed = true;

      // Show calibration factor for 3 seconds
      showCalibration = true;
      calibrationDisplayMillis = millis();
    }
  }

  if (reading == HIGH)
  {
    buttonPressed = false; // Button released
  }

  if (currentMillis - lastDisplayUpdateTime >= 1000)
  {
    lastDisplayUpdateTime = currentMillis;
  }

  // --- Speed Calculation ---
  if (currentMillis - lastCalcTime >= 100)
  {
    unsigned long pulses = pulseCount - lastPulseCount;
    lastPulseCount = pulseCount;
    lastCalcTime = currentMillis;

    // Check if we've received any pulses
    if (pulses > 0) {
      lastPulseTime = currentMillis; // Update last activity time
    }

    float rps = (pulses / (float)PULSES_PER_REV) / 0.1f;
    float rpm = rps * 60.0f;
    float knots = rps * BASE_CONVERSION_FACTOR * calibrationFactor;
    float kmh = knots * 1.852f;
    float mph = kmh / 1.609344f;

    // Check for timeout - force to zero when no pulses for ZERO_TIMEOUT ms
    bool isZero = (currentMillis - lastPulseTime >= ZERO_TIMEOUT);
    if (isZero) {
      rpm = 0;
      knots = 0;
      kmh = 0;
      mph = 0;
    }

    // Apply smoothing to both RPM and knots
    float prevDisplayedRPM = displayedRPM;
    float prevDisplayedKnots = displayedKnots;
    
    displayedRPM += (rpm - displayedRPM) * smoothingFactor;
    displayedKnots += (knots - displayedKnots) * smoothingFactor;
    
    // Force to exactly zero when timeout occurs
    if (isZero && displayedRPM < 10) {
      displayedRPM = 0;
    }
    if (isZero && displayedKnots < 0.1) {
      displayedKnots = 0;
    }

    bool forceUpdate = ((currentMillis - lastDisplayUpdateTime) >= 1000); // Update at least every second

    // Only update display if values changed significantly or forced update
    if (abs(displayedKnots - prevDisplayedKnots) >= 0.05 || abs(round(displayedRPM) - round(prevDisplayedRPM)) >= 3 || forceUpdate || isZero)
    {
      printOtherValues(displayedRPM, kmh, mph);
      plotNeedle(displayedKnots);
      lastDisplayUpdateTime = currentMillis;
    }
  }

  if (showCalibration)
  {
    showCalibrationFactor();
    if (millis() - calibrationDisplayMillis > 3000)
    {
      showCalibration = false;
      // Redraw dial and needle to clear calibration text
      drawDial();
      plotNeedle(displayedKnots);
      spr.fillSprite(bg_color);}
  }
}
