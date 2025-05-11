#include <TFT_eSPI.h>
#include "NotoSansBold36.h"
#define AA_FONT_LARGE NotoSansBold36

#define DIAL_CENTRE_X 120
#define DIAL_CENTRE_Y 120
#define DIAL_RADIUS   100
#define NEEDLE_LENGTH 70
#define NEEDLE_WIDTH  5

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite needle = TFT_eSprite(&tft);
TFT_eSprite spr    = TFT_eSprite(&tft);

uint16_t spr_width = 0;
uint16_t bg_color = TFT_BLACK;

void drawDial() {
  // Draw dial background
  tft.fillScreen(bg_color);
  tft.fillCircle(DIAL_CENTRE_X, DIAL_CENTRE_Y, DIAL_RADIUS, TFT_DARKGREY);
  tft.fillCircle(DIAL_CENTRE_X, DIAL_CENTRE_Y, DIAL_RADIUS-8, TFT_BLACK);

  // Draw ticks and numbers
  for (int i = 0; i <= 15; ++i) {
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
  tft.drawString("knots", DIAL_CENTRE_X, DIAL_CENTRE_Y + DIAL_RADIUS/2 + 30, 2);
}

void createNeedle() {
  needle.setColorDepth(16);
  needle.createSprite(NEEDLE_WIDTH, NEEDLE_LENGTH);
  needle.fillSprite(TFT_BLACK);
  needle.setPivot(NEEDLE_WIDTH/2, NEEDLE_LENGTH-2);
  needle.fillRect(0, 0, NEEDLE_WIDTH, NEEDLE_LENGTH, TFT_RED);
}

void plotNeedle(float value) {
  static float old_value = -1;

  // Erase old needle by overdrawing with dial center color
  if (old_value >= 0) {
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
  spr.drawString(buf, spr_width / 2 - 6, spr.fontHeight() / 2 + 4); // shift left, lower for better centering
  spr.pushSprite(DIAL_CENTRE_X - spr_width / 2 - 2, DIAL_CENTRE_Y - spr.fontHeight() / 2 + 54);

  old_value = value;
}

void setup() {
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(bg_color);

  drawDial();

  spr.loadFont(AA_FONT_LARGE);
  spr_width = spr.textWidth("15.0") + 18; // More horizontal padding
  spr.createSprite(spr_width, spr.fontHeight() + 2); // More vertical padding
  spr.fillSprite(bg_color);

  createNeedle();
  plotNeedle(0.0);
}

void loop() {
  for (float i = 0; i <= 15.0; i += 0.1) {
    plotNeedle(i);
    delay(50);
  }
  for (float i = 15.0; i >= 0.0; i -= 0.1) {
    plotNeedle(i);
    delay(50);
  }
}
