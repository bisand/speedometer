// #include <Arduino.h>
// #include <Adafruit_GC9A01A.h>
// #include <Adafruit_GFX.h>
// #include "lvgl.h"

// #define TFT_CS    D8  // GPIO15
// #define TFT_DC    D1  // GPIO5
// #define TFT_RST   D2  // GPIO4
// #define INTERRUPT_PIN D3

// Adafruit_GC9A01A tft(TFT_CS, TFT_DC, TFT_RST);

// volatile byte rpmSignals;
// unsigned int rpm;
// unsigned long lastCalcTime = 0;

// lv_obj_t *rpm_label;

// void IRAM_ATTR magnet_detect()
// {
//   rpmSignals++;
// }

// // LVGL flush callback for Adafruit_GC9A01A
// void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
// {
//   tft.startWrite();
//   for (int y = area->y1; y <= area->y2; y++) {
//     tft.setAddrWindow(area->x1, y, area->x2 - area->x1 + 1, 1);
//     tft.writePixels((uint16_t *)color_p, area->x2 - area->x1 + 1, true);
//     color_p += (area->x2 - area->x1 + 1) * sizeof(lv_color_t);
//   }
//   tft.endWrite();
//   lv_display_flush_ready(disp);
// }

// void setup()
// {
//   Serial.begin(115200);
//   pinMode(INTERRUPT_PIN, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), magnet_detect, RISING);
//   rpmSignals = 0;
//   rpm = 0;
//   lastCalcTime = millis();

//   tft.begin();
//   tft.setRotation(2);

//   lv_init();

//   static lv_color_t buf[240 * 40];
//   static lv_draw_buf_t draw_buf;
//   lv_draw_buf_init(&draw_buf, buf, NULL, 240 * 40);

//   lv_display_t *disp = lv_display_create(240, 240);
//   lv_display_set_flush_cb(disp, my_disp_flush);
//   lv_display_set_draw_buf(disp, &draw_buf);

//   rpm_label = lv_label_create(lv_scr_act());
//   lv_obj_align(rpm_label, LV_ALIGN_CENTER, 0, 0);
//   lv_label_set_text(rpm_label, "RPM: 0");
// }

// void loop()
// {
//   unsigned long now = millis();

//   if (now - lastCalcTime >= 100)
//   {
//     noInterrupts();
//     unsigned long signals = rpmSignals;
//     rpmSignals = 0;
//     interrupts();

//     float revolutions = signals / 3.0;
//     rpm = (unsigned int)((revolutions * 60000.0) / (now - lastCalcTime));
//     lastCalcTime = now;

//     static char buf[16];
//     snprintf(buf, sizeof(buf), "RPM: %u", rpm);
//     lv_label_set_text(rpm_label, buf);
//   }

//   lv_timer_handler(); // LVGL task handler
//   delay(5);
// }
