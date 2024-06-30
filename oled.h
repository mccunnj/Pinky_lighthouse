#ifndef OLED_H
#define OLED_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

extern Adafruit_SSD1306 oled;

void setupOLED();
void printSmall(const char* message);
void printFit(const char* message);
const char* reformatMessage(const char* originalMessage, uint8_t textSize);
void updateScroll();
void printFlash();
void printInput();

#endif