// Lightweight UI drawing helpers
#pragma once

#include <TFT_eSPI.h>
#include <stdint.h>
#include <string.h>

// Centered text helper (with optional background)
static inline void drawTextCentered(TFT_eSprite& s, const char* str, int x, int y, int size, uint16_t fg, uint16_t bg = 0 /* black */) {
  s.setTextWrap(false);
  s.setTextColor(fg, bg);
  s.setTextSize(size);
  int w = (int)strlen(str) * 6 * size;
  int h = 8 * size;
  s.setCursor(x - w / 2, y - h / 2);
  s.print(str);
}

// Centered text helper variant used by UI screens
static inline void drawTextCentered2(TFT_eSprite& s, const char* label, int x, int y, int size, uint16_t col) {
  s.setTextColor(col, 0 /* black */);
  s.setTextSize(size);
  int w = (int)strlen(label) * 6 * size;
  int h = 8 * size;
  s.setCursor(x - w / 2, y - h / 2);
  s.print(label);
}

// Rounded rectangle primitive used by settings UI
static inline void drawRoundedRect(TFT_eSprite& s, int x, int y, int w, int h, int r, uint16_t color) {
  s.fillCircle(x + r, y + r, r, color);
  s.fillCircle(x + w - r - 1, y + r, r, color);
  s.fillCircle(x + r, y + h - r - 1, r, color);
  s.fillCircle(x + w - r - 1, y + h - r - 1, r, color);
  s.fillRect(x + r, y, w - 2 * r, h, color);
  s.fillRect(x, y + r, w, h - 2 * r, color);
}

