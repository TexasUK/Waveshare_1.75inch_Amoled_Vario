// main.cpp — Display + Audio (ES8311 + I2S) + Vario beeper + CST92xx touch + BMP280 baro

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <esp_err.h>
#include <Preferences.h>

#include "pins_config.h"

// Display includes
#ifndef TOUCH_CS
#define TOUCH_CS -1
#endif
#include <TFT_eSPI.h>

// Your custom display and touch headers
#include "display/CO5300.h"
#include "touch/TouchDrvCST92xx.h"

// Managers
#include "system/SettingsManager.h"
#include "driver/audio/AudioManager.h"

// FreeRTOS for audio task
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// I2S and codec
#include "driver/i2s.h"
#include "driver/audio/es8311.h"

// BMP280
#include <Adafruit_BMP280.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ===================================================================
// =================== Constants and Pin Definitions =================
// ===================================================================

// Display constants
static const int LCD_W   = 466;
static const int LCD_H   = 466;
static const int COL_OFF = 6;

// Primary I2C for touch/codec control (from pins_config.h if present)
#if defined(IIC_SDA)
#  define I2C_SDA_PIN IIC_SDA
#elif !defined(I2C_SDA_PIN)
#  define I2C_SDA_PIN 15
#endif

#if defined(IIC_SCL)
#  define I2C_SCL_PIN IIC_SCL
#elif !defined(I2C_SCL_PIN)
#  define I2C_SCL_PIN 14
#endif

#if defined(TOUCH_RST)
#  define TOUCH_RST_PIN TOUCH_RST
#else
#  define TOUCH_RST_PIN -1
#endif

#if defined(TOUCH_INT)
#  define TOUCH_INT_PIN TOUCH_INT
#else
#  define TOUCH_INT_PIN -1
#endif

// Secondary I2C for BMP280 (requested mapping)
#ifndef BARO_SDA_PIN
#  define BARO_SDA_PIN 18
#endif
#ifndef BARO_SCL_PIN
#  define BARO_SCL_PIN 17
#endif

// Touch orientation
#ifndef TOUCH_SWAP_XY
#  define TOUCH_SWAP_XY false
#endif
#ifndef TOUCH_MIRROR_X
#  define TOUCH_MIRROR_X true
#endif
#ifndef TOUCH_MIRROR_Y
#  define TOUCH_MIRROR_Y true
#endif

// Audio constants
static constexpr float KTS_TO_MPS = 0.514444f;

// I2S pins fallbacks
#ifndef BCLKPIN
#  define BCLKPIN 9
#endif
#ifndef WSPIN
#  define WSPIN 45
#endif
#ifndef DOPIN
#  define DOPIN 8
#endif
#ifndef MCLK_OUT_PIN
#  define MCLK_OUT_PIN 42
#endif
#ifndef PAPIN
#  define PAPIN 46
#endif

// ===================================================================
// =================== Global Objects ================================
// ===================================================================

TFT_eSPI    tft;
TFT_eSprite spr(&tft);
TouchDrvCST92xx g_touch;

// Managers
SettingsManager settingsManager;
AudioManager   audioManager;

// BMP280 on second I2C
Adafruit_BMP280 g_bmp(&Wire1);
bool            g_bmp_ok = false;
float           g_qnh_hpa = 1013.25f; // default QNH, tweak later via Settings

// Baro-derived state
static float    g_alt_m_last = NAN;     // last raw altitude (m)
static float    g_alt_m_lp   = NAN;     // low-pass altitude (m)
static float    g_v_mps      = 0.0f;    // filtered vertical speed (m/s)
static uint32_t g_baro_last_ms = 0;

// Colors
static const uint16_t C_BLACK  = 0x0000;
static const uint16_t C_WHITE  = 0xFFFF;
static const uint16_t C_RED    = 0xF800;
static const uint16_t C_GREEN  = 0x07E0;
static const uint16_t C_LGREY  = 0xC618;
static const uint16_t C_BLUE   = 0x001F;
static const uint16_t C_YELLOW = 0xFFE0;
static const uint16_t C_DARK_BLUE = 0x0010;

// Application state
bool     g_touch_initialized = false;
bool     g_in_settings = false;
uint32_t g_last_touch_time = 0;

// Settings screen state
struct SettingsScreenState {
  bool calibrating = false;
  int  selected_setting = 0;    // 0=volume, 1=brightness, 2=units, 3=calibration
} g_settings_screen;

// ===================================================================
// =================== Settings Slider ===============================
// ===================================================================

struct SliderState {
  bool active = false;
  int  value = 0;
  int  min_value = 0;
  int  max_value = 10;
  int  x_pos = 0;
  int  y_pos = 0;
  int  width = 0;
  int  height = 30;
} g_slider;

// ===================================================================
// =================== Touch System ==================================
// ===================================================================

static inline bool valid_gpio(int pin) { return pin >= 0 && pin <= 48; }

static bool touch_begin() {
  Serial.println("\n[TOUCH] Starting touch initialization.");
  delay(500);

  if (valid_gpio(TOUCH_RST_PIN)) {
    Serial.printf("[TOUCH] Hardware reset on pin %d\n", TOUCH_RST_PIN);
    pinMode(TOUCH_RST_PIN, OUTPUT);
    digitalWrite(TOUCH_RST_PIN, LOW);
    delay(50);
    digitalWrite(TOUCH_RST_PIN, HIGH);
    delay(200);
  }

  Serial.printf("[TOUCH] I2C0 (primary): SDA=%d, SCL=%d @400kHz\n", I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000U);
  delay(100);

  const uint8_t try_addrs[] = { 0x15, 0x5A, 0x38, 0x2E, 0x14 };
  bool touch_found = false;

  for (uint8_t a : try_addrs) {
    Serial.printf("[TOUCH] Trying address 0x%02X.", a);
    if (valid_gpio(TOUCH_RST_PIN)) {
      digitalWrite(TOUCH_RST_PIN, LOW); delay(5);
      digitalWrite(TOUCH_RST_PIN, HIGH); delay(50);
    }
    if (g_touch.begin(Wire, a, I2C_SDA_PIN, I2C_SCL_PIN)) {
      Serial.printf(" SUCCESS\n");
      touch_found = true;
      g_touch.setMaxCoordinates(LCD_W, LCD_H);
      g_touch.setSwapXY(TOUCH_SWAP_XY);
      g_touch.setMirrorXY(TOUCH_MIRROR_X, TOUCH_MIRROR_Y);
      Serial.printf("[TOUCH] Configured: %dx%d, SwapXY=%d, MirrorX=%d, MirrorY=%d\n",
                    LCD_W, LCD_H, TOUCH_SWAP_XY, TOUCH_MIRROR_X, TOUCH_MIRROR_Y);
      break;
    } else {
      Serial.printf(" FAILED\n");
    }
    delay(50);
  }

  if (!touch_found) {
    Serial.println("[TOUCH] No touch controller found!");
    return false;
  }

  Serial.println("[TOUCH] Touch initialized successfully");
  return true;
}

static bool touch_read_once(int16_t &x, int16_t &y) {
  if (!g_touch_initialized) return false;
  int16_t xs[5], ys[5];
  uint8_t n = g_touch.getSupportTouchPoint();
  if (!n) n = 1;
  uint8_t got = g_touch.getPoint(xs, ys, n);
  if (got) { x = xs[0]; y = ys[0]; g_last_touch_time = millis(); return true; }
  return false;
}

// ===================================================================
// =================== UI Constants ==================================
// ===================================================================

static const int   BAND_W          = 50;
static const int   GAP             = 5;
static const int   SHRINK          = 0;
static const float VARIO_MAX       = 10.0f;
static const float VARIO_GAM       = 0.6f;
static const int   RADIAL_TK_MAJOR = 6;
static const int   RADIAL_TK_MINOR = 5;
static const float CHEV_DEPTH      = 0.90f;
static const float CHEV_HALF       = 7.0f;
static const int   CHEV_TK         = 5;
static const float CHEV_STEP_DEG   = 0.20f;

static const int gap     = 4;
static const int bigW    = 64, bigH    = 116, bigSize   = 6;
static const int smallW  = 54, smallH  = 96,  smallSize = 5;
static const int splitX  = LCD_W / 2;
static const int RIGHT_HALF_MARGIN_L = 6;
static const int rowX   = splitX + RIGHT_HALF_MARGIN_L;
static const int rowY   = (LCD_H - bigH) / 2;
static const int ySmall = rowY + (bigH - smallH) / 2;

static const int x0 = rowX;
static const int x1 = x0 + bigW  + gap;
static const int x2 = x1 + smallW + gap;

static const int ASI_MIN = 45;
static const int ASI_MAX = 65;
static const int ASI_Y   = ySmall + smallH + 48;
static const int ASI_X   = rowX;

static const int ASI_FONT        = 6;
static const int ASI_KTS_FONT    = 4;
static const int ASI_CHAR_W      = 6 * ASI_FONT;
static const int ASI_CHAR_H      = 8 * ASI_FONT;
static const int ASI_KTS_CHAR_H  = 8 * ASI_KTS_FONT;
static const int ASI_FIELD_CHARS = 3;
static const int ASI_FIELD_W     = ASI_FIELD_CHARS * ASI_CHAR_W;
static const int ASI_GUTTER      = 0;
static const int ASI_LABEL_W     = 3 * (6 * ASI_KTS_FONT);
static const int ASI_LINE_W      = ASI_FIELD_W + ASI_GUTTER + ASI_LABEL_W;

// ===================================================================
// =================== UI Helper Functions ===========================
// ===================================================================

static inline void getBandRadii(int &rDisplay, int &rOuter, int &rInner, int &rCenter) {
  rDisplay = min(LCD_W, LCD_H) / 2 - max(0, SHRINK);
  if (rDisplay < 1) rDisplay = 1;
  rOuter  = max(1, rDisplay - max(0, GAP));
  rInner  = max(0, rOuter   - max(1, BAND_W));
  rCenter = (rOuter + rInner) / 2;
}

static inline float varioAngleDeg(float v) {
  if (v >  VARIO_MAX) v =  VARIO_MAX;
  if (v < -VARIO_MAX) v = -VARIO_MAX;
  float s = (v >= 0) ? +1.0f : -1.0f;
  float t = powf(fabsf(v) / VARIO_MAX, VARIO_GAM);
  float d = 90.0f * t;
  return 180.0f + s * d;
}

static inline void drawTextCentered(TFT_eSprite& s,const char* str,int x,int y,int size,uint16_t fg,uint16_t bg=C_BLACK){
  s.setTextWrap(false);
  s.setTextColor(fg, bg);
  s.setTextSize(size);
  int w = (int)strlen(str) * 6 * size;
  int h = 8 * size;
  s.setCursor(x - w/2, y - h/2);
  s.print(str);
}

// ===================================================================
// =================== Slider Functions ==============================
// ===================================================================

static void drawRoundedRect(TFT_eSprite& s, int x, int y, int w, int h, int r, uint16_t color) {
  s.fillCircle(x + r, y + r, r, color);
  s.fillCircle(x + w - r - 1, y + r, r, color);
  s.fillCircle(x + r, y + h - r - 1, r, color);
  s.fillCircle(x + w - r - 1, y + h - r - 1, r, color);
  s.fillRect(x + r, y, w - 2 * r, h, color);
  s.fillRect(x, y + r, w, h - 2 * r, color);
}

static void drawSlider() {
  int slider_radius = g_slider.height / 2;
  drawRoundedRect(spr, g_slider.x_pos, g_slider.y_pos, g_slider.width, g_slider.height, slider_radius, C_WHITE);
  float ratio = (float)(g_slider.value - g_slider.min_value) / (float)(g_slider.max_value - g_slider.min_value);
  int filled_width = (int)(ratio * (g_slider.width - 4));
  if (filled_width > 0) {
    drawRoundedRect(spr, g_slider.x_pos + 2, g_slider.y_pos + 2, filled_width, g_slider.height - 4, slider_radius - 2, C_BLUE);
  }
  int handle_x = g_slider.x_pos + (int)(ratio * (g_slider.width - 30));
  int handle_y = g_slider.y_pos + g_slider.height / 2;
  spr.fillCircle(handle_x + 15, handle_y, 12, C_WHITE);
  spr.fillCircle(handle_x + 15, handle_y, 10, C_BLUE);
  char value_str[16]; snprintf(value_str, sizeof(value_str), "%d", g_slider.value);
  drawTextCentered(spr, value_str, g_slider.x_pos + g_slider.width / 2, g_slider.y_pos + g_slider.height + 25, 3, C_WHITE);
}

static void updateSliderFromTouch(int16_t x, int16_t y) {
  if (x < g_slider.x_pos || x > g_slider.x_pos + g_slider.width) return;
  float ratio = (float)(x - g_slider.x_pos) / (float)g_slider.width;
  ratio = constrain(ratio, 0.0f, 1.0f);
  int new_value = g_slider.min_value + (int)(ratio * (g_slider.max_value - g_slider.min_value));
  if (new_value != g_slider.value) {
    g_slider.value = new_value;
    switch (g_settings_screen.selected_setting) {
      case 0: settingsManager.setVolume(g_slider.value); audioManager.setVolume(g_slider.value); break;
      case 1: settingsManager.setBrightness(map(g_slider.value, 0, 10, 0, 255)); lcd_brightness(settingsManager.settings.display_brightness); break;
    }
  }
}

static void setupSlider() {
  g_slider.x_pos = 40; g_slider.y_pos = 280; g_slider.width = LCD_W - 80; g_slider.active = true;
  switch (g_settings_screen.selected_setting) {
    case 0: g_slider.min_value = 0; g_slider.max_value = 10; g_slider.value = settingsManager.settings.audio_volume; break;
    case 1: g_slider.min_value = 0; g_slider.max_value = 10; g_slider.value = (int)map(settingsManager.settings.display_brightness, 0, 255, 0, 10); break;
    case 2: g_slider.active = false; break;
    case 3: g_slider.active = false; break;
  }
}

// ===================================================================
// =================== UI Classes ====================================
// ===================================================================

class Vario {
public:
  void update(float v) { v_ = v; }
  void draw(TFT_eSprite& s);  // defined after the class

private:
  float v_ = 0.f;

  // helpers (declared here, defined after the class)
  static void fillDiskVertical(TFT_eSprite& s, int cx, int cy, int r, uint16_t color);
  static void drawBand(TFT_eSprite& s);
  static void drawCaps(TFT_eSprite& s);
  static void drawRadialAt(TFT_eSprite& s, float angleDeg, uint16_t color, int thickness);
  static void drawRadials(TFT_eSprite& s);
  static void drawLabels(TFT_eSprite& s);
  void        drawChevron(TFT_eSprite& s) const;
  void        drawCenterValue(TFT_eSprite& s) const;
};

// -- Vario rendering orchestration (defined after helpers so all are visible)
void Vario::draw(TFT_eSprite& s) {
  drawBand(s);
  drawCaps(s);
  drawRadials(s);
  drawChevron(s);
  drawLabels(s);
  drawCenterValue(s);
}

void Vario::fillDiskVertical(TFT_eSprite& s, int cx, int cy, int r, uint16_t color) {
  if (r < 1) return;
  const long r2 = 1L * r * r;
  for (int x = cx - r; x <= cx + r; ++x) {
    if (x < 0 || x >= LCD_W) continue;
    long dx = x - cx;
    long term = r2 - dx * dx;
    if (term < 0) continue;
    int dy   = (int)floorf(sqrtf((float)term));
    int yTop = cy - dy;
    int h    = 2 * dy + 1;
    if (yTop < 0) { h += yTop; yTop = 0; }
    if (yTop + h > LCD_H) { h = LCD_H - yTop; }
    if (h > 0) { s.fillRect(x, yTop, 1, h, color); }
  }
}

void Vario::drawBand(TFT_eSprite& s) {
  const int cx = LCD_W / 2;
  const int cy = LCD_H / 2;
  int rDisplay, rOuter, rInner, rCenter;
  getBandRadii(rDisplay, rOuter, rInner, rCenter);

  const long rO2 = 1L * rOuter * rOuter;
  const long rI2 = 1L * rInner * rInner;

  int y0 = max(0, cy - rOuter);
  int y1 = min(LCD_H - 2, cy + rOuter);
  if (y0 & 1) y0++;

  for (int y = y0; y <= y1; y += 2) {
    long dy  = (long)y - cy;
    long dy2 = dy * dy;
    if (dy2 > rO2) continue;

    int halfOuter = (int)floorf((float)sqrt((double)(rO2 - dy2)));
    int halfInner = (dy2 < rI2) ? (int)floorf((float)sqrt((double)(rI2 - dy2))) : 0;

    int x = cx - halfOuter;
    int w = (cx - halfInner) - x;
    if (x < 0) { w += x; x = 0; }
    if (x + w > LCD_W) { w = LCD_W - x; }
    if (w > 0) { s.fillRect(x, y, w, 2, C_WHITE); }
  }
}

void Vario::drawCaps(TFT_eSprite& s) {
  const int cx = LCD_W / 2;
  const int cy = LCD_H / 2;
  int rDisplay, rOuter, rInner, rCenter;
  getBandRadii(rDisplay, rOuter, rInner, rCenter);

  int rCap = max(1, BAND_W / 2);
  fillDiskVertical(s, cx, cy - rCenter, rCap, C_WHITE);
  fillDiskVertical(s, cx, cy + rCenter, rCap, C_WHITE);
}

void Vario::drawRadialAt(TFT_eSprite& s, float angleDeg, uint16_t color, int thickness) {
  const int cx = LCD_W / 2;
  const int cy = LCD_H / 2;
  float rad = angleDeg * (float)M_PI / 180.0f;
  float ux = cosf(rad), uy = sinf(rad);

  int rDisplay, rOuter, rInner, rCenter;
  getBandRadii(rDisplay, rOuter, rInner, rCenter);

  int half = max(1, thickness / 2);
  for (int r = rInner; r <= rOuter; ++r) {
    int x = cx + (int)lroundf(ux * r);
    int y = cy + (int)lroundf(uy * r);
    int x0 = x - half, y0 = y - half, w = thickness, h = thickness;

    if (x0 < 0) { w += x0; x0 = 0; }
    if (y0 < 0) { h += y0; y0 = 0; }
    if (x0 + w > LCD_W)  { w = LCD_W  - x0; }
    if (y0 + h > LCD_H)  { h = LCD_H  - y0; }
    if (w > 0 && h > 0) { s.fillRect(x0, y0, w, h, color); }
  }
}

void Vario::drawRadials(TFT_eSprite& s) {
  drawRadialAt(s, varioAngleDeg(0.0f), C_BLACK, RADIAL_TK_MAJOR);
  const int ticks[] = {2, 4, 6, 8};
  for (int i = 0; i < 4; ++i) {
    int v = ticks[i];
    drawRadialAt(s, varioAngleDeg(+v), C_BLACK, RADIAL_TK_MINOR);
    drawRadialAt(s, varioAngleDeg(-v), C_BLACK, RADIAL_TK_MINOR);
  }
}

void Vario::drawChevron(TFT_eSprite& s) const {
  const int cx = LCD_W / 2;
  const int cy = LCD_H / 2;
  int rDisplay, rOuter, rInner, rCenter;
  getBandRadii(rDisplay, rOuter, rInner, rCenter);

  float mid = varioAngleDeg(v_);
  const float a0 = mid - CHEV_HALF;
  const float a1 = mid + CHEV_HALF;
  const int depthPx = (int)roundf(CHEV_DEPTH * BAND_W);
  int half = max(1, CHEV_TK / 2);

  for (float a = a0; a <= a1 + 1e-3f; a += CHEV_STEP_DEG) {
    float frac = 1.0f - fabsf(a - mid) / CHEV_HALF;
    if (frac < 0) frac = 0;
    int rEnd = rInner + (int)roundf(frac * depthPx);
    if (rEnd > rOuter) rEnd = rOuter;

    float rad = a * (float)M_PI / 180.0f;
    float ux = cosf(rad), uy = sinf(rad);

    for (int r = rInner; r <= rEnd; ++r) {
      int x = cx + (int)lroundf(ux * r);
      int y = cy + (int)lroundf(uy * r);
      int x0 = x - half, y0 = y - half, w = CHEV_TK, h = CHEV_TK;

      if (x0 < 0) { w += x0; x0 = 0; }
      if (y0 < 0) { h += y0; y0 = 0; }
      if (x0 + w > LCD_W)  { w = LCD_W  - x0; }
      if (y0 + h > LCD_H)  { h = LCD_H  - y0; }
      if (w > 0 && h > 0) { s.fillRect(x0, y0, w, h, C_RED); }
    }
  }
}

void Vario::drawLabels(TFT_eSprite& s) {
  const int cx = LCD_W / 2;
  const int cy = LCD_H / 2;
  int rDisplay, rOuter, rInner, rCenter;
  getBandRadii(rDisplay, rOuter, rInner, rCenter);

  const int rLab = rInner + BAND_W / 2;
  const int size = 4;
  char buf[8];

  // 0
  {
    float a = varioAngleDeg(0.0f) * (float)M_PI / 180.0f;
    int x = cx + (int)lroundf(cosf(a) * rLab);
    int y = cy + (int)lroundf(sinf(a) * rLab);
    strcpy(buf, "0");
    drawTextCentered(s, buf, x, y, size, C_BLACK);
  }

  const int ticks[] = {2, 4, 6, 8};
  for (int i = 0; i < 4; ++i) {
    int v = ticks[i];
    {
      float a = varioAngleDeg((float)+v) * (float)M_PI / 180.0f;
      int x = cx + (int)lroundf(cosf(a) * rLab);
      int y = cy + (int)lroundf(sinf(a) * rLab);
      snprintf(buf, sizeof(buf), "+%d", v);
      drawTextCentered(s, buf, x, y, size, C_BLACK);
    }
    {
      float a = varioAngleDeg((float)-v) * (float)M_PI / 180.0f;
      int x = cx + (int)lroundf(cosf(a) * rLab);
      int y = cy + (int)lroundf(sinf(a) * rLab);
      snprintf(buf, sizeof(buf), "-%d", v);
      drawTextCentered(s, buf, x, y, size, C_BLACK);
    }
  }
}

void Vario::drawCenterValue(TFT_eSprite& s) const {
  const int x = LCD_W / 4;
  const int y = LCD_H / 2;
  char buf[16];
  snprintf(buf, sizeof(buf), "%+.1f", v_);
  uint16_t color = (v_ >= 0.0f) ? C_GREEN : C_RED;
  drawTextCentered(s, buf, x, y, 6, color);
};

class Altimeter {
public:
  void update(float alt_ft) { alt_ = constrain(alt_ft, 0.f, 9999.f); }
  void draw(TFT_eSprite& s) {
    const int ft_i = (int)floorf(alt_ + 0.00001f);
    const int d1k  = (ft_i / 1000) % 10;
    const int d100 = (ft_i /  100) % 10;
    const int d10  = (ft_i /   10) % 10;

    const float unitsValue   = fmod(alt_, 10.0f);
    const float f10          = (unitsValue >= 9.0f) ? (unitsValue - 9.0f) : 0.0f;
    const float tensValue    = fmod(alt_ / 10.0f, 10.0f);
    const float f100         = (tensValue >= 9.0f && unitsValue >= 9.0f) ? (unitsValue - 9.0f) : 0.0f;
    const float hundredsVal  = fmod(alt_ / 100.0f, 10.0f);
    const float f1k          = (hundredsVal >= 9.0f && tensValue >= 9.0f && unitsValue >= 9.0f) ? (unitsValue - 9.0f) : 0.0f;

    drawColumn(s, x0, rowY,   bigW,   bigH,   bigSize,   d1k,  f1k);
    drawColumn(s, x1, ySmall, smallW, smallH, smallSize, d100, f100);
    drawColumn(s, x2, ySmall, smallW, smallH, smallSize, d10,  f10);

    s.setTextColor(C_LGREY, C_BLACK); s.setTextSize(2); s.setCursor(x2 + smallW + 10, ySmall + smallH - 20); s.print("ft");
  }
private:
  float alt_ = 0.f;
  static inline void frame(TFT_eSprite& s, int x, int y, int w, int h, int t=4) {
    if (t < 1) { t = 1; } if (t > w/2) { t = w/2; } if (t > h/2) { t = h/2; }
    s.fillRect(x,         y,         w, t, C_WHITE);
    s.fillRect(x,         y + h - t, w, t, C_WHITE);
    s.fillRect(x,         y + t,     t, h - 2*t, C_WHITE);
    s.fillRect(x + w - t, y + t,     t, h - 2*t, C_WHITE);
  }
  static inline bool fullyInside(int topY, int textSize, int innerY, int innerH) {
    const int chH = 8 * textSize; return (topY >= innerY) && (topY + chH <= innerY + innerH);
  }
  static void drawColumn(TFT_eSprite& s, int x, int y, int w, int h, int textSize, int currentDigit, float frac) {
    const int frameT = 4; const int ix = x + frameT, iy = y + frameT; const int iw = w - 2 * frameT, ih = h - 2 * frameT;
    s.fillRect(ix, iy, iw, ih, C_BLACK);
    int dCur  = (currentDigit % 10 + 10) % 10; int dPrev = (dCur + 9) % 10; int dNext = (dCur + 1) % 10;
    const int chW = 6 * textSize; const int chH = 8 * textSize; const int xText = ix + (iw - chW) / 2; const int yCenter = iy + (ih - chH) / 2;
    if (frac < 0) { frac = 0; } if (frac > 1) { frac = 1; } const int roll = (int)lroundf(frac * chH);
    const int yPrevTop = yCenter - chH - roll; const int yCurTop  = yCenter - roll; const int yNextTop = yCenter + chH - roll;
    s.setTextWrap(false); s.setTextColor(C_WHITE, C_BLACK); s.setTextSize(textSize);
    if (fullyInside(yPrevTop, textSize, iy, ih)) { s.setCursor(xText, yPrevTop); s.print(dPrev); }
    if (fullyInside(yCurTop,  textSize, iy, ih)) { s.setCursor(xText, yCurTop);  s.print(dCur);  }
    if (fullyInside(yNextTop, textSize, iy, ih)) { s.setCursor(xText, yNextTop); s.print(dNext); }
    frame(s, x, y, w, h, frameT);
  }
};

class ASI {
public:
  void begin() { ASI_NUM_X_ = ASI_X; ASI_KTS_X_ = ASI_NUM_X_ + ASI_FIELD_W + ASI_GUTTER; }
  void update(float kts) { kts_ = constrain(kts, (float)ASI_MIN, (float)ASI_MAX); }
  void draw(TFT_eSprite& s) {
    char fld[4]; snprintf(fld, sizeof(fld), "%3d", (int)roundf(kts_));
    s.fillRect(ASI_NUM_X_, ASI_Y, ASI_FIELD_W, ASI_CHAR_H, C_BLACK);
    s.setTextWrap(false); s.setTextColor(C_WHITE, C_BLACK); s.setTextSize(ASI_FONT);
    s.setCursor(ASI_NUM_X_, ASI_Y); s.print(fld);
    int ktsY = ASI_Y + (ASI_CHAR_H - ASI_KTS_CHAR_H) / 2;
    s.setTextSize(ASI_KTS_FONT); s.setCursor(ASI_KTS_X_, ktsY); s.print("kts");
  }
private:
  float kts_ = 55.0f; int ASI_NUM_X_ = 0; int ASI_KTS_X_ = 0;
};

// Instances of UI components
static Vario gVario;
static Altimeter gAlt;
static ASI gASI;

// ===================================================================
// =================== App Screens ===================================
// ===================================================================

struct SwipeState { bool active = false; int16_t x0 = 0, y0 = 0; int16_t xLast = 0, yLast = 0; uint32_t t0 = 0; } gSwipe;

static void drawMainScreen(float v, float alt_ft, float asi_kts_now) {
  spr.fillSprite(C_BLACK);
  gVario.update(v);  gVario.draw(spr);
  gAlt.update(alt_ft); gAlt.draw(spr);
  gASI.update(asi_kts_now); gASI.draw(spr);
  uint16_t touchColor = (millis() - g_last_touch_time < 1000) ? C_GREEN : C_RED; spr.fillCircle(LCD_W - 20, 20, 8, touchColor);
  spr.setTextColor(C_LGREY, C_BLACK); spr.setTextSize(1); spr.setCursor(10, LCD_H - 20); spr.print("Swipe L->R for Settings");
  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
}

static void drawSettingsScreen() {
  spr.fillSprite(C_BLACK);
  spr.setTextColor(C_YELLOW, C_BLACK); spr.setTextSize(4); drawTextCentered(spr, "SETTINGS", LCD_W/2, 30, 4, C_YELLOW);
  uint16_t text_color = C_WHITE;
  char volume_text[32]; snprintf(volume_text, sizeof(volume_text), "Audio Volume: %d", settingsManager.settings.audio_volume);
  text_color = (g_settings_screen.selected_setting == 0) ? C_BLUE : C_WHITE; drawTextCentered(spr, volume_text, LCD_W/2, 90, 3, text_color);
  char brightness_text[32]; snprintf(brightness_text, sizeof(brightness_text), "Brightness: %d", (int)map(settingsManager.settings.display_brightness, 0, 255, 0, 10));
  text_color = (g_settings_screen.selected_setting == 1) ? C_BLUE : C_WHITE; drawTextCentered(spr, brightness_text, LCD_W/2, 140, 3, text_color);
  char units_text[32]; snprintf(units_text, sizeof(units_text), "Units: %s", settingsManager.settings.use_metric ? "METRES" : "FEET");
  text_color = (g_settings_screen.selected_setting == 2) ? C_BLUE : C_WHITE; drawTextCentered(spr, units_text, LCD_W/2, 190, 3, text_color);
  text_color = (g_settings_screen.selected_setting == 3) ? C_BLUE : C_WHITE; drawTextCentered(spr, "Calibration", LCD_W/2, 240, 3, text_color);
  if (g_slider.active && (g_settings_screen.selected_setting == 0 || g_settings_screen.selected_setting == 1)) drawSlider();
  spr.setTextColor(C_LGREY, C_BLACK); spr.setTextSize(2); drawTextCentered(spr, "Swipe R->L to go back", LCD_W/2, LCD_H - 30, 2, C_LGREY);
  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
}

static void handleSettingsTouch(int16_t x, int16_t y) {
  if (g_slider.active && y >= g_slider.y_pos - 10 && y <= g_slider.y_pos + g_slider.height + 10) { updateSliderFromTouch(x, y); return; }
  if      (y >=  75 && y <= 105) { g_settings_screen.selected_setting = 0; setupSlider(); }
  else if (y >= 125 && y <= 155) { g_settings_screen.selected_setting = 1; setupSlider(); }
  else if (y >= 175 && y <= 205) { g_settings_screen.selected_setting = 2; settingsManager.setUseMetric(!settingsManager.settings.use_metric); g_slider.active = false; }
  else if (y >= 225 && y <= 255) { g_settings_screen.selected_setting = 3; g_settings_screen.calibrating = true; g_slider.active = false; }
}

// ===================================================================
// =================== BMP280 Helpers ================================
// ===================================================================

static void baro_begin() {
  Serial.printf("[BARO] I2C1 (secondary): SDA=%d, SCL=%d @400kHz\n", BARO_SDA_PIN, BARO_SCL_PIN);
  Wire1.begin(BARO_SDA_PIN, BARO_SCL_PIN, 400000U);

  const uint8_t addrs[] = {0x76, 0x77};
  for (uint8_t a : addrs) {
    Serial.printf("[BARO] Probing BMP280 @ 0x%02X... ", a);
    if (g_bmp.begin(a)) { Serial.println("FOUND"); g_bmp_ok = true; break; } else { Serial.println("nope"); }
  }
  if (!g_bmp_ok) { Serial.println("[BARO] ✗ No BMP280 detected"); return; }

// Configure sampling: normal mode, decent oversampling, strong IIR
g_bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X16,   // temperature oversampling
                  Adafruit_BMP280::SAMPLING_X2,    // pressure oversampling
                  Adafruit_BMP280::FILTER_X16      // IIR filter
                  /* standby left at library default (often 125 ms) */);
  Serial.println("[BARO] ✓ BMP280 ready");
}

static void baro_update() {
  if (!g_bmp_ok) return;
  uint32_t now = millis();
  if (g_baro_last_ms == 0) {
    g_baro_last_ms = now;
    (void)g_bmp.readPressure();
    g_alt_m_last = g_bmp.readAltitude(g_qnh_hpa);
    g_alt_m_lp = g_alt_m_last;
    return;
  }
  float dt = (now - g_baro_last_ms) * 0.001f; if (dt <= 0.0f) dt = 1e-3f;

  // Read latest sample(s)
  (void)g_bmp.readTemperature(); // not used yet; can trigger internal refresh on some variants
  float alt_m = g_bmp.readAltitude(g_qnh_hpa);
  if (!isnan(alt_m)) {
    // Low-pass altitude for altimeter
    const float tau_alt = 0.8f; // seconds
    float alpha_alt = dt / (tau_alt + dt);
    g_alt_m_lp = isnan(g_alt_m_lp) ? alt_m : (g_alt_m_lp + alpha_alt * (alt_m - g_alt_m_lp));

    // Differentiate raw altitude for vario, then low-pass the velocity
    float v_inst = (isnan(g_alt_m_last)) ? 0.0f : (alt_m - g_alt_m_last) / dt;
    const float tau_v = 0.40f; // seconds
    float beta_v = dt / (tau_v + dt);
    g_v_mps = g_v_mps + beta_v * (v_inst - g_v_mps);

    g_alt_m_last = alt_m;
  }
  g_baro_last_ms = now;
}

// ===================================================================
// =================== Arduino Lifecycle ==============================
// ===================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n\n[SYSTEM] Starting initialization.");

  settingsManager.begin();

  // Display
  Serial.println("[DISPLAY] Initializing display.");
  CO5300_init(); delay(500); lcd_setRotation(0);
  lcd_brightness(settingsManager.settings.display_brightness);
  spr.setColorDepth(16); spr.createSprite(LCD_W, LCD_H); spr.fillSprite(C_BLACK);
  spr.setTextColor(C_WHITE, C_BLACK); spr.setTextSize(3); spr.setCursor(LCD_W/2 - 80, LCD_H/2 - 20); spr.print("BOOTING.");
  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
  delay(1000);

  gASI.begin();
  Serial.println("[DISPLAY] Display ready");

  // Touch
  Serial.println("[TOUCH] Starting touch initialization.");
  g_touch_initialized = touch_begin();
  Serial.println(g_touch_initialized ? "[TOUCH] ✓ Touch initialized" : "[TOUCH] ✗ Touch initialization failed");

  // Audio
  Serial.println("[AUDIO] Initializing audio system.");
  if (!audioManager.begin(settingsManager.settings.audio_volume)) Serial.println("[AUDIO] ✗ Audio initialization failed");
  else Serial.println("[AUDIO] ✓ Audio system ready");

  // Baro
  baro_begin();

  Serial.println("[SYSTEM] Setup complete!");
}

void loop() {
  static uint32_t t0 = millis();
  static uint32_t last_debug = 0;
  static int16_t  last_touch_y = 0;
  float t = (millis() - t0) * 0.001f;

  // Update baro (if present)
  baro_update();

  // Signals
  float v;
  float alt_ft;
  if (g_bmp_ok) {
    v = g_v_mps;                                 // m/s for vario + audio
    alt_ft = (isnan(g_alt_m_lp) ? 0.0f : g_alt_m_lp) * 3.28084f;  // display in feet
  } else {
    // Fallback demo if no baro connected
    v = (5.0f * KTS_TO_MPS) * sinf(t * 0.6f);
    alt_ft = 3962.0f + 7.0f * sinf(t * 0.12f) + 0.5f * sinf(t * 0.6f);
  }

  static float asi_val = 55.f; // still demo for ASI until airspeed sensor is added
  static uint32_t lastAsi = 0;
  if (millis() - lastAsi > 100) {
    lastAsi = millis();
    int delta10 = random(-8, 9);
    asi_val += delta10 / 10.0f;
    asi_val = constrain(asi_val, 45.0f, 65.0f);
  }

  // Audio vario
  audioManager.setVario(v);

  // Touch & swipe
  if (g_touch_initialized) {
    int16_t x, y; bool pressed = touch_read_once(x, y); uint32_t now = millis();
    if (pressed && !gSwipe.active) {
      gSwipe.active = true; gSwipe.x0 = x; gSwipe.y0 = y; gSwipe.xLast = x; gSwipe.yLast = y; gSwipe.t0 = now; last_touch_y = y;
      if (g_in_settings) {
        if (g_settings_screen.calibrating) { g_settings_screen.calibrating = false; }
        else { handleSettingsTouch(x, y); }
      }
    } else if (pressed && gSwipe.active) {
      gSwipe.xLast = x; gSwipe.yLast = y;
    } else if (!pressed && gSwipe.active) {
      int dx = gSwipe.xLast - gSwipe.x0; int dy = gSwipe.yLast - gSwipe.y0; uint32_t dt = now - gSwipe.t0;
      bool isLeftToRight = (dx > 50) && (abs(dy) < 100) && (dt < 1000);
      bool isRightToLeft = (dx < -50) && (abs(dy) < 100) && (dt < 1000);
      gSwipe.active = false;
      if (isLeftToRight && !g_in_settings) { g_in_settings = true;  g_slider.active = false; }
      else if (isRightToLeft && g_in_settings) { g_in_settings = false; g_slider.active = false; }
    }
  }

  // Render
  if (g_in_settings) drawSettingsScreen(); else drawMainScreen(v, alt_ft, asi_val);

  // Debug
  if (millis() - last_debug > 5000) {
    last_debug = millis();
    Serial.printf("[STATUS] Touch:%s Screen:%s Baro:%s v=%.2f m/s alt=%.1fft\n",
                  g_touch_initialized ? "OK" : "FAIL",
                  g_in_settings ? "SETTINGS" : "MAIN",
                  g_bmp_ok ? "OK" : "-",
                  v, alt_ft);
  }

  delay(16);
}
