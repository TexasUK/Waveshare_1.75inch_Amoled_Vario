// ESP32-S3 — S3 Telemetry Consumer + Round Instrument UI + Polar TE Compensation
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <esp_err.h>
#include <Preferences.h>

// === Board/display/touch/audio ===
#include "pins_config.h"
#ifndef TOUCH_CS
#define TOUCH_CS -1
#endif
#include <TFT_eSPI.h>
#include "display/CO5300.h"
#include "touch/TouchDrvCST92xx.h"
#include "system/SettingsManager.h"
#include "driver/audio/AudioManager.h"
#include "driver/i2s.h"
#include "driver/audio/es8311.h"

// === CSV link (replaces LinkProtocol) ===
#include "CsvSerial.h"

// ===================================================================
// =================== Constants & Pins ===============================
// ===================================================================

static const int LCD_W   = 466;
static const int LCD_H   = 466;
static const int COL_OFF = 6;

// UART1 pins to the sensor (matches your working pair)
static constexpr uint32_t LINK_BAUD = 115200;
static constexpr int8_t   S3_RX = 44;     // display RX (from sensor TX=43)
static constexpr int8_t   S3_TX = 43;     // display TX (to   sensor RX=44)

// Hardware serial port for the link
HardwareSerial LinkUart(1); // UART1
CsvSerial      csv(LinkUart, S3_RX, S3_TX, LINK_BAUD);

// I2C for touch/codec control
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

#ifndef TOUCH_SWAP_XY
#  define TOUCH_SWAP_XY false
#endif
#ifndef TOUCH_MIRROR_X
#  define TOUCH_MIRROR_X true
#endif
#ifndef TOUCH_MIRROR_Y
#  define TOUCH_MIRROR_Y true
#endif

static inline bool valid_gpio(int pin) { return pin >= 0 && pin <= 48; }

// ===================================================================
// =================== Globals & Managers ============================
// ===================================================================

TFT_eSPI        tft;
TFT_eSprite     spr(&tft);
TouchDrvCST92xx g_touch;
SettingsManager settingsManager;
AudioManager    audioManager;
Preferences     ui_prefs;

// Swipe state for gesture detection
struct SwipeState { bool active=false; int16_t x0=0,y0=0; int16_t xLast=0,yLast=0; uint32_t t0=0; } gSwipe;

// ===================================================================
// =================== Polar Settings ================================
// ===================================================================

struct PolarSettings {
    bool teCompEnabled = true;
    int  selectedPolar = 0;
    char polarName[32] = "LS8-b";
    bool settingsChanged = false;
};

static PolarSettings polarSettings;
static int settingsPage = 0;  // 0=main, 3=polar
static int g_selected_setting = 0;

// Available polar names for UI
const char* polarNames[] = { "LS8-b", "DG-800", "ASG-29", "Discus" };
const int polarNameCount = 4;

// ===================================================================
// =================== Telemetry Processing ==========================
// ===================================================================

// We'll parse into CsvTlm (from CsvSerial.h)
static bool     filtersInit=false;
static uint32_t firstFrameMs=0;

// Vario smoothing
template<size_t N>
struct Ring {
  float d[N]; size_t idx=0, sz=0;
  void push(float v){ d[idx]=v; idx=(idx+1)%N; if(sz<N) sz++; }
  float median() const {
    if (!sz) return 0.0f;
    float tmp[N];
    for(size_t i=0;i<sz;++i) tmp[i]=d[i];
    for(size_t i=1;i<sz;++i){
      float v=tmp[i]; size_t j=i;
      while(j>0 && tmp[j-1]>v){ tmp[j]=tmp[j-1]; --j; }
      tmp[j]=v;
    }
    return (sz&1) ? tmp[sz/2] : 0.5f*(tmp[sz/2-1]+tmp[sz/2]);
  }
};
static Ring<9> varioMed;
static float    varioEma=0.0f, altEma=0.0f;

// Current telemetry values for UI
static float current_v_ui = 0.0f;
static float current_alt_ft = 0.0f;
static float current_asi_kts = 0.0f;
static int current_gps_sats = 0;
static int current_flight_mode = 0;

// ===================================================================
// =================== Touch & UI helpers ============================
// ===================================================================

static bool g_touch_initialized = false;
static uint32_t g_last_touch_time = 0;

static bool touch_begin() {
  Serial.println("\n[TOUCH] Starting touch initialization.");
  delay(300);

  if (valid_gpio(TOUCH_RST_PIN)) {
    pinMode(TOUCH_RST_PIN, OUTPUT);
    digitalWrite(TOUCH_RST_PIN, LOW);  delay(40);
    digitalWrite(TOUCH_RST_PIN, HIGH); delay(150);
  }
  Serial.printf("[TOUCH] I2C0: SDA=%d, SCL=%d @400kHz\n", I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000U);

  const uint8_t try_addrs[] = { 0x15, 0x5A, 0x38, 0x2E, 0x14 };
  for (uint8_t a : try_addrs) {
    if (g_touch.begin(Wire, a, I2C_SDA_PIN, I2C_SCL_PIN)) {
      g_touch.setMaxCoordinates(LCD_W, LCD_H);
      g_touch.setSwapXY(TOUCH_SWAP_XY);
      g_touch.setMirrorXY(TOUCH_MIRROR_X, TOUCH_MIRROR_Y);
      Serial.printf("[TOUCH] Controller @0x%02X\n", a);
      return true;
    }
  }
  Serial.println("[TOUCH] No controller found");
  return false;
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
// =================== UI (Vario/Alt/ASI) ============================
// ===================================================================

static const uint16_t C_BLACK  = 0x0000;
static const uint16_t C_WHITE  = 0xFFFF;
static const uint16_t C_RED    = 0xF800;
static const uint16_t C_GREEN  = 0x07E0;
static const uint16_t C_LGREY  = 0xC618;
static const uint16_t C_BLUE   = 0x001F;
static const uint16_t C_YELLOW = 0xFFE0;
static const uint16_t C_DGREY  = 0x7BEF;
static const uint16_t C_CYAN   = 0x07FF;
static const uint16_t C_ORANGE = 0xFC00;
static const uint16_t C_PURPLE = 0xF81F;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ===================================================================
// =================== Flight Mode Helpers ===========================
// ===================================================================

// Flight mode constants
enum FlightMode {
  MODE_CRUISE = 0,
  MODE_THERMAL = 1,
  MODE_CLIMB = 2,
  MODE_DESCENT = 3
};

// Get flight mode text
static const char* getFlightModeText(int mode) {
  switch (mode) {
    case MODE_CRUISE:  return "CRUISE";
    case MODE_THERMAL: return "THERMAL";
    case MODE_CLIMB:   return "CLIMB";
    case MODE_DESCENT: return "DESCENT";
    default:           return "UNKNOWN";
  }
}

// Get flight mode color
static uint16_t getFlightModeColor(int mode) {
  switch (mode) {
    case MODE_CRUISE:  return C_BLUE;
    case MODE_THERMAL: return C_ORANGE;
    case MODE_CLIMB:   return C_GREEN;
    case MODE_DESCENT: return C_RED;
    default:           return C_LGREY;
  }
}

// UI constants
static const int   BAND_W    = 50;
static const int   GAP       = 5;
static const int   SHRINK    = 0;
static const float VARIO_MAX = 10.0f;
static const float VARIO_GAM = 0.6f;

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

// ASI
static const int ASI_Y   = ySmall + smallH + 48;
static const int ASI_X   = rowX;
static const int ASI_FONT= 6, ASI_KTS_FONT=4;
static const int ASI_CHAR_W=6*ASI_FONT, ASI_CHAR_H=8*ASI_FONT;
static const int ASI_KTS_CHAR_H=8*ASI_KTS_FONT;
static const int ASI_FIELD_CHARS=3;
static const int ASI_FIELD_W=ASI_FIELD_CHARS*ASI_CHAR_W;
static const int ASI_GUTTER=0;

static inline void drawTextCentered(TFT_eSprite& s, const char* str, int x, int y, int size, uint16_t fg, uint16_t bg=C_BLACK) {
  s.setTextWrap(false); s.setTextColor(fg, bg); s.setTextSize(size);
  int w = (int)strlen(str) * 6 * size; int h = 8 * size;
  s.setCursor(x - w/2, y - h/2); s.print(str);
}

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

class Vario {
public:
  void update(float v) { v_ = v; }
  void draw(TFT_eSprite& s) {
    drawBand(s); drawCaps(s); drawRadials(s); drawChevron(s); drawLabels(s); drawCenterValue(s);
  }
private:
  float v_ = 0.f;
  static void fillDiskVertical(TFT_eSprite& s, int cx, int cy, int r, uint16_t color){
    if (r < 1) return;
    const long r2 = 1L * r * r;
    for (int x = cx - r; x <= cx + r; ++x) {
      if (x < 0 || x >= LCD_W) continue;
      long dx = x - cx; long term = r2 - dx * dx; if (term < 0) continue;
      int dy = (int)floorf(sqrtf((float)term)); int yTop = cy - dy; int h = 2 * dy + 1;
      if (yTop < 0) { h += yTop; yTop = 0; } if (yTop + h > LCD_H) { h = LCD_H - yTop; }
      if (h > 0) { s.fillRect(x, yTop, 1, h, C_WHITE); }
    }
  }
  static void drawBand(TFT_eSprite& s){
    const int cx = LCD_W / 2, cy = LCD_H / 2;
    int rDisplay, rOuter, rInner, rCenter; getBandRadii(rDisplay, rOuter, rInner, rCenter);
    const long rO2 = 1L * rOuter * rOuter, rI2 = 1L * rInner * rInner;
    int y0 = max(0, cy - rOuter), y1 = min(LCD_H - 2, cy + rOuter); if (y0 & 1) y0++;
    for (int y = y0; y <= y1; y += 2) {
      long dy = (long)y - cy, dy2 = dy * dy; if (dy2 > rO2) continue;
      int halfOuter = (int)floorf((float)sqrt((double)(rO2 - dy2)));
      int halfInner = (dy2 < rI2) ? (int)floorf((float)sqrt((double)(rI2 - dy2))) : 0;
      int x = cx - halfOuter; int w = (cx - halfInner) - x;
      if (x < 0) { w += x; x = 0; } if (x + w > LCD_W) { w = LCD_W - x; }
      if (w > 0) { s.fillRect(x, y, w, 2, C_WHITE); }
    }
  }
  static void drawCaps(TFT_eSprite& s){
    const int cx = LCD_W / 2, cy = LCD_H / 2;
    int rDisplay, rOuter, rInner, rCenter; getBandRadii(rDisplay, rOuter, rInner, rCenter);
    int rCap = max(1, BAND_W / 2);
    fillDiskVertical(s, cx, cy - rCenter, rCap, C_WHITE);
    fillDiskVertical(s, cx, cy + rCenter, rCap, C_WHITE);
  }
  static void drawRadialAt(TFT_eSprite& s, float aDeg, uint16_t color, int thickness){
    const int cx = LCD_W / 2, cy = LCD_H / 2;
    float rad = aDeg * (float)M_PI / 180.0f, ux = cosf(rad), uy = sinf(rad);
    int rDisplay, rOuter, rInner, rCenter; getBandRadii(rDisplay, rOuter, rInner, rCenter);
    int half = max(1, thickness / 2);
    for (int r = rInner; r <= rOuter; ++r) {
      int x = cx + (int)lroundf(ux * r), y = cy + (int)lroundf(uy * r);
      int x0=x-half, y0=y-half, w=thickness, h=thickness;
      if (x0<0){w+=x0; x0=0;} if (y0<0){h+=y0; y0=0;}
      if (x0+w>LCD_W){w=LCD_W-x0;} if (y0+h>LCD_H){h=LCD_H-y0;}
      if (w>0 && h>0) s.fillRect(x0,y0,w,h,color);
    }
  }
  static void drawRadials(TFT_eSprite& s){
    drawRadialAt(s, varioAngleDeg(0.0f), C_BLACK, 6);
    const int ticks[] = {2,4,6,8};
    for (int v: ticks){ drawRadialAt(s,varioAngleDeg(+v),C_BLACK,5); drawRadialAt(s,varioAngleDeg(-v),C_BLACK,5); }
  }
  void drawChevron(TFT_eSprite& s) const {
    const int cx = LCD_W / 2, cy = LCD_H / 2;
    int rDisplay, rOuter, rInner, rCenter; getBandRadii(rDisplay, rOuter, rInner, rCenter);
    float mid = varioAngleDeg(v_);
    const float a0 = mid - 7.0f, a1 = mid + 7.0f;
    const int depthPx = (int)roundf(0.90f * BAND_W);
    int half = max(1, 5 / 2);
    for (float a = a0; a <= a1 + 1e-3f; a += 0.20f) {
      float frac = 1.0f - fabsf(a - mid) / 7.0f; if (frac < 0) frac = 0;
      int rEnd = rInner + (int)roundf(frac * depthPx); if (rEnd > rOuter) rEnd = rOuter;
      float rad = a * (float)M_PI / 180.0f, ux = cosf(rad), uy = sinf(rad);
      for (int r = rInner; r <= rEnd; ++r) {
        int x = cx + (int)lroundf(ux*r), y = cy + (int)lroundf(uy*r);
        int x0=x-half, y0=y-half, w=5, h=5;
        if (x0<0){w+=x0; x0=0;} if (y0<0){h+=y0; y0=0;}
        if (x0+w>LCD_W){w=LCD_W-x0;} if (y0+h>LCD_H){h=LCD_H-y0;}
        if (w>0 && h>0) s.fillRect(x0,y0,w,h, (v_>=0?C_GREEN:C_RED) );
      }
    }
  }
  static void drawLabels(TFT_eSprite& s){
    const int cx = LCD_W / 2, cy = LCD_H / 2;
    int rDisplay, rOuter, rInner, rCenter; getBandRadii(rDisplay, rOuter, rInner, rCenter);
    const int rLab = rInner + BAND_W/2; const int size = 4;
    char buf[8];
    float a0 = varioAngleDeg(0.0f) * (float)M_PI / 180.0f;
    int x = cx + (int)lroundf(cosf(a0) * rLab), y = cy + (int)lroundf(sinf(a0) * rLab);
    strcpy(buf,"0"); drawTextCentered(s, buf, x, y, size, C_BLACK);
    const int ticks[] = {2,4,6,8};
    for (int v: ticks){
      float a = varioAngleDeg((float)+v)*(float)M_PI/180.0f;
      int x1 = cx + (int)lroundf(cosf(a)*rLab), y1 = cy + (int)lroundf(sinf(a)*rLab);
      snprintf(buf,sizeof(buf),"+%d",v); drawTextCentered(s, buf, x1, y1, size, C_BLACK);
      a = varioAngleDeg((float)-v)*(float)M_PI/180.0f;
      int x2 = cx + (int)lroundf(cosf(a)*rLab), y2 = cy + (int)lroundf(sinf(a)*rLab);
      snprintf(buf,sizeof(buf),"-%d",v); drawTextCentered(s, buf, x2, y2, size, C_BLACK);
    }
  }
  void drawCenterValue(TFT_eSprite& s) const {
    const int x = LCD_W / 4, y = LCD_H / 2;
    char buf[16]; snprintf(buf, sizeof(buf), "%+.1f", v_);
    drawTextCentered(s, buf, x, y, 6, C_WHITE);
  }
};

class Altimeter {
public:
  void update(float alt_ft) { alt_ = alt_ft; }
  void draw(TFT_eSprite& s) {
    const bool neg = (alt_ < 0);
    const float alt_abs = fabsf(alt_);
    const int ft_i = (int)floorf(alt_abs + 0.00001f);
    const int d1k  = (ft_i / 1000) % 10;
    const int d100 = (ft_i /  100) % 10;
    const int d10  = (ft_i /   10) % 10;

    const float unitsValue   = fmod(alt_abs, 10.0f);
    const float f10          = (unitsValue >= 9.0f) ? (unitsValue - 9.0f) : 0.0f;
    const float tensValue    = fmod(alt_abs / 10.0f, 10.0f);
    const float f100         = (tensValue >= 9.0f && unitsValue >= 9.0f) ? (unitsValue - 9.0f) : 0.0f;
    const float hundredsVal  = fmod(alt_abs / 100.0f, 10.0f);
    const float f1k          = (hundredsVal >= 9.0f && tensValue >= 9.0f && unitsValue >= 9.0f) ? (unitsValue - 9.0f) : 0.0f;

    if (neg) { s.setTextColor(C_WHITE, C_BLACK); s.setTextSize(3); s.setCursor(x0 - 14, rowY + bigH/2 - 8); s.print("-"); }

    drawColumn(s, x0, rowY,   bigW,   bigH,   6, d1k,  f1k);
    drawColumn(s, x1, ySmall, smallW, smallH, 5, d100, f100);
    drawColumn(s, x2, ySmall, smallW, smallH, 5, d10,  f10);

    s.setTextColor(C_LGREY, C_BLACK); s.setTextSize(2); s.setCursor(x2 + smallW + 10, ySmall + smallH - 20); s.print("ft");
  }
private:
  float alt_ = 0.f;
  static inline void frame(TFT_eSprite& s, int x, int y, int w, int h, int t=4) {
    if (t < 1) t = 1; if (t > w/2) t = w/2; if (t > h/2) t = h/2;
    s.fillRect(x,         y,         w, t, C_WHITE);
    s.fillRect(x,         y + h - t, w, t, C_WHITE);
    s.fillRect(x,         y + t,     t, h - 2*t, C_WHITE);
    s.fillRect(x + w - t, y + t,     t, h - 2*t, C_WHITE);
  }
  static inline bool fullyInside(int topY, int textSize, int innerY, int innerH) {
    const int chH = 8 * textSize; return (topY >= innerY) && (topY + chH <= innerY + innerH);
  }
  static void drawColumn(TFT_eSprite& s, int x, int y, int w, int h, int textSize, int currentDigit, float frac) {
    const int frameT = 4; const int ix = x + frameT, iy = y + frameT; const int iw = w - 2*frameT, ih = h - 2*frameT;
    s.fillRect(ix, iy, iw, ih, C_BLACK);
    int dCur  = (currentDigit % 10 + 10) % 10; int dPrev = (dCur + 9) % 10; int dNext = (dCur + 1) % 10;
    const int chW = 6 * textSize; const int chH = 8 * textSize; const int xText = ix + (iw - chW) / 2; const int yCenter = iy + (ih - chH) / 2;
    if (frac < 0) frac = 0; if (frac > 1) frac = 1; const int roll = (int)lroundf(frac * chH);
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
  void update(float kts) { kts_ = (kts < 0.0f ? 0.0f : kts); }
  void draw(TFT_eSprite& s) {
    char fld[4]; snprintf(fld, sizeof(fld), "%3d", (int)roundf(kts_));
    s.fillRect(ASI_NUM_X_, ASI_Y, ASI_FIELD_W, ASI_CHAR_H, C_BLACK);
    s.setTextWrap(false); s.setTextColor(C_WHITE, C_BLACK); s.setTextSize(ASI_FONT);
    s.setCursor(ASI_NUM_X_, ASI_Y); s.print(fld);
    int ktsY = ASI_Y + (ASI_CHAR_H - ASI_KTS_CHAR_H) / 2;
    s.setTextSize(ASI_KTS_FONT); s.setCursor(ASI_KTS_X_, ktsY); s.print("kts");
  }
private:
  float kts_ = 0.0f; int ASI_NUM_X_ = 0; int ASI_KTS_X_ = 0;
};

static Vario     gVario;
static Altimeter gAlt;
static ASI       gASI;

// ===================================================================
// =================== Polar Settings UI =============================
// ===================================================================

struct SliderState {
  bool active=false; int value=0; int min_value=0; int max_value=10; int x_pos=0; int y_pos=0; int width=0; int height=30;
} g_slider;

static void drawRoundedRect(TFT_eSprite& s, int x, int y, int w, int h, int r, uint16_t color) {
  s.fillCircle(x + r, y + r, r, color);
  s.fillCircle(x + w - r - 1, y + r, r, color);
  s.fillCircle(x + r, y + h - r - 1, r, color);
  s.fillCircle(x + w - r - 1, y + h - r - 1, r, color);
  s.fillRect(x + r, y, w - 2 * r, h, color);
  s.fillRect(x, y + r, w, h - 2 * r, color);
}

static void drawTextCentered2(TFT_eSprite& s, const char* label, int x, int y, int size, uint16_t col){
  s.setTextColor(col, C_BLACK); s.setTextSize(size);
  int w = strlen(label)*6*size, h=8*size;
  s.setCursor(x-w/2, y-h/2); s.print(label);
}

static void drawButton(TFT_eSprite& s, int x, int y, int w, int h, const char* label, uint16_t fg, uint16_t bg) {
  s.fillRect(x, y, w, h, bg);
  s.drawRect(x, y, w, h, C_WHITE);
  drawTextCentered2(s, label, x + w/2, y + h/2, 2, fg);
}

static void drawSlider() {
  int slider_radius = g_slider.height / 2;
  drawRoundedRect(spr, g_slider.x_pos, g_slider.y_pos, g_slider.width, g_slider.height, slider_radius, C_WHITE);
  float ratio = (float)(g_slider.value - g_slider.min_value) / (float)(g_slider.max_value - g_slider.min_value);
  int filled_width = (int)(ratio * (g_slider.width - 4));
  if (filled_width > 0) drawRoundedRect(spr, g_slider.x_pos + 2, g_slider.y_pos + 2, filled_width, g_slider.height - 4, slider_radius - 2, C_BLUE);
  int handle_x = g_slider.x_pos + (int)(ratio * (g_slider.width - 30));
  int handle_y = g_slider.y_pos + g_slider.height / 2;
  spr.fillCircle(handle_x + 15, handle_y, 12, C_WHITE);
  spr.fillCircle(handle_x + 15, handle_y, 10, C_BLUE);
  char value_str[16]; snprintf(value_str, sizeof(value_str), "%d", g_slider.value);
  drawTextCentered2(spr, value_str, g_slider.x_pos + g_slider.width / 2, g_slider.y_pos + g_slider.height + 25, 3, C_WHITE);
}

static void updateSliderFromTouch(int16_t x) {
  if (x < g_slider.x_pos || x > g_slider.x_pos + g_slider.width) return;
  float ratio = (float)(x - g_slider.x_pos) / (float)g_slider.width;
  ratio = constrain(ratio, 0.0f, 1.0f);
  g_slider.value = g_slider.min_value + (int)(ratio * (g_slider.max_value - g_slider.min_value));
}

static void setupSlider(int which) {
  g_slider.x_pos = 40; g_slider.y_pos = 280; g_slider.width = LCD_W - 80; g_slider.height = 30; g_slider.active = (which < 2);
  if      (which==0) { g_slider.min_value=0; g_slider.max_value=10; g_slider.value=settingsManager.settings.audio_volume; }
  else if (which==1) { g_slider.min_value=0; g_slider.max_value=10; g_slider.value=(int)map(settingsManager.settings.display_brightness,0,255,0,10); }
}

static bool g_in_settings = false;

// NEW: Draw main settings screen
static void drawMainSettingsScreen() {
  spr.fillSprite(C_BLACK);
  drawTextCentered2(spr, "SETTINGS", LCD_W/2, 30, 4, C_YELLOW);
  
  // Draw settings options with selection highlighting
  drawTextCentered2(spr, "Volume", LCD_W/2, 100, 3, 
                   g_selected_setting == 0 ? C_YELLOW : C_WHITE);
  drawTextCentered2(spr, "Brightness", LCD_W/2, 150, 3, 
                   g_selected_setting == 1 ? C_YELLOW : C_WHITE);
  drawTextCentered2(spr, "Polar Settings", LCD_W/2, 200, 3, 
                   g_selected_setting == 2 ? C_YELLOW : C_WHITE);
  drawTextCentered2(spr, "Toggle TE Comp", LCD_W/2, 250, 3, 
                   g_selected_setting == 3 ? C_YELLOW : C_WHITE);
  
  // Show current TE status
  drawTextCentered2(spr, polarSettings.teCompEnabled ? "ON" : "OFF", 
                   LCD_W/2, 280, 3, 
                   polarSettings.teCompEnabled ? C_GREEN : C_RED);
  
  // Draw slider if active
  if (g_slider.active) {
    drawSlider();
  }
  
  drawTextCentered2(spr, "Swipe LEFT to go back", LCD_W/2, LCD_H - 30, 2, C_LGREY);
  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
}

static void drawPolarSettingsScreen() {
  spr.fillSprite(C_BLACK);
  drawTextCentered2(spr, "POLAR SETTINGS", LCD_W/2, 30, 4, C_YELLOW);

  // TE Compensation toggle
  drawTextCentered2(spr, "TE Compensation:", LCD_W/2, 80, 2, C_WHITE);
  drawButton(spr, LCD_W/2 - 30, 100, 60, 30,
             polarSettings.teCompEnabled ? "ON" : "OFF",
             polarSettings.teCompEnabled ? C_GREEN : C_RED, C_BLACK);

  // Current polar display
  char polarBuf[64];
  snprintf(polarBuf, sizeof(polarBuf), "Current: %s", polarSettings.polarName);
  drawTextCentered2(spr, polarBuf, LCD_W/2, 120, 2, C_CYAN);

  // Polar selection buttons
  drawButton(spr, 50, 160, 120, 40, "LS8-b", C_WHITE, C_DGREY);
  drawButton(spr, 200, 160, 120, 40, "DG-800", C_WHITE, C_DGREY);
  drawButton(spr, 50, 210, 120, 40, "ASG-29", C_WHITE, C_DGREY);
  drawButton(spr, 200, 210, 120, 40, "Discus", C_WHITE, C_DGREY);

  // Back button
  drawButton(spr, LCD_W/2 - 80, 270, 160, 50, "<< BACK", C_YELLOW, C_RED);

  drawTextCentered2(spr, "Swipe LEFT to go back", LCD_W/2, LCD_H - 30, 2, C_LGREY);
  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
}

static void handlePolarTouch(int16_t x, int16_t y) {
  // TE Compensation toggle
  if (y >= 70 && y <= 100 && x >= (LCD_W/2 + 40) && x <= (LCD_W/2 + 100)) {
    polarSettings.teCompEnabled = !polarSettings.teCompEnabled;
    polarSettings.settingsChanged = true;
    return;
  }

  // Polar selection buttons
  if (y >= 160 && y <= 200) {
    if (x >= 50 && x <= 170) {
      polarSettings.selectedPolar = 0;
      strlcpy(polarSettings.polarName, "LS8-b", sizeof(polarSettings.polarName));
      polarSettings.settingsChanged = true;
    } else if (x >= 200 && x <= 320) {
      polarSettings.selectedPolar = 1;
      strlcpy(polarSettings.polarName, "DG-800", sizeof(polarSettings.polarName));
      polarSettings.settingsChanged = true;
    }
  } else if (y >= 210 && y <= 250) {
    if (x >= 50 && x <= 170) {
      polarSettings.selectedPolar = 2;
      strlcpy(polarSettings.polarName, "ASG-29", sizeof(polarSettings.polarName));
      polarSettings.settingsChanged = true;
    } else if (x >= 200 && x <= 320) {
      polarSettings.selectedPolar = 3;
      strlcpy(polarSettings.polarName, "Discus", sizeof(polarSettings.polarName));
      polarSettings.settingsChanged = true;
    }
  } else if (y >= 270 && y <= 320 && x >= (LCD_W/2 - 80) && x <= (LCD_W/2 + 80)) {
    // Back button - go to main settings
    settingsPage = 0;
    Serial.println("[TOUCH] Back to main settings from polar");
  }
}

static void sendPolarSettingsToSensor() {
  if (!polarSettings.settingsChanged) return;

  csv.sendSetPolar((uint8_t)polarSettings.selectedPolar);
  csv.sendSetTE(polarSettings.teCompEnabled);

  Serial.printf("[S3] Sent to SENSOR: Polar=%s (index %d), TE=%s\n",
                polarSettings.polarName, polarSettings.selectedPolar,
                polarSettings.teCompEnabled ? "ON" : "OFF");

  polarSettings.settingsChanged = false;
}

static void applySettingsChanges() {
  // Apply slider changes
  if (g_slider.active) {
    if (g_selected_setting == 0) { // Volume
      settingsManager.settings.audio_volume = g_slider.value;
      audioManager.setVolume(g_slider.value);
      settingsManager.save();
      Serial.printf("[SETTINGS] Volume set to: %d\n", g_slider.value);
      csv.sendSetVol((uint8_t)g_slider.value);
    } else if (g_selected_setting == 1) { // Brightness
      settingsManager.settings.display_brightness = (uint8_t)map((long)g_slider.value, 0L, 10L, 0L, 255L);
      lcd_brightness(settingsManager.settings.display_brightness);
      settingsManager.save();
      Serial.printf("[SETTINGS] Brightness set to: %d\n", settingsManager.settings.display_brightness);
      csv.sendSetBri((uint8_t)g_slider.value); // 0..10 scale
    }
  }

  // Send any pending polar settings
  sendPolarSettingsToSensor();
}

// ===================================================================
// =================== Main Display ==================================
// ===================================================================

static inline void drawMainScreen() {
  spr.fillSprite(C_BLACK);

  // Polar/TE status indicator
  spr.setTextColor(polarSettings.teCompEnabled ? C_GREEN : C_LGREY, C_BLACK);
  spr.setTextSize(1);
  spr.setCursor(10, 10);
  if (polarSettings.teCompEnabled) {
    spr.printf("TE: %s", polarSettings.polarName);
  } else {
    spr.print("NETTO");
  }

  // NEW: GPS status above altimeter
  spr.setTextSize(2);
  if (current_gps_sats < 3) {
    spr.setTextColor(C_RED, C_BLACK);
  } else {
    spr.setTextColor(C_GREEN, C_BLACK);
  }
  spr.setCursor(rowX + 20, rowY - 30);
  spr.print("GPS");

  // NEW: Flight mode display - prominent position at top center
  const char* modeText = getFlightModeText(current_flight_mode);
  uint16_t modeColor = getFlightModeColor(current_flight_mode);
  spr.setTextColor(modeColor, C_BLACK);
  spr.setTextSize(3);
  int modeTextWidth = strlen(modeText) * 6 * 3;
  spr.setCursor((LCD_W - modeTextWidth) / 2, 70);
  spr.print(modeText);

  // Mode-specific indicators
  spr.setTextSize(1);
  spr.setTextColor(C_LGREY, C_BLACK);
  switch (current_flight_mode) {
    case MODE_THERMAL:
      spr.setCursor(10, 100);
      spr.print("Thermal Entry/Exit");
      break;
    case MODE_CLIMB:
      spr.setCursor(10, 100);
      spr.print("Climbing");
      break;
    case MODE_DESCENT:
      spr.setCursor(10, 100);
      spr.print("Descending");
      break;
    case MODE_CRUISE:
      spr.setCursor(10, 100);
      spr.print("Cruising");
      break;
  }

  gVario.update(current_v_ui);  gVario.draw(spr);
  gAlt.update(current_alt_ft);  gAlt.draw(spr);
  gASI.update(current_asi_kts); gASI.draw(spr);

  uint16_t touchColor = (millis() - g_last_touch_time < 1000) ? C_GREEN : C_RED;
  spr.fillCircle(LCD_W - 20, 20, 8, touchColor);

  spr.setTextColor(C_LGREY, C_BLACK); spr.setTextSize(1);
  spr.setCursor(10, LCD_H - 20); spr.print("Swipe RIGHT for Settings");

  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
}

// NEW: Unified screen drawing function
static void drawCurrentScreen() {
  if (g_in_settings) {
    if (settingsPage == 0) {
      drawMainSettingsScreen();
    } else if (settingsPage == 3) {
      drawPolarSettingsScreen();
    }
  } else {
    drawMainScreen();
  }
}

// ===================================================================
// =================== App lifecycle =================================
// ===================================================================

static void handleTapGesture() {
  if (g_in_settings && settingsPage == 0) {
    if (gSwipe.yLast >= 80 && gSwipe.yLast <= 120) {
      g_selected_setting = 0; setupSlider(0);
    } else if (gSwipe.yLast >= 130 && gSwipe.yLast <= 170) {
      g_selected_setting = 1; setupSlider(1);
    } else if (gSwipe.yLast >= 180 && gSwipe.yLast <= 220) {
      g_selected_setting = 2; settingsPage = 3; Serial.println("[TOUCH] Going to polar settings");
    } else if (gSwipe.yLast >= 230 && gSwipe.yLast <= 270) {
      g_selected_setting = 3; polarSettings.teCompEnabled = !polarSettings.teCompEnabled; polarSettings.settingsChanged = true;
      Serial.printf("[TOUCH] TE toggled: %s\n", polarSettings.teCompEnabled ? "ON" : "OFF");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Reduce library chatter
  esp_log_level_set("*", ESP_LOG_WARN);

  Serial.println("\n[S3] Boot (CSV link, no LinkProtocol)");

  settingsManager.begin();

  // Load polar settings from preferences
  ui_prefs.begin("s3ui", true);
  polarSettings.teCompEnabled = ui_prefs.getBool("teEnabled", true);
  polarSettings.selectedPolar = ui_prefs.getUInt("polarIdx", 0);
  if (polarSettings.selectedPolar < polarNameCount) {
    strlcpy(polarSettings.polarName, polarNames[polarSettings.selectedPolar], sizeof(polarSettings.polarName));
  }
  ui_prefs.end();

  // Display
  CO5300_init(); delay(300); lcd_setRotation(0);
  lcd_brightness(settingsManager.settings.display_brightness);
  spr.setColorDepth(16); spr.createSprite(LCD_W, LCD_H); spr.fillSprite(C_BLACK);
  spr.setTextColor(C_WHITE, C_BLACK); spr.setTextSize(3); spr.setCursor(LCD_W/2 - 80, LCD_H/2 - 20); spr.print("BOOTING.");
  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
  delay(500);

  gASI.begin();

  // Touch
  g_touch_initialized = touch_begin();

  // Audio
  audioManager.begin(settingsManager.settings.audio_volume);

  // CSV link on UART1 (RX=44, TX=43)
  csv.begin();
  while (LinkUart.available()) (void)LinkUart.read(); // purge
  Serial.printf("[S3] Link (CSV) UART1: RX=%d, TX=%d @%u\n", S3_RX, S3_TX, LINK_BAUD);

  // Handshake + settings sync
  csv.onPong = [](){ Serial.println("[DISPLAY] got PONG ✔"); };
  csv.onHelloSensor = [](){ Serial.println("[DISPLAY] HELLO from sensor"); };
  csv.onAck = [](const char* key, long v){ Serial.printf("[DISPLAY] ACK %s=%ld\n", key, v); };

  csv.sendPing(); delay(20);
  csv.sendSetTE(polarSettings.teCompEnabled);
  csv.sendSetPolar((uint8_t)polarSettings.selectedPolar);
  csv.sendSetVol((uint8_t)settingsManager.settings.audio_volume);
  csv.sendSetBri((uint8_t)map((long)settingsManager.settings.display_brightness, 0L, 255L, 0L, 10L));

  Serial.printf("[S3] Polar: %s, TE: %s, Volume: %d, Brightness(0-10): %ld\n",
                polarSettings.polarName, polarSettings.teCompEnabled ? "ON" : "OFF",
                settingsManager.settings.audio_volume,
                map((long)settingsManager.settings.display_brightness, 0L, 255L, 0L, 10L));

  Serial.println("[S3] Ready.");
}

void loop() {
  static uint32_t last_debug = 0;
  static uint32_t telemetryCount = 0;
  static bool needsRedraw = true;

  // Service the CSV link
  csv.poll();

  // Throttled raw RX visibility (sanity)
  static uint32_t lastRaw = 0;
  if (millis() - lastRaw > 1000) {
    lastRaw = millis();
    Serial.printf("[LINK/RX] available=%d\n", LinkUart.available());
  }

  // Pull current telemetry from CSV
  CsvTlm m;
  bool got = csv.getLatest(m);
  if (got) {
    telemetryCount++;

    uint32_t now = millis();

    // First-frame init
    if (!filtersInit) {
      filtersInit = true;
      firstFrameMs = now;
      varioEma = 0.0f;
      altEma = m.alt_m;
      Serial.println("[S3] Filters initialized");
    }

    // Startup settling ~2s
    const bool settling = (now - firstFrameMs) < 2000;

    // Choose vario stream based on TE toggle
    float vClamp = polarSettings.teCompEnabled ? m.te : m.netto;

    // Clamp insane spikes
    if (vClamp > 20.0f) vClamp = 20.0f;
    if (vClamp < -20.0f) vClamp = -20.0f;

    varioMed.push(vClamp);
    float vMed = varioMed.median();
    const float aVar = settling ? 0.12f : 0.28f;
    varioEma = (1 - aVar) * varioEma + aVar * vMed;

    // Altitude EMA
    const float aAlt = 0.12f;
    altEma = (1 - aAlt) * altEma + aAlt * m.alt_m;

    // deadband for audio/needle stability
    const float V_DEADBAND = 0.15f;
    float v_disp = (fabsf(varioEma) < V_DEADBAND) ? 0.0f : varioEma;

    // UI-only smoothing
    current_v_ui = 0.85f * current_v_ui + 0.15f * v_disp;

    // Altitude (m -> ft)
    current_alt_ft = altEma * 3.28084f;

    // ASI (only when we have a GPS fix)
    const bool hasFix = (m.fix >= 1);
    current_asi_kts = hasFix ? (m.asi_kts < 0.0f ? 0.0f : m.asi_kts) : 0.0f;

    // GPS satellites
    current_gps_sats = m.sats;

    // Flight mode (from CsvTlm struct)
    current_flight_mode = m.mode;

    // Audio uses the de-deadbanded vario
    audioManager.setVario(v_disp);

    // Mark that we need to redraw
    needsRedraw = true;
  }

  // Draw the current screen if needed
  if (needsRedraw) {
    drawCurrentScreen();
    needsRedraw = false;
  }

  // Touch / navigation
  uint32_t now = millis();
  if (g_touch_initialized) {
    int16_t x, y;
    bool pressed = touch_read_once(x, y);

    if (pressed && !gSwipe.active) {
      gSwipe.active = true;
      gSwipe.x0 = x; gSwipe.y0 = y;
      gSwipe.xLast = x; gSwipe.yLast = y;
      gSwipe.t0 = now;
    }
    else if (pressed && gSwipe.active) {
      gSwipe.xLast = x; gSwipe.yLast = y;

      // Slider
      if (g_in_settings && settingsPage == 0 && g_slider.active) {
        if (y >= g_slider.y_pos && y <= g_slider.y_pos + g_slider.height) {
          updateSliderFromTouch(x);
          needsRedraw = true;
        }
      }
      // Polar page
      if (g_in_settings && settingsPage == 3) {
        handlePolarTouch(x, y);
        needsRedraw = true;
      }
    }
    else if (!pressed && gSwipe.active) {
      int dx = gSwipe.xLast - gSwipe.x0;
      int dy = gSwipe.yLast - gSwipe.y0;
      uint32_t dtg = now - gSwipe.t0;

      bool isSwipe = (abs(dx) > 50) && (abs(dy) < 40) && (dtg < 1000);
      bool isLeftSwipe = isSwipe && (dx < 0);
      bool isRightSwipe = isSwipe && (dx > 0);
      bool isTap = (abs(dx) < 20 && abs(dy) < 20 && dtg < 400);

      Serial.printf("[TOUCH] End: dx=%d, Lswipe=%d, Rswipe=%d, tap=%d\n",
                    dx, isLeftSwipe, isRightSwipe, isTap);

      gSwipe.active = false;

      if (isRightSwipe && !g_in_settings) {
        g_in_settings = true;
        settingsPage = 0;
        g_slider.active = false;
        needsRedraw = true;
        Serial.println("[TOUCH] Entering settings");
      }
      else if (isLeftSwipe && g_in_settings) {
        applySettingsChanges();
        g_in_settings = false;
        g_slider.active = false;
        needsRedraw = true;
        Serial.println("[TOUCH] Exiting settings");
      }
      else if (isTap) {
        if (g_in_settings) {
          handleTapGesture();
          needsRedraw = true;
        }
      }
    }
  }

  // Send polar settings periodically if changed
  static uint32_t lastPolarSend = 0;
  if (polarSettings.settingsChanged && (now - lastPolarSend > 1000)) {
    sendPolarSettingsToSensor();
    lastPolarSend = now;

    // Save to preferences
    ui_prefs.begin("s3ui", false);
    ui_prefs.putBool("teEnabled", polarSettings.teCompEnabled);
    ui_prefs.putUInt("polarIdx", (uint32_t)polarSettings.selectedPolar);
    ui_prefs.end();

    Serial.printf("[S3-POLAR] Sent to SENSOR: %s, TE=%s\n",
                  polarSettings.polarName, polarSettings.teCompEnabled ? "ON" : "OFF");
  }

  // Debug heartbeat
  if (now - last_debug > 5000) {
    last_debug = now;
    Serial.printf("[S3-STATUS] TE=%s polar=%s frames=%lu mode=%d(%s)\n",
                  polarSettings.teCompEnabled ? "ON" : "OFF",
                  polarSettings.polarName, (unsigned long)telemetryCount,
                  current_flight_mode, getFlightModeText(current_flight_mode));
  }

  delay(16);
}