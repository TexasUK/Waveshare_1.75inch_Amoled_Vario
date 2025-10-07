// ESP32-S3 — C3 Telemetry Consumer + Round Instrument UI + Polar TE Compensation
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

// === Link (updated with TE vario) ===
#include "link/LinkProtocol.h"

// ===================================================================
// =================== Constants & Pins ===============================
// ===================================================================

static const int LCD_W   = 466;
static const int LCD_H   = 466;
static const int COL_OFF = 6;

// S3 header UART pins to C3 link
static constexpr uint32_t LINK_BAUD = 115200;
static constexpr int8_t   S3_RX = 44;
static constexpr int8_t   S3_TX = 43;

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

// ===================================================================
// =================== Globals & Managers ============================
// ===================================================================

TFT_eSPI        tft;
TFT_eSprite     spr(&tft);
TouchDrvCST92xx g_touch;
SettingsManager settingsManager;
AudioManager    audioManager;
Preferences     ui_prefs;

// S3 needs its own LinkProtocol instance
LinkProtocol    g_link;

// ADD THIS LINE - Swipe state for gesture detection
struct SwipeState { bool active=false; int16_t x0=0,y0=0; int16_t xLast=0,yLast=0; uint32_t t0=0; } gSwipe;

static inline bool valid_gpio(int pin) { return pin >= 0 && pin <= 48; }

// ===================================================================
// =================== Polar Settings ================================
// ===================================================================

struct PolarSettings {
    bool teCompEnabled = true;
    int selectedPolar = 0;
    char polarName[32] = "LS8-b";
    bool settingsChanged = false;
};

static PolarSettings polarSettings;
static int settingsPage = 0;  // 0=main, 1=audio, 2=display, 3=polar
static int g_selected_setting = 0;

// Available polar names for UI
const char* polarNames[] = {
    "LS8-b", "DG-800", "ASG-29", "Discus"
};
const int polarNameCount = 4;

// ===================================================================
// =================== Minimal Link RX (SLIP + CRC) ==================
// ===================================================================

#ifndef LP_SLIP_FEND
  #define LP_SLIP_FEND  0xC0
#endif
#ifndef LP_SLIP_FESC
  #define LP_SLIP_FESC  0xDB
#endif
#ifndef LP_SLIP_TFEND
  #define LP_SLIP_TFEND 0xDC
#endif
#ifndef LP_SLIP_TFESC
  #define LP_SLIP_TFESC 0xDD
#endif

static uint16_t crc16_ccitt(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF) {
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int i = 0; i < 8; ++i)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// Public telemetry globals
TelemetryMsg  gTlm;
TelemetryMsg  gTlmRaw;
volatile bool gTlmFresh = false;

// RX state / stats
static const size_t RX_MAX = 256;
static uint8_t  rxbuf[RX_MAX];
static size_t   rxlen = 0;
static bool     in_frame = false;
static bool     esc = false;
static uint32_t framesOk=0, framesCrc=0, framesUnknown=0, bytesRx=0;

// Vario smoothing
template<size_t N>
struct Ring {
  float d[N]; size_t idx=0, sz=0;
  void push(float v){ d[idx]=v; idx=(idx+1)%N; if(sz<N) sz++; }
  float median() const {
    if (!sz) return 0.0f;
    float tmp[N];
    for(size_t i=0;i<sz;++i) tmp[i]=d[i];
    for(size_t i=1;i<sz;++i){ float v=tmp[i]; size_t j=i; while(j>0 && tmp[j-1]>v){ tmp[j]=tmp[j-1]; --j; } tmp[j]=v; }
    return (sz&1) ? tmp[sz/2] : 0.5f*(tmp[sz/2-1]+tmp[sz/2]);
  }
};
static Ring<9> varioMed;
static bool     filtersInit=false;
static uint32_t firstFrameMs=0;
static float    varioEma=0.0f, altEma=0.0f;

static void reset_frame(){ in_frame=false; esc=false; rxlen=0; }

static void process_frame(){
  if (rxlen < 3) return;
  const uint8_t  type = rxbuf[0];
  const size_t   plen = rxlen - 1 - 2;
  const uint16_t got  = (uint16_t)rxbuf[1+plen] | ((uint16_t)rxbuf[1+plen+1] << 8);
  uint16_t calc = crc16_ccitt(rxbuf, 1+plen);
  if (calc != got) { framesCrc++; return; }

  if (type == FT_TELEMETRY && plen >= sizeof(TelemetryMsg)) {
    TelemetryMsg m; memcpy(&m, &rxbuf[1], sizeof(TelemetryMsg));
    gTlmRaw = m;

    const uint32_t now = millis();
    if (!filtersInit) { filtersInit=true; firstFrameMs=now; varioEma=0.0f; altEma=m.alt_m; }

    // Startup settling ~2s
    const bool settling = (now - firstFrameMs) < 2000;

    // Clamp insane vario spikes
    float vClamp = polarSettings.teCompEnabled ? m.te_vario_mps : m.vario_mps;
    if (vClamp > 20.0f) vClamp=20.0f; if (vClamp < -20.0f) vClamp=-20.0f;
    varioMed.push(vClamp);
    float vMed = varioMed.median();
    const float aVar = settling ? 0.12f : 0.28f;
    varioEma = (1-aVar)*varioEma + aVar*vMed;

    // Altitude EMA
    const float aAlt = 0.12f;
    altEma = (1-aAlt)*altEma + aAlt*m.alt_m;

    TelemetryMsg f = m;
    f.vario_mps = settling ? 0.0f : varioEma;
    f.alt_m     = altEma;
    if (f.asi_kts < 2.0f) f.track_deg = NAN;

    gTlm = f;
    gTlmFresh = true;
    framesOk++;
  } else {
    framesUnknown++;
  }
}

static void link_poll() {
  while (Serial1.available()) {
    uint8_t b=(uint8_t)Serial1.read(); bytesRx++;
    if (!in_frame) { if (b==LP_SLIP_FEND){ in_frame=true; rxlen=0; esc=false; } continue; }
    if (b==LP_SLIP_FEND){ if (rxlen>=3) process_frame(); reset_frame(); continue; }
    if (esc){ if (b==LP_SLIP_TFEND) b=LP_SLIP_FEND; else if (b==LP_SLIP_TFESC) b=LP_SLIP_TFESC; else { reset_frame(); continue; } esc=false; }
    else if (b==LP_SLIP_FESC){ esc=true; continue; }
    if (rxlen<RX_MAX) rxbuf[rxlen++]=b; else reset_frame();
  }
}

// ===================================================================
// =================== Touch & UI helpers ============================
// ===================================================================

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

static bool g_touch_initialized = false;
static uint32_t g_last_touch_time = 0;

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
static const int ASI_MIN = 45;
static const int ASI_MAX = 65;
static const int ASI_Y   = ySmall + smallH + 48;
static const int ASI_X   = rowX;
static const int ASI_FONT= 6, ASI_KTS_FONT=4;
static const int ASI_CHAR_W=6*ASI_FONT, ASI_CHAR_H=8*ASI_FONT;
static const int ASI_KTS_CHAR_H=8*ASI_KTS_FONT;
static const int ASI_FIELD_CHARS=3;
static const int ASI_FIELD_W=ASI_FIELD_CHARS*ASI_CHAR_W;
static const int ASI_GUTTER=0;
static const int ASI_LABEL_W=3*(6*ASI_KTS_FONT);

// UI helpers
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

static inline void drawTextCentered(TFT_eSprite& s, const char* str, int x, int y, int size, uint16_t fg, uint16_t bg=C_BLACK) {
  s.setTextWrap(false); s.setTextColor(fg, bg); s.setTextSize(size);
  int w = (int)strlen(str) * 6 * size; int h = 8 * size;
  s.setCursor(x - w/2, y - h/2); s.print(str);
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

void drawPolarSettingsScreen() {
  spr.fillSprite(C_BLACK);
  drawTextCentered2(spr, "POLAR SETTINGS", LCD_W/2, 30, 4, C_YELLOW);
  
  // TE Compensation toggle
  drawTextCentered2(spr, "TE Compensation:", LCD_W/2 - 80, 80, 2, C_WHITE);
  drawButton(spr, LCD_W/2 + 40, 70, 60, 30, 
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
  
  // Larger, more prominent back button
  drawButton(spr, LCD_W/2 - 80, 270, 160, 50, "<< BACK", C_YELLOW, C_RED);
  
  drawTextCentered2(spr, "Swipe LEFT to go back", LCD_W/2, LCD_H - 30, 2, C_LGREY);
  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
}

void handlePolarTouch(int16_t x, int16_t y) {
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

void sendPolarSettingsToC3() {
    if (!polarSettings.settingsChanged) return;
    
    // Send polar selection to C3
    g_link.sendPolarSelect(polarSettings.selectedPolar);
    
    // Send TE toggle to C3  
    g_link.sendTEToggle(polarSettings.teCompEnabled);
    
    Serial.printf("[S3] Sent to C3: Polar=%s (index %d), TE=%s\n",
                  polarSettings.polarName, polarSettings.selectedPolar,
                  polarSettings.teCompEnabled ? "ON" : "OFF");
    
    polarSettings.settingsChanged = false;
}

void applySettingsChanges() {
    // Apply slider changes
    if (g_slider.active) {
        if (g_selected_setting == 0) { // Volume
            settingsManager.settings.audio_volume = g_slider.value;
            audioManager.setVolume(g_slider.value);
            settingsManager.save();
            Serial.printf("[SETTINGS] Volume set to: %d\n", g_slider.value);
        } else if (g_selected_setting == 1) { // Brightness
            settingsManager.settings.display_brightness = map(g_slider.value, 0, 10, 0, 255);
            lcd_brightness(settingsManager.settings.display_brightness);
            settingsManager.save();
            Serial.printf("[SETTINGS] Brightness set to: %d\n", settingsManager.settings.display_brightness);
        }
    }
    
    // Send any pending polar settings
    sendPolarSettingsToC3();
}

void handleTapGesture() {
    if (g_in_settings) {
        if (settingsPage == 0) { // Main settings
            // Determine which item was tapped
            if (gSwipe.yLast >= 80 && gSwipe.yLast <= 120) {
                g_selected_setting = 0;
                setupSlider(0); // Volume slider
            } else if (gSwipe.yLast >= 130 && gSwipe.yLast <= 170) {
                g_selected_setting = 1;
                setupSlider(1); // Brightness slider
            } else if (gSwipe.yLast >= 180 && gSwipe.yLast <= 220) {
                g_selected_setting = 2;
                settingsPage = 3; // Go directly to polar settings
                Serial.println("[TOUCH] Going to polar settings");
            } else if (gSwipe.yLast >= 230 && gSwipe.yLast <= 270) {
                g_selected_setting = 3;
                polarSettings.teCompEnabled = !polarSettings.teCompEnabled;
                polarSettings.settingsChanged = true;
                Serial.printf("[TOUCH] TE toggled: %s\n", polarSettings.teCompEnabled ? "ON" : "OFF");
            }
        }
    }
}

void drawSettingsScreen() {
  switch(settingsPage) {
    case 0: // Main settings
      spr.fillSprite(C_BLACK);
      drawTextCentered2(spr, "SETTINGS", LCD_W/2, 30, 4, C_YELLOW);

      char volume_text[32];  snprintf(volume_text,  sizeof(volume_text),  "Audio Volume: %d", settingsManager.settings.audio_volume);
      char bright_text[32];  snprintf(bright_text,  sizeof(bright_text),  "Brightness: %d", (int)map(settingsManager.settings.display_brightness,0,255,0,10));
      char polar_text[32];   snprintf(polar_text,   sizeof(polar_text),   "Polar: %s", polarSettings.polarName);
      char te_text[32];      snprintf(te_text,      sizeof(te_text),      "TE: %s", polarSettings.teCompEnabled ? "ON" : "OFF");

      drawTextCentered2(spr, volume_text, LCD_W/2,  90, 3, (g_selected_setting==0)?C_BLUE:C_WHITE);
      drawTextCentered2(spr, bright_text, LCD_W/2, 140, 3, (g_selected_setting==1)?C_BLUE:C_WHITE);
      drawTextCentered2(spr, polar_text,  LCD_W/2, 190, 3, (g_selected_setting==2)?C_BLUE:C_WHITE);
      drawTextCentered2(spr, te_text,     LCD_W/2, 240, 3, (g_selected_setting==3)?C_BLUE:C_WHITE);

      // Draw slider if active
      if (g_slider.active) drawSlider();

      spr.setTextColor(C_LGREY, C_BLACK); spr.setTextSize(2);
      drawTextCentered2(spr, "Swipe LEFT to go back", LCD_W/2, LCD_H - 30, 2, C_LGREY);
      break;
      
    case 3: // Polar settings
      drawPolarSettingsScreen();
      break;
  }
  
  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
}

// ===================================================================
// =================== Main Display ==================================
// ===================================================================

static inline void drawMainScreen(float v_ui, float alt_ft, float asi_kts_now) {
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
  
  gVario.update(v_ui);  gVario.draw(spr);
  gAlt.update(alt_ft);  gAlt.draw(spr);
  gASI.update(asi_kts_now); gASI.draw(spr);
  
  uint16_t touchColor = (millis() - g_last_touch_time < 1000) ? C_GREEN : C_RED;
  spr.fillCircle(LCD_W - 20, 20, 8, touchColor);
  
  spr.setTextColor(C_LGREY, C_BLACK); spr.setTextSize(1);
  spr.setCursor(10, LCD_H - 20); spr.print("Swipe RIGHT for Settings");
  
  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
}

// ===================================================================
// =================== App lifecycle =================================
// ===================================================================

//struct SwipeState { bool active=false; int16_t x0=0,y0=0; int16_t xLast=0,yLast=0; uint32_t t0=0; } gSwipe;

void setup() {
  Serial.begin(115200);
  delay(300);
  
  // DISABLE TOUCH LIBRARY DEBUG MESSAGES
  esp_log_level_set("*", ESP_LOG_WARN);
  
  Serial.println("\n[S3] Boot with Polar TE Support");

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

  // Link - Initialize g_link with UART
  g_link.begin(LINK_BAUD, S3_RX, S3_TX);
  Serial.printf("[S3] Link listening on UART (RX=%d, TX=%d) @%u\n", S3_RX, S3_TX, LINK_BAUD);
  Serial.printf("[S3] Polar: %s, TE: %s\n", polarSettings.polarName, polarSettings.teCompEnabled ? "ON" : "OFF");

  Serial.println("[S3] Ready.");
}

void loop() {
  static uint32_t last_debug = 0;

  // Poll link and decode frames
  link_poll();

  // Pull current signals
  static float v_ui = 0.0f;
  static float asi_kts_now = 0.0f;
  static float alt_ft = 0.0f;

  static uint32_t last_update_ms = 0;
  uint32_t now = millis();
  float dt = (now - last_update_ms) * 0.001f; if (dt <= 0) dt = 1e-3f; last_update_ms = now;

  // If we have fresh telemetry, use it
  if (gTlmFresh) {
    gTlmFresh = false;

    float v_raw = polarSettings.teCompEnabled ? gTlm.te_vario_mps : gTlm.vario_mps;
    
    // deadband for audio/needle stability
    const float V_DEADBAND = 0.15f;
    float v_disp = (fabsf(v_raw) < V_DEADBAND) ? 0.0f : v_raw;

    // UI-only smoothing
    v_ui = 0.85f * v_ui + 0.15f * v_disp;

    // Altitude (m -> ft)
    alt_ft = gTlm.alt_m * 3.28084f;

    // ASI
    const bool hasFix = (gTlm.fix >= 1);
    asi_kts_now = hasFix ? (gTlm.asi_kts < 0.0f ? 0.0f : gTlm.asi_kts) : 0.0f;

    // Audio uses the de-deadbanded vario
    audioManager.setVario(v_disp);
  } else {
    // No fresh frame
    v_ui *= 0.999f;
  }

  // Touch / navigation
  if (g_touch_initialized) {
    int16_t x, y; 
    bool pressed = touch_read_once(x, y); 
    uint32_t t0 = now;
    
    if (pressed && !gSwipe.active) {
        // Touch started
        gSwipe.active = true; 
        gSwipe.x0 = x; 
        gSwipe.y0 = y; 
        gSwipe.xLast = x; 
        gSwipe.yLast = y; 
        gSwipe.t0 = t0;
    } 
    else if (pressed && gSwipe.active) {
        // Just update the last position for swipe detection
        gSwipe.xLast = x; 
        gSwipe.yLast = y;
        
        // If slider is active, update it in real-time
        if (g_in_settings && settingsPage == 0 && g_slider.active) {
            if (y >= g_slider.y_pos && y <= g_slider.y_pos + g_slider.height) {
                updateSliderFromTouch(x);
            }
        }
        
        // Handle polar settings touches in real-time
        if (g_in_settings && settingsPage == 3) {
            handlePolarTouch(x, y);
        }
    }
    else if (!pressed && gSwipe.active) {
        // Touch ended - detect simple gestures
        int dx = gSwipe.xLast - gSwipe.x0;
        int dy = gSwipe.yLast - gSwipe.y0; 
        uint32_t dtg = now - gSwipe.t0;
        
        // Much simpler and more reliable gesture detection
        bool isSwipe = (abs(dx) > 50) && (abs(dy) < 40) && (dtg < 1000);
        bool isLeftSwipe = isSwipe && (dx < 0);
        bool isRightSwipe = isSwipe && (dx > 0);
        bool isTap = (abs(dx) < 20 && abs(dy) < 20 && dtg < 400);
        
        Serial.printf("[TOUCH] End: dx=%d, Lswipe=%d, Rswipe=%d, tap=%d\n", 
                     dx, isLeftSwipe, isRightSwipe, isTap);
        
        gSwipe.active = false;
        
        if (isRightSwipe && !g_in_settings) {
            // Swipe right to enter settings (more natural)
            g_in_settings = true;
            settingsPage = 0;
            g_slider.active = false;
            Serial.println("[TOUCH] Entering settings");
        }
        else if (isLeftSwipe && g_in_settings) {
            // Swipe left to exit settings (more natural) - works from any settings screen
            applySettingsChanges();
            g_in_settings = false;
            g_slider.active = false;
            Serial.println("[TOUCH] Exiting settings");
        }
        else if (isTap) {
            // Handle taps
            handleTapGesture();
        }
    }
  }

  // Send polar settings periodically if changed
  static uint32_t lastPolarSend = 0;
  if (polarSettings.settingsChanged && (now - lastPolarSend > 1000)) {
    sendPolarSettingsToC3();
    lastPolarSend = now;
    
    // Save to preferences
    ui_prefs.begin("s3ui", false);
    ui_prefs.putBool("teEnabled", polarSettings.teCompEnabled);
    ui_prefs.putUInt("polarIdx", polarSettings.selectedPolar);
    ui_prefs.end();
  }

  // Render
  if (g_in_settings) {
    drawSettingsScreen();
  } else {
    drawMainScreen(v_ui, alt_ft, asi_kts_now);
  }

  // Debug heartbeat
  if (now - last_debug > 2000) {
    last_debug = now;
    const char* teStatus = polarSettings.teCompEnabled ? "TE" : "NETTO";
    Serial.printf("[S3] %s=%.2f alt=%.0fft asi=%.1fkt polar=%s\n",
                  teStatus, v_ui, alt_ft, asi_kts_now, polarSettings.polarName);
  }

  delay(16);
}