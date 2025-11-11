// ESP32-S3 — S3 Telemetry Consumer + Round Instrument UI + Polar TE Compensation
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <esp_err.h>
#include <Preferences.h>
#include <ctype.h>

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
// Extracted helpers
#include "ui/DrawUtils.h"
#include "utils/Filters.h"

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
// =================== Startup Sequence ===============================
// ===================================================================

enum StartupState {
    STARTUP_CONNECTING = 0,      // Wait for sensor connection
    STARTUP_LOADING_POLARS = 1,  // Load polar data with progress
    STARTUP_QNH_ENTRY = 2,       // User enters QNH
    STARTUP_POLAR_SELECTION = 3, // User selects polar
    STARTUP_COMPLETE = 4         // Ready for operation
};

struct StartupSettings {
    StartupState currentState = STARTUP_CONNECTING;
    uint32_t qnhPa = 101325;  // Default 1013.25 hPa
    int selectedPolar = 0;
    bool startupComplete = false;
};

struct PolarLoadingState {
    bool sensorConnected = false;
    bool polarListReceived = false;
    int expectedPolars = 0;
    int receivedPolars = 0;
    float progress = 0.0f;  // 0.0 to 1.0
    uint32_t connectionStartTime = 0;
    uint32_t loadingStartTime = 0;
    int connectionAttempts = 0;
    int maxConnectionAttempts = 3;
    uint32_t connectionTimeout = 10000;  // 10 seconds
    uint32_t loadingTimeout = 30000;     // 30 seconds
};

static StartupSettings startupSettings;
static PolarLoadingState g_polarLoading;
uint32_t g_lastPolarReceived = 0; // Global variable for tracking last polar received


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

// ===================================================================
// =================== Dynamic Polar Data =============================
// ===================================================================

struct PolarData {
  char name[32];
  float speeds[10];    // Speeds in m/s
  float sinks[10];     // Sink rates in m/s
  int pointCount;      // Number of data points
  bool valid;
};

struct PolarList {
  PolarData polars[100];  // Up to 100 polars
  int count;
  bool received;
};

static PolarList g_polarList;

// Fallback polar names (used until sensor sends list)
const char* fallbackPolarNames[] = { "LS8-b", "DG-800", "ASG-29", "Discus", "LS4", "ASW-27", "Nimbus-4", "Ventus-2" };
const int fallbackPolarCount = 8;

// Helper functions for polar list management
static const char* getPolarName(int index) {
  if (g_polarList.received && index < g_polarList.count) {
    return g_polarList.polars[index].name;
  } else if (index < fallbackPolarCount) {
    return fallbackPolarNames[index];
  }
  return "Unknown";
}

static int getPolarCount() {
  int count = g_polarList.received ? g_polarList.count : fallbackPolarCount;
  if (g_polarList.received) {
    Serial.printf("[POLARS] Using dynamic list: %d polars\n", count);
  } else {
    Serial.printf("[POLARS] Using fallback list: %d polars\n", count);
  }
  return count;
}

[[maybe_unused]] static bool isPolarValid(int index) {
  if (g_polarList.received && index < g_polarList.count) {
    return g_polarList.polars[index].valid;
  }
  return index < fallbackPolarCount;
}

// Parse polar list from CSV command
static void parsePolarList(const char* line) {
  // Expected format: "POLARS,name1,name2,name3,..."
  if (strncmp(line, "POLARS,", 7) != 0) {
    Serial.printf("[POLARS] Not a POLARS command: %s\n", line);
    return;
  }
  
  Serial.printf("[POLARS] Received polar list: %s\n", line);
  
  const char* data = line + 7; // Skip "POLARS,"
  
  // If this is the first POLARS command, reset the list
  static bool firstPolarList = true;
  if (firstPolarList) {
    g_polarList.count = 0;
    firstPolarList = false;
  }
  
  // Parse comma-separated polar names
  char* token;
  char* dataCopy = strdup(data);
  token = strtok(dataCopy, ",");
  
  while (token != NULL && g_polarList.count < 100) {
    // Trim whitespace
    while (*token == ' ') token++;
    char* end = token + strlen(token) - 1;
    while (end > token && *end == ' ') end--;
    *(end + 1) = '\0';
    
    // Copy name to polar data
    strncpy(g_polarList.polars[g_polarList.count].name, token, sizeof(g_polarList.polars[g_polarList.count].name) - 1);
    g_polarList.polars[g_polarList.count].name[sizeof(g_polarList.polars[g_polarList.count].name) - 1] = '\0';
    g_polarList.polars[g_polarList.count].valid = true;
    g_polarList.polars[g_polarList.count].pointCount = 0; // Will be filled by POLAR_DATA command
    
    g_polarList.count++;
    token = strtok(NULL, ",");
  }
  
  free(dataCopy);
  g_polarList.received = true;
  
  // Clean up any "POLAR" entries (duplicates or invalid entries)
  int writeIndex = 0;
  for (int i = 0; i < g_polarList.count; i++) {
    const char* name = g_polarList.polars[i].name;
    // Remove entries that are exactly "POLAR", "Polar", or empty
    if (strcmp(name, "POLAR") != 0 && 
        strcmp(name, "Polar") != 0 && 
        strcmp(name, "polar") != 0 &&
        strlen(name) > 0) {
      if (writeIndex != i) {
        // Move this polar to the write position
        g_polarList.polars[writeIndex] = g_polarList.polars[i];
      }
      writeIndex++;
    } else {
      Serial.printf("[POLARS] Removed invalid polar entry: '%s' at index %d\n", name, i);
    }
  }
  g_polarList.count = writeIndex;
  
  Serial.printf("[POLARS] Received %d polars from sensor (after cleanup)\n", g_polarList.count);
  for (int i = 0; i < g_polarList.count; i++) {
    Serial.printf("[POLARS] [%d] %s\n", i, g_polarList.polars[i].name);
  }
  
  // If we receive polar list, sensor is definitely connected
  if (!g_polarLoading.sensorConnected) {
    Serial.println("[POLARS] Received polar list - setting sensorConnected = true");
    g_polarLoading.sensorConnected = true;
  }
  
  // Update loading state
  g_polarLoading.polarListReceived = true;
  g_polarLoading.expectedPolars = g_polarList.count;
  g_polarLoading.receivedPolars = 0;
  g_polarLoading.progress = 0.1f;  // 10% for receiving the list
  
  Serial.printf("[POLARS] Updated expected polars: %d (from list count)\n", g_polarLoading.expectedPolars);
}

// Parse chunked polar data from CSV command
// Format: POLAR_DATA_CHUNK,chunkNum,totalChunks,polar1Name,point1Speed,point1Sink,point2Speed,point2Sink,...,polar2Name,...
static bool isNumericToken(const char* s) {
  if (!s || !*s) return false;
  bool hasDigit = false;
  const char* p = s;
  if (*p == '+' || *p == '-') ++p;
  for (; *p; ++p) {
    if (isdigit((unsigned char)*p)) { hasDigit = true; continue; }
    if (*p == '.') continue;
    return false;
  }
  return hasDigit;
}

static void parsePolarDataChunk(const char* line) {
  Serial.printf("[PARSE] parsePolarDataChunk called with: %s\n", line);

  // Expected format: "POLAR_DATA_CHUNK,chunkNum,totalChunks,polarName,speed1,sink1,...,polarName2,..."
  if (strncmp(line, "POLAR_DATA_CHUNK,", 17) != 0) {
    Serial.printf("[PARSE] Not a POLAR_DATA_CHUNK command: %s\n", line);
    return;
  }

  char* buf = strdup(line);
  if (!buf) { Serial.println("[PARSE] Memory allocation failed"); return; }

  // Tokenize by replacing commas with NUL in our copy
  static const int MAX_TOK = 1024;
  char* tokens[MAX_TOK];
  int ntok = 0;
  char* p = buf;
  while (p && *p && ntok < MAX_TOK) {
    tokens[ntok++] = p;
    char* comma = strchr(p, ',');
    if (!comma) break;
    *comma = '\0';
    p = comma + 1;
  }

  if (ntok < 3) { free(buf); Serial.println("[PARSE] Not enough tokens in chunk header"); return; }

  // Header: tokens[0]=POLAR_DATA_CHUNK, tokens[1]=chunkNum, tokens[2]=totalChunks
  int chunkNum = atoi(tokens[1]);
  int totalChunks = atoi(tokens[2]);
  Serial.printf("[PARSE] Processing chunk %d of %d\n", chunkNum, totalChunks);

  if (chunkNum == 1) {
    g_polarLoading.receivedPolars = 0;
    g_polarLoading.expectedPolars = 0;
    Serial.println("[PARSE] First chunk - resetting loading counters only");
  }

  int polarsInChunk = 0;

  // Walk remaining tokens, detecting polar name then numeric pairs
  int idx = 3; // start after header
  while (idx < ntok) {
    const char* nameTok = tokens[idx];
    if (!nameTok || !*nameTok) { ++idx; continue; }

    // Find polar index by exact name match
    int pi = -1;
    for (int i = 0; i < g_polarList.count; ++i) {
      if (strcmp(g_polarList.polars[i].name, nameTok) == 0) { pi = i; break; }
    }

    if (pi < 0) { // Unknown token; skip
      ++idx;
      continue;
    }

    ++idx; // move to first possible speed
    int pointCount = 0;
    while ((idx + 1) < ntok && isNumericToken(tokens[idx]) && isNumericToken(tokens[idx + 1])) {
      float speed = atof(tokens[idx]);
      float sink  = atof(tokens[idx + 1]);
      if (pointCount < 10) {
        g_polarList.polars[pi].speeds[pointCount] = speed;
        g_polarList.polars[pi].sinks[pointCount]  = sink;
      }
      ++pointCount;
      idx += 2;
    }

    g_polarList.polars[pi].pointCount = (pointCount > 10) ? 10 : pointCount;
    g_polarList.polars[pi].valid = true;
    ++polarsInChunk;
    Serial.printf("[PARSE] Chunk %d: Polar %d (%s) points=%d\n", chunkNum, pi, g_polarList.polars[pi].name, g_polarList.polars[pi].pointCount);
  }

  // Update loading state
  g_polarLoading.receivedPolars += polarsInChunk;
  g_polarLoading.expectedPolars = g_polarList.count;

  if (chunkNum == totalChunks) {
    g_polarLoading.progress = 1.0f;
    g_polarList.received = true;
    Serial.printf("[PARSE] Last chunk received! Total polars: %d\n", g_polarList.count);
  } else if (totalChunks > 0) {
    g_polarLoading.progress = (float)chunkNum / (float)totalChunks;
  }

  g_lastPolarReceived = millis();
  Serial.printf("[PARSE] Chunk %d complete: polarsInChunk=%d, total=%d, progress=%.2f\n",
                chunkNum, polarsInChunk, g_polarList.count, g_polarLoading.progress);

  free(buf);
}

// Parse polar data points from CSV command
static void parsePolarData(const char* line) {
  Serial.printf("[PARSE] parsePolarData called with: %s\n", line);
  // Expected format: "POLAR_DATA,index,speed1,sink1,speed2,sink2,..."
  if (strncmp(line, "POLAR_DATA,", 11) != 0) {
    Serial.println("[PARSE] Line doesn't start with POLAR_DATA, - ignoring");
    return;
  }
  Serial.println("[PARSE] Line format OK, proceeding with parsing");
  
  const char* data = line + 11; // Skip "POLAR_DATA,"
  char* dataCopy = strdup(data);
  
  // Parse index
  char* token = strtok(dataCopy, ",");
  if (token == NULL) {
    Serial.println("[PARSE] No token found for index");
    free(dataCopy);
    return;
  }
  
  int index = atoi(token);
  Serial.printf("[PARSE] Parsed index: %d, polarList.count: %d\n", index, g_polarList.count);
  if (index < 0) {
    Serial.printf("[PARSE] Index %d is negative, ignoring\n", index);
    free(dataCopy);
    return;
  }
  
  // Allow polar data for indices beyond the initial list count
  if (index >= g_polarList.count) {
    Serial.printf("[PARSE] Index %d beyond initial list (%d), but allowing it\n", index, g_polarList.count);
  }
  
  // Parse speed/sink pairs
  // For indices beyond the initial list, we'll just track the count but not store the data
  if (index < g_polarList.count) {
    PolarData* polar = &g_polarList.polars[index];
    polar->pointCount = 0;
    
    while (polar->pointCount < 10) {
      token = strtok(NULL, ",");
      if (token == NULL) break;
      polar->speeds[polar->pointCount] = atof(token);
      
      token = strtok(NULL, ",");
      if (token == NULL) break;
      polar->sinks[polar->pointCount] = atof(token);
      
      polar->pointCount++;
    }
    
    Serial.printf("[POLAR_DATA] Polar %d (%s): %d points\n", 
                  index, polar->name, polar->pointCount);
  } else {
    // For indices beyond the list, just parse and discard the data
    Serial.printf("[POLAR_DATA] Polar %d: data received but not stored (beyond list)\n", index);
    
    int pointCount = 0;
    while (pointCount < 10) {
      token = strtok(NULL, ",");
      if (token == NULL) break;
      float speed = atof(token);
      (void)speed;  // Mark as used to avoid warning
      
      token = strtok(NULL, ",");
      if (token == NULL) break;
      float sink = atof(token);
      (void)sink;  // Mark as used to avoid warning
      
      pointCount++;
    }
    Serial.printf("[POLAR_DATA] Polar %d: %d points (discarded)\n", index, pointCount);
  }
  
  free(dataCopy);
  
  // If we receive polar data, sensor is definitely connected
  if (!g_polarLoading.sensorConnected) {
    Serial.println("[POLARS] Received polar data - setting sensorConnected = true");
    g_polarLoading.sensorConnected = true;
  } else {
    Serial.println("[POLARS] Received polar data - sensorConnected already true");
  }
  
  // Update expected count if we receive data for a higher index
  if (index >= g_polarLoading.expectedPolars) {
    g_polarLoading.expectedPolars = index + 1;  // +1 because index is 0-based
    Serial.printf("[POLARS] Updated expected polars to %d (based on received index %d)\n", 
                  g_polarLoading.expectedPolars, index);
  }
  
  // If we haven't received a POLARS command yet, but we're getting POLAR_DATA,
  // assume the sensor is sending data for all polars it has
  if (!g_polarLoading.polarListReceived && g_polarLoading.expectedPolars > g_polarList.count) {
    Serial.printf("[POLARS] No POLARS command received, but got data for index %d. Assuming %d total polars.\n", 
                  index, g_polarLoading.expectedPolars);
    g_polarList.count = g_polarLoading.expectedPolars;
    g_polarLoading.polarListReceived = true;
  }
  
  // Update the actual polar list count to match what we're receiving
  if (g_polarLoading.expectedPolars > g_polarList.count) {
    g_polarList.count = g_polarLoading.expectedPolars;
    Serial.printf("[POLARS] Updated polar list count to %d (based on expected polars)\n", g_polarList.count);
  }
  
  // Update loading progress
  g_polarLoading.receivedPolars++;
  if (g_polarLoading.expectedPolars > 0) {
    g_polarLoading.progress = 0.1f + (0.9f * g_polarLoading.receivedPolars / g_polarLoading.expectedPolars);
  }
  
  // Update timestamp for last polar received (used for timeout detection)
  g_lastPolarReceived = millis();
  
  Serial.printf("[POLARS] Progress: received=%d, expected=%d, progress=%.2f\n", 
                g_polarLoading.receivedPolars, g_polarLoading.expectedPolars, g_polarLoading.progress);
  
  // Check if all polars are loaded
  if (g_polarLoading.receivedPolars >= g_polarLoading.expectedPolars) {
    Serial.println("[POLARS] All polar data loaded successfully!");
    g_polarLoading.progress = 1.0f;
  }
}

// Calculate optimal speed from polar data
[[maybe_unused]] static float calculateOptimalSpeed(int polarIndex, float vario) {
  if (!g_polarList.received || polarIndex >= g_polarList.count) {
    return 0.0f; // No polar data available
  }
  
  PolarData* polar = &g_polarList.polars[polarIndex];
  if (polar->pointCount < 2) {
    return 0.0f; // Insufficient polar data
  }
  
  // Find the speed that gives the best glide ratio for the current vario
  float bestSpeed = polar->speeds[0];
  float bestGlideRatio = 0.0f;
  
  for (int i = 0; i < polar->pointCount; i++) {
    float speed = polar->speeds[i];
    float sink = polar->sinks[i];
    
    // Calculate glide ratio (speed / sink rate)
    if (sink > 0.0f) {
      float glideRatio = speed / sink;
      
      // For positive vario, we want the best glide ratio
      // For negative vario, we want to minimize sink rate
      if (vario >= 0.0f && glideRatio > bestGlideRatio) {
        bestGlideRatio = glideRatio;
        bestSpeed = speed;
      } else if (vario < 0.0f && sink < bestGlideRatio) {
        bestGlideRatio = sink;
        bestSpeed = speed;
      }
    }
  }
  
  return bestSpeed;
}

// ===================================================================
// =================== Telemetry Processing ==========================
// ===================================================================

// We'll parse into CsvTlm (from CsvSerial.h)
static bool     filtersInit=false;
static uint32_t firstFrameMs=0;

// Vario smoothing
// Use shared Filters.h for Ring median buffer
#if 0
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
#endif
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

// drawTextCentered now provided by ui/DrawUtils.h

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
    if (t < 1) t = 1;
    if (t > w/2) t = w/2;
    if (t > h/2) t = h/2;
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
    if (frac < 0) frac = 0;
    if (frac > 1) frac = 1;
    const int roll = (int)lroundf(frac * chH);
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
// =================== Startup Screens ================================
// ===================================================================

// QNH Entry Screen
struct QNHEntryState {
  bool active = false;
  uint32_t qnhPa = 101325;  // Default 1013.25 hPa
  int cursorPos = 0;  // 0-3 for digits (1013)
  bool editing = false;
  char displayValue[5] = "1013";  // Display string (4 digits)
  uint32_t lastTouchTime = 0;  // For debouncing
} g_qnhEntry;

static void handleStartupSequence() {
  uint32_t now = millis();
  uint32_t elapsed = 0;  // Declare outside switch to avoid jump errors
  
  static uint32_t lastDebugTime = 0;
  if (now - lastDebugTime > 3000) {  // Debug every 3 seconds to reduce overhead
    lastDebugTime = now;
    Serial.printf("[STARTUP] State: %d, sensorConnected: %d, progress: %.2f\n", 
                  startupSettings.currentState, g_polarLoading.sensorConnected, g_polarLoading.progress);
  }
  
  switch (startupSettings.currentState) {
    case STARTUP_CONNECTING:
      // Check if sensor is connected
      if (g_polarLoading.sensorConnected) {
        Serial.println("[STARTUP] Sensor connected, requesting polar data...");
        startupSettings.currentState = STARTUP_LOADING_POLARS;
        g_polarLoading.loadingStartTime = now;
        Serial.println("[STARTUP] Sending GET_POLARS command...");
        csv.sendGetPolars();
        return;
      }
      
      // Check for timeout
      if (now - g_polarLoading.connectionStartTime > g_polarLoading.connectionTimeout) {
        g_polarLoading.connectionAttempts++;
        if (g_polarLoading.connectionAttempts >= g_polarLoading.maxConnectionAttempts) {
          Serial.println("[STARTUP] Connection failed, using fallback polars");
          startupSettings.currentState = STARTUP_QNH_ENTRY;
          g_qnhEntry.active = true;
        } else {
          Serial.printf("[STARTUP] Connection timeout, retry %d/%d\n", 
                        g_polarLoading.connectionAttempts, g_polarLoading.maxConnectionAttempts);
          g_polarLoading.connectionStartTime = now;
        }
      }
      break;
      
    case STARTUP_LOADING_POLARS:
      // Check if all polar data is loaded
      if (g_polarLoading.progress >= 1.0f) {
        Serial.println("[STARTUP] Polar data loaded, proceeding to QNH entry");
        startupSettings.currentState = STARTUP_QNH_ENTRY;
        g_qnhEntry.active = true;
        return;
      }
      
      // Only move to next screen when ALL polar data is loaded
      if (g_polarLoading.progress >= 1.0f) {
        Serial.printf("[STARTUP] All polar data loaded: received=%d, expected=%d, elapsed=%dms\n", 
                      g_polarLoading.receivedPolars, g_polarLoading.expectedPolars, elapsed);
        startupSettings.currentState = STARTUP_QNH_ENTRY;
        g_qnhEntry.active = true;
        return;
      }
      
      // If we haven't received any new polar data for 5 seconds, assume sensor is done
      if (g_polarLoading.receivedPolars > 0 && g_lastPolarReceived > 0 && now - g_lastPolarReceived > 5000) {
        Serial.printf("[STARTUP] No new polar data for 5 seconds, assuming complete: received=%d, expected=%d\n", 
                      g_polarLoading.receivedPolars, g_polarLoading.expectedPolars);
        
        // If we received a reasonable number of polars (even if not all 83), proceed
        if (g_polarLoading.receivedPolars >= 20) {
          Serial.printf("[STARTUP] Received %d polars, proceeding to QNH entry\n", g_polarLoading.receivedPolars);
          startupSettings.currentState = STARTUP_QNH_ENTRY;
          g_qnhEntry.active = true;
          return;
        } else {
          Serial.println("[STARTUP] Not enough polars received, using fallback list");
          g_polarList.count = 8; // Use fallback count
          g_polarLoading.expectedPolars = 8;
          startupSettings.currentState = STARTUP_QNH_ENTRY;
          g_qnhEntry.active = true;
          return;
        }
      }
      
      // Check for absolute timeout (5 minutes max) to prevent infinite waiting
      elapsed = now - g_polarLoading.loadingStartTime;
      if (elapsed > 300000) { // 5 minutes absolute maximum
        Serial.printf("[STARTUP] Absolute timeout reached: received=%d, expected=%d, elapsed=%dms\n", 
                      g_polarLoading.receivedPolars, g_polarLoading.expectedPolars, elapsed);
        
        // If we didn't receive enough polars, use fallback
        if (g_polarLoading.receivedPolars < 5) {
          Serial.println("[STARTUP] Not enough polars received, using fallback list");
          g_polarList.count = 8; // Use fallback count
          g_polarLoading.expectedPolars = 8;
        }
        
        startupSettings.currentState = STARTUP_QNH_ENTRY;
        g_qnhEntry.active = true;
      }
      break;
      
    case STARTUP_QNH_ENTRY:
    case STARTUP_POLAR_SELECTION:
      // These are handled by user interaction
      break;
      
    case STARTUP_COMPLETE:
      // Startup is complete
      break;
  }
}

// Polar Selection Screen  
struct PolarSelectionState {
  bool active = false;
  int selectedIndex = 0;
  int scrollOffset = 0;
  int visibleItems = 6;  // Show 6 items at a time
  bool dropdownOpen = false;
  uint32_t lastTouchTime = 0;  // For debouncing
} g_polarSelection;

// ===================================================================
// =================== Polar Settings UI =============================
// ===================================================================

struct SliderState {
  bool active=false; int value=0; int min_value=0; int max_value=10; int x_pos=0; int y_pos=0; int width=0; int height=30;
} g_slider;

// drawRoundedRect now provided by ui/DrawUtils.h

// drawTextCentered2 now provided by ui/DrawUtils.h

static void drawButton(TFT_eSprite& s, int x, int y, int w, int h, const char* label, uint16_t fg, uint16_t bg) {
  s.fillRect(x, y, w, h, bg);
  s.drawRect(x, y, w, h, C_WHITE);
  drawTextCentered2(s, label, x + w/2, y + h/2, 2, fg);
}

// ===================================================================
// =================== Connecting to Sensor Screen ==================
// ===================================================================

static void drawConnectingToSensorScreen() {
  spr.fillSprite(C_BLACK);
  
  // Title
  drawTextCentered2(spr, "CONNECTING TO SENSOR", LCD_W/2, 50, 3, C_YELLOW);
  
  // Status message
  if (g_polarLoading.connectionAttempts > 0) {
    char statusMsg[64];
    snprintf(statusMsg, sizeof(statusMsg), "Attempt %d of %d", 
             g_polarLoading.connectionAttempts, g_polarLoading.maxConnectionAttempts);
    drawTextCentered2(spr, statusMsg, LCD_W/2, 100, 2, C_WHITE);
  }
  
  // Main message
  drawTextCentered2(spr, "Searching for", LCD_W/2, 140, 2, C_WHITE);
  drawTextCentered2(spr, "sensor board...", LCD_W/2, 160, 2, C_WHITE);
  
  // Spinning indicator
  static uint32_t lastSpinTime = 0;
  static int spinFrame = 0;
  uint32_t now = millis();
  if (now - lastSpinTime > 200) {  // 200ms per frame
    lastSpinTime = now;
    spinFrame = (spinFrame + 1) % 8;
  }
  
  int centerX = LCD_W / 2;
  int centerY = 220;
  int radius = 20;
  
  // Draw spinning dots
  for (int i = 0; i < 8; i++) {
    float angle = (i * 45.0f) + (spinFrame * 45.0f);
    float x = centerX + radius * cos(angle * PI / 180.0f);
    float y = centerY + radius * sin(angle * PI / 180.0f);
    
    uint16_t color = (i == 0) ? C_WHITE : C_DGREY;
    spr.fillCircle(x, y, 3, color);
  }
  
  // Timeout warning
  uint32_t elapsed = millis() - g_polarLoading.connectionStartTime;
  if (elapsed > 5000) {  // Show warning after 5 seconds
    drawTextCentered2(spr, "Taking longer than expected...", LCD_W/2, 280, 2, C_ORANGE);
  }
  
  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
}

// ===================================================================
// =================== Loading Polar Data Screen ====================
// ===================================================================

static void drawLoadingPolarDataScreen() {
  static uint32_t lastDrawTime = 0;
  uint32_t now = millis();
  if (now - lastDrawTime > 2000) {  // Debug every 2 seconds to reduce overhead
    lastDrawTime = now;
    Serial.printf("[LOADING] Drawing loading screen: received=%d, expected=%d, progress=%.2f\n", 
                  g_polarLoading.receivedPolars, g_polarLoading.expectedPolars, g_polarLoading.progress);
  }
  
  spr.fillSprite(C_BLACK);
  
  // Title
  drawTextCentered2(spr, "LOADING POLAR DATA", LCD_W/2, 50, 3, C_YELLOW);
  
  // Status message
  if (g_polarLoading.polarListReceived) {
    char statusMsg[64];
    snprintf(statusMsg, sizeof(statusMsg), "Found: %d polars", g_polarLoading.expectedPolars);
    drawTextCentered2(spr, statusMsg, LCD_W/2, 100, 2, C_WHITE);
  } else {
    drawTextCentered2(spr, "Receiving polar list...", LCD_W/2, 100, 2, C_WHITE);
  }
  
  // Progress bar
  int barX = LCD_W/2 - 150;
  int barY = 160;
  int barW = 300;
  int barH = 30;
  
  // Background
  spr.fillRect(barX, barY, barW, barH, C_DGREY);
  spr.drawRect(barX, barY, barW, barH, C_WHITE);
  
  // Progress fill
  int fillW = (int)(barW * g_polarLoading.progress);
  if (fillW > 0) {
    spr.fillRect(barX + 2, barY + 2, fillW - 4, barH - 4, C_GREEN);
  }
  
  // Progress percentage
  char progressText[16];
  snprintf(progressText, sizeof(progressText), "%.0f%%", g_polarLoading.progress * 100.0f);
  drawTextCentered2(spr, progressText, LCD_W/2, barY + barH + 20, 2, C_WHITE);
  
  // Detailed status
  if (g_polarLoading.polarListReceived) {
    char detailMsg[64];
    snprintf(detailMsg, sizeof(detailMsg), "Loaded: %d of %d", 
             g_polarLoading.receivedPolars, g_polarLoading.expectedPolars);
    drawTextCentered2(spr, detailMsg, LCD_W/2, 250, 2, C_CYAN);
  }
  
  // Loading animation
  static uint32_t lastAnimTime = 0;
  static int animFrame = 0;
  if (now - lastAnimTime > 300) {
    lastAnimTime = now;
    animFrame = (animFrame + 1) % 4;
  }
  
  const char* animChars[] = {"|", "/", "-", "\\"};
  drawTextCentered2(spr, animChars[animFrame], LCD_W/2, 300, 3, C_YELLOW);
  
  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
}

// ===================================================================
// =================== QNH Entry Screen ===============================
// ===================================================================

static void drawQNHEntryScreen() {
  spr.fillSprite(C_BLACK);
  
  // Title - moved up to avoid circular edge
  drawTextCentered2(spr, "QNH SETTING", LCD_W/2, 25, 3, C_YELLOW);
  drawTextCentered2(spr, "Enter barometric pressure", LCD_W/2, 50, 2, C_WHITE);
  
  // Current QNH display
  char qnhStr[16];
  snprintf(qnhStr, sizeof(qnhStr), "%s hPa", g_qnhEntry.displayValue);
  
  // QNH value box - centered and smaller
  int qnhBoxX = LCD_W/2 - 60;
  int qnhBoxY = 75;
  int qnhBoxW = 120;
  int qnhBoxH = 50;
  
  spr.fillRect(qnhBoxX, qnhBoxY, qnhBoxW, qnhBoxH, C_BLACK);
  spr.drawRect(qnhBoxX, qnhBoxY, qnhBoxW, qnhBoxH, C_WHITE);
  drawTextCentered2(spr, qnhStr, LCD_W/2, qnhBoxY + qnhBoxH/2, 3, C_CYAN);
  
  // Cursor indicator
  if (g_qnhEntry.editing) {
    int cursorX = qnhBoxX + 15 + (g_qnhEntry.cursorPos * 25);
    spr.fillRect(cursorX, qnhBoxY + 8, 2, qnhBoxH - 16, C_YELLOW);
  }
  
  // Numeric keypad - 3x3 layout with better spacing
  const char* keys[3][3] = {
    {"1", "2", "3"},
    {"4", "5", "6"}, 
    {"7", "8", "9"}
  };
  
  int keyW = 70, keyH = 50;
  int keySpacing = 15;
  int startX = (LCD_W - (3 * keyW + 2 * keySpacing)) / 2;
  int startY = 140;
  
  // Draw number buttons
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      int x = startX + col * (keyW + keySpacing);
      int y = startY + row * (keyH + keySpacing);
      
      spr.fillRect(x, y, keyW, keyH, C_DGREY);
      spr.drawRect(x, y, keyW, keyH, C_WHITE);
      drawTextCentered2(spr, keys[row][col], x + keyW/2, y + keyH/2, 3, C_WHITE);
    }
  }
  
  // Zero button - centered below number pad
  int zeroX = startX + keyW + keySpacing;
  int zeroY = startY + 3 * (keyH + keySpacing);
  spr.fillRect(zeroX, zeroY, keyW, keyH, C_DGREY);
  spr.drawRect(zeroX, zeroY, keyW, keyH, C_WHITE);
  drawTextCentered2(spr, "0", zeroX + keyW/2, zeroY + keyH/2, 3, C_WHITE);
  
  // Control buttons - on the right side
  int controlX = startX + 3 * (keyW + keySpacing) + 10;
  int controlY = startY;
  int controlW = 60;
  int controlH = 40;
  
  // DEL button
  spr.fillRect(controlX, controlY, controlW, controlH, C_RED);
  spr.drawRect(controlX, controlY, controlW, controlH, C_WHITE);
  drawTextCentered2(spr, "DEL", controlX + controlW/2, controlY + controlH/2, 2, C_WHITE);
  
  // CLR button
  spr.fillRect(controlX, controlY + controlH + 10, controlW, controlH, C_ORANGE);
  spr.drawRect(controlX, controlY + controlH + 10, controlW, controlH, C_WHITE);
  drawTextCentered2(spr, "CLR", controlX + controlW/2, controlY + controlH + 10 + controlH/2, 2, C_WHITE);
  
  // OK button - under DEL and CLR buttons
  spr.fillRect(controlX, controlY + 2 * (controlH + 10), controlW, controlH, C_GREEN);
  spr.drawRect(controlX, controlY + 2 * (controlH + 10), controlW, controlH, C_WHITE);
  drawTextCentered2(spr, "OK", controlX + controlW/2, controlY + 2 * (controlH + 10) + controlH/2, 2, C_BLACK);
  
  // Instructions
  drawTextCentered2(spr, "Range: 950-1050 hPa", LCD_W/2, LCD_H - 25, 2, C_LGREY);
  
  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
}

// ===================================================================
// =================== Polar Selection Screen ========================
// ===================================================================

static void drawPolarSelectionScreen() {
  spr.fillSprite(C_BLACK);
  
  // Title - bigger text
  drawTextCentered2(spr, "POLAR SELECTION", LCD_W/2, 20, 3, C_YELLOW);
  drawTextCentered2(spr, "Select your glider", LCD_W/2, 50, 2, C_WHITE);
  
  // 3-line polar selection box - moved much further down
  int boxX = LCD_W/2 - 180;  // Much wider box
  int boxY = 140;  // Moved up to avoid button overlap
  int boxW = 360;  // Much wider for 466px screen
  int boxH = 150;  // Much taller for better visibility
  int itemHeight = 50;  // Much taller items
  
  // Up chevron above the box (pointing UP) - always visible
  int upX = LCD_W/2;
  int upY = boxY - 50;  // Positioned relative to box
  if (g_polarSelection.scrollOffset > 0) {
    spr.fillTriangle(upX, upY, upX - 15, upY + 20, upX + 15, upY + 20, C_WHITE);
  } else {
    // Show disabled up chevron
    spr.fillTriangle(upX, upY, upX - 15, upY + 20, upX + 15, upY + 20, C_DGREY);
  }
  
  // Draw box background
  spr.fillRect(boxX, boxY, boxW, boxH, C_BLACK);
  spr.drawRect(boxX, boxY, boxW, boxH, C_WHITE);
  
  // Draw 3 polar items
  for (int i = 0; i < 3; i++) {
    int polarIndex = g_polarSelection.scrollOffset + i;
    if (polarIndex >= getPolarCount()) break;
    
    int y = boxY + i * itemHeight;
    bool isSelected = (polarIndex == g_polarSelection.selectedIndex);
    
    uint16_t bgColor = isSelected ? C_GREEN : C_WHITE;
    uint16_t fgColor = isSelected ? C_BLACK : C_BLACK;
    
    // Draw item background
    spr.fillRect(boxX + 2, y + 2, boxW - 4, itemHeight - 4, bgColor);
    
            // Draw polar name - bigger text
            drawTextCentered2(spr, getPolarName(polarIndex), LCD_W/2, y + itemHeight/2, 3, fgColor);
    
    // Draw selection indicator
    if (isSelected) {
      spr.fillRect(boxX + 5, y + 5, 4, itemHeight - 10, C_YELLOW);
    }
  }
  
  // Down chevron below the box (pointing DOWN)
  int downX = LCD_W/2;
  int downY = boxY + boxH + 30;
  if (g_polarSelection.scrollOffset + 3 < getPolarCount()) {
    // Bigger down chevron
    spr.fillTriangle(downX, downY + 25, downX - 20, downY, downX + 20, downY, C_WHITE);
  } else {
    // Show disabled down chevron
    spr.fillTriangle(downX, downY + 25, downX - 20, downY, downX + 20, downY, C_DGREY);
  }
  
  // Action buttons (moved down for better scrolling) - bigger buttons
  int buttonY = LCD_H - 80;
  int buttonW = 120;
  int buttonH = 50;
  int buttonSpacing = 20;
  
  // Back button
  int backX = (LCD_W - (2 * buttonW + buttonSpacing)) / 2;
  drawButton(spr, backX, buttonY, buttonW, buttonH, "BACK", C_WHITE, C_RED);
  
  // OK button
  int okX = backX + buttonW + buttonSpacing;
  drawButton(spr, okX, buttonY, buttonW, buttonH, "OK", C_BLACK, C_GREEN);
  
  lcd_PushColors(COL_OFF, 0, LCD_W, LCD_H, (uint16_t*)spr.getPointer());
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
  
  // Apply changes immediately
  if (g_selected_setting == 0) { // Volume
    settingsManager.settings.audio_volume = g_slider.value;
    audioManager.setVolume(g_slider.value);
    csv.sendSetVol((uint8_t)g_slider.value);
  } else if (g_selected_setting == 1) { // Brightness
    settingsManager.settings.display_brightness = (uint8_t)map((long)g_slider.value, 0L, 10L, 0L, 255L);
    lcd_brightness(settingsManager.settings.display_brightness);
    csv.sendSetBri((uint8_t)g_slider.value);
  }
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
    // Apply immediately
    csv.sendSetTE(polarSettings.teCompEnabled);
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

static void completeStartupSequence() {
  if (!startupSettings.startupComplete) return;
  
  // Send remaining settings after startup is complete
  csv.sendSetTE(polarSettings.teCompEnabled);
  csv.sendSetVol((uint8_t)settingsManager.settings.audio_volume);
  csv.sendSetBri((uint8_t)map((long)settingsManager.settings.display_brightness, 0L, 255L, 0L, 10L));
  
  Serial.println("[STARTUP] Sequence complete - sending remaining settings");
}

// ===================================================================
// =================== Startup Screen Touch Handlers =================
// ===================================================================

static void handleQNHTouch(int16_t x, int16_t y) {
  // Debounce touch input
  uint32_t now = millis();
  if (now - g_qnhEntry.lastTouchTime < 500) { // 500ms debounce for less sensitivity
    return;
  }
  g_qnhEntry.lastTouchTime = now;
  
  Serial.printf("[QNH] Touch at (%d, %d)\n", x, y);
  
  // Numeric keypad coordinates (3x3 grid)
  int keyW = 70, keyH = 50;
  int keySpacing = 15;
  int startX = (LCD_W - (3 * keyW + 2 * keySpacing)) / 2;
  int startY = 140;
  
  // Check number buttons (1-9)
  if (x >= startX && x <= startX + 3 * (keyW + keySpacing) - keySpacing && 
      y >= startY && y <= startY + 3 * (keyH + keySpacing) - keySpacing) {
    
    int col = (x - startX) / (keyW + keySpacing);
    int row = (y - startY) / (keyH + keySpacing);
    
    if (col >= 0 && col < 3 && row >= 0 && row < 3) {
      int digit = row * 3 + col + 1;
      if (row == 2 && col == 2) digit = 9; // Fix for 9
      
      // Add digit at cursor position
      if (g_qnhEntry.cursorPos < 4) {
        g_qnhEntry.displayValue[g_qnhEntry.cursorPos] = '0' + digit;
        g_qnhEntry.cursorPos++;
        Serial.printf("[QNH] Added digit %d, cursor at %d\n", digit, g_qnhEntry.cursorPos);
      }
      return;
    }
  }
  
  // Zero button - centered below number pad
  int zeroX = startX + keyW + keySpacing;
  int zeroY = startY + 3 * (keyH + keySpacing);
  if (x >= zeroX && x <= zeroX + keyW && y >= zeroY && y <= zeroY + keyH) {
    if (g_qnhEntry.cursorPos < 4) {
      g_qnhEntry.displayValue[g_qnhEntry.cursorPos] = '0';
      g_qnhEntry.cursorPos++;
      Serial.printf("[QNH] Added zero, cursor at %d\n", g_qnhEntry.cursorPos);
    }
    return;
  }
  
  // Control buttons - on the right side
  int controlX = startX + 3 * (keyW + keySpacing) + 10;
  int controlY = startY;
  int controlW = 60;
  int controlH = 40;
  
  // DEL button
  if (x >= controlX && x <= controlX + controlW && y >= controlY && y <= controlY + controlH) {
    if (g_qnhEntry.cursorPos > 0) {
      // Shift digits left from cursor position
      for (int i = g_qnhEntry.cursorPos - 1; i < 3; i++) {
        g_qnhEntry.displayValue[i] = g_qnhEntry.displayValue[i + 1];
      }
      g_qnhEntry.displayValue[3] = '0'; // Fill with zero
      g_qnhEntry.cursorPos--;
      Serial.printf("[QNH] Deleted digit, cursor at %d\n", g_qnhEntry.cursorPos);
    }
    return;
  }
  
  // CLR button
  if (x >= controlX && x <= controlX + controlW && y >= controlY + controlH + 10 && y <= controlY + 2 * controlH + 10) {
    strcpy(g_qnhEntry.displayValue, "0000");
    g_qnhEntry.cursorPos = 0;
    Serial.printf("[QNH] Cleared all digits\n");
    return;
  }
  
  // OK button - under DEL and CLR buttons
  if (x >= controlX && x <= controlX + controlW && y >= controlY + 2 * (controlH + 10) && y <= controlY + 3 * controlH + 2 * 10) {
    // Validate QNH range
    int qnhValue = atoi(g_qnhEntry.displayValue);
    if (qnhValue >= 950 && qnhValue <= 1050) {
      startupSettings.qnhPa = qnhValue * 100; // Convert to Pa
      csv.sendSetQNH(startupSettings.qnhPa);
      
      // Set default polar to ASK13 (index 0 in fallback list, or find ASK13 in loaded list)
      int ask13Index = 0; // Default to first polar (ASK13 in fallback)
      for (int i = 0; i < g_polarList.count; i++) {
        if (strcmp(g_polarList.polars[i].name, "ASK13") == 0) {
          ask13Index = i;
          break;
        }
      }
      
      startupSettings.selectedPolar = ask13Index;
      g_polarSelection.selectedIndex = ask13Index;
      polarSettings.selectedPolar = ask13Index;
      strncpy(polarSettings.polarName, getPolarName(ask13Index), sizeof(polarSettings.polarName) - 1);
      polarSettings.polarName[sizeof(polarSettings.polarName) - 1] = '\0';
      
      // Send polar settings to sensor
      csv.sendSetPolar(ask13Index);
      csv.sendSetTE(polarSettings.teCompEnabled);
      
      // Complete startup
      startupSettings.currentState = STARTUP_COMPLETE;
      startupSettings.startupComplete = true;
      g_qnhEntry.active = false;
      
      Serial.printf("[STARTUP] QNH set to: %d Pa, default polar: %s (index %d)\n", 
                   startupSettings.qnhPa, getPolarName(ask13Index), ask13Index);
    } else {
      Serial.printf("[QNH] Invalid range: %d (must be 950-1050)\n", qnhValue);
    }
    return;
  }
  
  Serial.printf("[QNH] Touch not handled at (%d, %d)\n", x, y);
}

static void handlePolarSelectionTouch(int16_t x, int16_t y) {
  // Debounce touch input
  uint32_t now = millis();
  if (now - g_polarSelection.lastTouchTime < 500) { // 500ms debounce for less sensitivity
    return;
  }
  g_polarSelection.lastTouchTime = now;
  
  Serial.printf("[POLAR] Touch at (%d, %d) - Screen: %dx%d\n", x, y, LCD_W, LCD_H);
  
  // 3-line polar selection box - much larger for 466x466 screen
  int boxX = LCD_W/2 - 180;  // Much wider box
  int boxY = 140;  // Moved up to avoid button overlap
  int boxW = 360;  // Much wider for 466px screen
  int boxH = 150;  // Much taller for better visibility
  int itemHeight = 50;  // Much taller items
  
  // Check if touch is in the 3-line box
  Serial.printf("[POLAR] Box area: x=%d-%d, y=%d-%d\n", boxX, boxX + boxW, boxY, boxY + boxH);
  if (x >= boxX && x <= boxX + boxW && y >= boxY && y <= boxY + boxH) {
    int itemIndex = (y - boxY) / itemHeight;
    int polarIndex = g_polarSelection.scrollOffset + itemIndex;
    
    if (polarIndex < getPolarCount()) {
      g_polarSelection.selectedIndex = polarIndex;
      Serial.printf("[POLAR] Selected: %s (index %d)\n", getPolarName(polarIndex), polarIndex);
    }
    return;
  }
  
  // Up chevron area (above the box) - scrolls UP to show earlier items
  int upX = LCD_W/2;
  int upY = boxY - 50;  // Positioned relative to box
  Serial.printf("[POLAR] Up chevron area: x=%d-%d, y=%d-%d\n", upX - 40, upX + 40, upY - 20, upY + 20);
  if (x >= upX - 40 && x <= upX + 40 && y >= upY - 20 && y <= upY + 20) {
    if (g_polarSelection.scrollOffset > 0) {
      g_polarSelection.scrollOffset--;
      Serial.printf("[POLAR] Scrolled up to offset %d (total polars: %d)\n", g_polarSelection.scrollOffset, getPolarCount());
    } else {
      Serial.printf("[POLAR] Already at top (offset: %d)\n", g_polarSelection.scrollOffset);
    }
    return;
  }
  
  // Down chevron area (below the box) - scrolls DOWN to show later items
  int downX = LCD_W/2;
  int downY = boxY + boxH + 30;
  Serial.printf("[POLAR] Down chevron area: x=%d-%d, y=%d-%d\n", downX - 60, downX + 60, downY - 30, downY + 30);
  if (x >= downX - 60 && x <= downX + 60 && y >= downY - 30 && y <= downY + 30) {
    if (g_polarSelection.scrollOffset + 3 < getPolarCount()) {
      g_polarSelection.scrollOffset++;
      Serial.printf("[POLAR] Scrolled down to offset %d (total polars: %d)\n", g_polarSelection.scrollOffset, getPolarCount());
    }
    return;
  }
  
  // Action buttons (moved up from bottom edge) - bigger buttons
  int buttonY = LCD_H - 80;
  int buttonW = 120;
  int buttonH = 50;
  int buttonSpacing = 20;
  int backX = (LCD_W - (2 * buttonW + buttonSpacing)) / 2;
  int okX = backX + buttonW + buttonSpacing;
  
  if (y >= buttonY && y <= buttonY + buttonH) {
    if (x >= backX && x <= backX + buttonW) { // Back button
      if (startupSettings.startupComplete) {
        // If we're in normal operation, go back to settings
        g_polarSelection.active = false;
        g_in_settings = true;
        settingsPage = 0;
        Serial.printf("[POLAR] Back to settings\n");
      } else {
        // If we're in startup, go back to QNH entry
        startupSettings.currentState = STARTUP_QNH_ENTRY;
        g_qnhEntry.active = true;
        g_polarSelection.active = false;
        Serial.printf("[POLAR] Back to QNH entry\n");
      }
    } else if (x >= okX && x <= okX + buttonW) { // OK button
      if (startupSettings.startupComplete) {
        // If we're in normal operation, just update the polar selection
        polarSettings.selectedPolar = g_polarSelection.selectedIndex;
        strncpy(polarSettings.polarName, getPolarName(g_polarSelection.selectedIndex), sizeof(polarSettings.polarName) - 1);
        polarSettings.polarName[sizeof(polarSettings.polarName) - 1] = '\0';
        polarSettings.settingsChanged = true;
        
        // Send to sensor
        csv.sendSetPolar(g_polarSelection.selectedIndex);
        
        // Go back to settings
        g_polarSelection.active = false;
        g_in_settings = true;
        settingsPage = 0;
        
        Serial.printf("[SETTINGS] Polar changed to: %s (index %d)\n", 
                      getPolarName(g_polarSelection.selectedIndex), g_polarSelection.selectedIndex);
      } else {
        // If we're in startup, complete the startup sequence
        startupSettings.selectedPolar = g_polarSelection.selectedIndex;
        csv.sendSetPolar(startupSettings.selectedPolar);
        startupSettings.currentState = STARTUP_COMPLETE;
        startupSettings.startupComplete = true;
        g_polarSelection.active = false;
        
        // Update polar settings
        polarSettings.selectedPolar = startupSettings.selectedPolar;
        strncpy(polarSettings.polarName, getPolarName(startupSettings.selectedPolar), sizeof(polarSettings.polarName) - 1);
        polarSettings.polarName[sizeof(polarSettings.polarName) - 1] = '\0';
        
        Serial.printf("[STARTUP] Polar selected: %s (index %d)\n", 
                      getPolarName(startupSettings.selectedPolar), startupSettings.selectedPolar);
      }
    }
  }
  
  Serial.printf("[POLAR] Touch not handled at (%d, %d)\n", x, y);
}

[[maybe_unused]] static void applySettingsChanges() {
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
  spr.setCursor((LCD_W - modeTextWidth) / 2, 90);
  spr.print(modeText);

  // Mode-specific indicators
  spr.setTextSize(1);
  spr.setTextColor(C_LGREY, C_BLACK);
  switch (current_flight_mode) {
    case MODE_THERMAL:
      spr.setCursor(10, 120);
      spr.print("Thermal Entry/Exit");
      break;
    case MODE_CLIMB:
      spr.setCursor(10, 120);
      spr.print("Climbing");
      break;
    case MODE_DESCENT:
      spr.setCursor(10, 120);
      spr.print("Descending");
      break;
    case MODE_CRUISE:
      spr.setCursor(10, 120);
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
  // Check startup sequence first
  if (!startupSettings.startupComplete) {
    if (startupSettings.currentState == STARTUP_CONNECTING) {
      drawConnectingToSensorScreen();
    } else if (startupSettings.currentState == STARTUP_LOADING_POLARS) {
      drawLoadingPolarDataScreen();
    } else if (startupSettings.currentState == STARTUP_QNH_ENTRY) {
      drawQNHEntryScreen();
    } else if (startupSettings.currentState == STARTUP_POLAR_SELECTION) {
      drawPolarSelectionScreen();
    }
  } else if (g_polarSelection.active) {
    // Polar selection screen (when accessed from settings)
    drawPolarSelectionScreen();
  } else if (g_in_settings) {
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
      g_selected_setting = 2; 
      // Go to polar selection screen instead of polar settings
      g_polarSelection.active = true;
      g_polarSelection.selectedIndex = polarSettings.selectedPolar; // Set current selection
      g_in_settings = false;
      Serial.printf("[TOUCH] Going to polar selection screen (current: %s)\n", getPolarName(polarSettings.selectedPolar));
    } else if (gSwipe.yLast >= 230 && gSwipe.yLast <= 270) {
      g_selected_setting = 3; polarSettings.teCompEnabled = !polarSettings.teCompEnabled; polarSettings.settingsChanged = true;
      // Apply immediately
      csv.sendSetTE(polarSettings.teCompEnabled);
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

  // Initialize startup sequence
  startupSettings.currentState = STARTUP_CONNECTING;
  startupSettings.startupComplete = false;
  g_polarLoading.connectionStartTime = millis();
  g_polarLoading.connectionAttempts = 0;
  g_polarLoading.sensorConnected = false;
  g_polarLoading.polarListReceived = false;
  g_polarLoading.progress = 0.0f;
  
  // Initialize QNH entry state (will be activated later)
  g_qnhEntry.active = false;
  g_qnhEntry.qnhPa = 101325; // Default 1013.25 hPa
  g_qnhEntry.cursorPos = 0;
  strcpy(g_qnhEntry.displayValue, "1013");
  g_polarSelection.active = false;
  g_polarSelection.selectedIndex = 0;

  // Load polar settings from preferences
  ui_prefs.begin("s3ui", true);
  polarSettings.teCompEnabled = ui_prefs.getBool("teEnabled", true);
  polarSettings.selectedPolar = ui_prefs.getUInt("polarIdx", 0);
  // Note: polar name will be set when polar list is received from sensor
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
  #if 0  // disable malformed onPong line inserted earlier
  csv.onPong = [](){ Serial.println("[DISPLAY] got PONG ✔"); };
  #endif
  // Fixed onPong callback
  csv.onPong = [](){ Serial.println("[DISPLAY] got PONG"); };
  csv.onHelloSensor = [](){ 
    Serial.println("[DISPLAY] HELLO from sensor - setting sensorConnected = true");
    g_polarLoading.sensorConnected = true;
  };
  csv.onAck = [](const char* key, long v){ 
    Serial.printf("[DISPLAY] ACK %s=%ld\n", key, v);
    
    // Handle startup sequence acknowledgments
    if (!startupSettings.startupComplete) {
      if (strcmp(key, "QNH") == 0) {
        Serial.println("[STARTUP] QNH acknowledged by sensor");
      } else if (strcmp(key, "POLAR") == 0) {
        Serial.println("[STARTUP] Polar acknowledged by sensor");
      }
    }
  };
  
  // Polar list and data handlers
  csv.onPolarList = [](const char* line) { 
    Serial.printf("[DISPLAY] Received POLARS command: %s\n", line);
    // If we receive polar list, sensor is definitely connected
    if (!g_polarLoading.sensorConnected) {
      Serial.println("[POLARS] Received polar list - setting sensorConnected = true");
      g_polarLoading.sensorConnected = true;
    }
    Serial.println("[DISPLAY] Calling parsePolarList...");
    parsePolarList(line); 
    Serial.println("[DISPLAY] parsePolarList completed");
  };
  csv.onPolarData = [](const char* line) { 
    Serial.printf("[DISPLAY] Received POLAR_DATA command: %s\n", line);
    Serial.println("[DISPLAY] Calling parsePolarData...");
    parsePolarData(line); 
    Serial.println("[DISPLAY] parsePolarData completed");
  };
  csv.onPolarDataChunk = [](const char* line) { 
    Serial.printf("[DISPLAY] Received POLAR_DATA_CHUNK command: %s\n", line);
    Serial.println("[DISPLAY] Calling parsePolarDataChunk...");
    parsePolarDataChunk(line); 
    Serial.println("[DISPLAY] parsePolarDataChunk completed");
  };

  csv.sendPing(); delay(20);
  
  // Only send settings after startup is complete
  if (startupSettings.startupComplete) {
  csv.sendSetTE(polarSettings.teCompEnabled);
  csv.sendSetPolar((uint8_t)polarSettings.selectedPolar);
  csv.sendSetVol((uint8_t)settingsManager.settings.audio_volume);
  csv.sendSetBri((uint8_t)map((long)settingsManager.settings.display_brightness, 0L, 255L, 0L, 10L));
  }

  Serial.printf("[S3] Startup: %s, Polar: %s, TE: %s, Volume: %d, Brightness(0-10): %ld\n",
                startupSettings.startupComplete ? "COMPLETE" : "IN PROGRESS",
                polarSettings.polarName, polarSettings.teCompEnabled ? "ON" : "OFF",
                settingsManager.settings.audio_volume,
                map((long)settingsManager.settings.display_brightness, 0L, 255L, 0L, 10L));

  Serial.println("[S3] Ready.");
}

void loop() {
  static uint32_t last_debug = 0;
  static uint32_t telemetryCount = 0;
  static bool needsRedraw = true;

  // Service the CSV link - call multiple times to ensure all data is processed
  csv.poll();
  csv.poll(); // Call twice to process more data
  csv.poll(); // Call three times to ensure buffer is empty
  
  // Debug: Check if we're receiving any data
  static uint32_t lastDebugTime = 0;
  if (millis() - lastDebugTime > 10000) { // Every 10 seconds to reduce overhead
    lastDebugTime = millis();
    Serial.printf("[DEBUG] Polar list status: received=%d, count=%d, using=%s\n", 
                  g_polarList.received, g_polarList.count, 
                  g_polarList.received ? "dynamic" : "fallback");
  }
  
  // Check for stuck data in UART buffer during loading
  if (!startupSettings.startupComplete && startupSettings.currentState == STARTUP_LOADING_POLARS) {
    static uint32_t lastBufferCheck = 0;
    if (millis() - lastBufferCheck > 5000) { // Check every 5 seconds during loading
      lastBufferCheck = millis();
      // Force a few more polls to clear any stuck data
      for (int i = 0; i < 10; i++) {
        csv.poll();
      }
      Serial.printf("[DEBUG] Forced 10 additional CSV polls during loading\n");
    }
  }
  
  // Handle startup sequence state transitions
  if (!startupSettings.startupComplete) {
    handleStartupSequence();
    needsRedraw = true;  // Always redraw during startup sequence
  }

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

      // Startup screens
      if (!startupSettings.startupComplete) {
        if (startupSettings.currentState == STARTUP_QNH_ENTRY) {
          handleQNHTouch(x, y);
          needsRedraw = true;
        } else if (startupSettings.currentState == STARTUP_POLAR_SELECTION) {
          handlePolarSelectionTouch(x, y);
          needsRedraw = true;
        }
        // CONNECTING and LOADING_POLARS phases don't handle touch
      }
      // Polar selection screen (when accessed from settings)
      else if (g_polarSelection.active) {
        handlePolarSelectionTouch(x, y);
        needsRedraw = true;
      }
      // Slider
      else if (g_in_settings && settingsPage == 0 && g_slider.active) {
        if (y >= g_slider.y_pos && y <= g_slider.y_pos + g_slider.height) {
          updateSliderFromTouch(x);
          needsRedraw = true;
        }
      }
      // Polar page
      else if (g_in_settings && settingsPage == 3) {
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
        // Save settings to preferences (changes already applied immediately)
        settingsManager.save();
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

  // Complete startup sequence if needed
  static bool startupCompleted = false;
  if (startupSettings.startupComplete && !startupCompleted) {
    completeStartupSequence();
    startupCompleted = true;
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
