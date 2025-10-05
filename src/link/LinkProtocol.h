#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

// --------- Pins / UART config (override in pins_config.h if you like) ----------
#ifndef LINK_UART_NUM
#define LINK_UART_NUM 2     // use Serial2 on ESP32-S3
#endif
#ifndef LINK_BAUD
#define LINK_BAUD 921600
#endif
#ifndef LINK_RX_PIN
#define LINK_RX_PIN 5
#endif
#ifndef LINK_TX_PIN
#define LINK_TX_PIN 4
#endif

// --------- Frame types ----------
enum : uint8_t {
  FT_TELEMETRY        = 0x01,   // C3 -> S3
  FT_SETTINGS_SYNC    = 0x02,   // C3 -> S3 (optional)
  FT_PING             = 0x03,   // either way
  FT_PONG             = 0x04,   // either way
  FT_TOUCH            = 0x81,   // S3 -> C3
  FT_SETTING_CHANGE   = 0x82,   // S3 -> C3
  FT_UI_EVENT         = 0x83    // S3 -> C3 (nav, enter/exit screen, etc.)
};

// --------- Telemetry payload (little-endian) ----------
struct TelemetryMsg {
  float vario_mps;      // vertical speed
  float alt_m;          // altitude (MSL or display—your choice on C3)
  float asi_kts;        // speed in knots for ASI
  float track_deg;      // ground track [0..360)
  uint8_t fix;          // 0=none,1=2D,2=3D
  uint8_t sats;         // satellites in view
  uint16_t rsv;         // padding
};

// --------- Touch payload ----------
enum : uint16_t { TOUCH_DOWN=0, TOUCH_MOVE=1, TOUCH_UP=2 };
struct TouchMsg {
  uint16_t type;  // TOUCH_DOWN/MOVE/UP
  uint16_t x;
  uint16_t y;
};

// --------- Setting change payload ----------
enum : uint8_t { SET_VOLUME=0, SET_BRIGHTNESS=1, SET_UNITS=2 };
struct SettingChangeMsg {
  uint8_t  id;      // e.g., SET_VOLUME
  int32_t  value;   // meaning depends on id (0..10 for volume, 0..255 for brightness, 0/1 for units)
};

// Simple SLIP + CRC16 link
class LinkProtocol {
public:
  void begin();
  // Poll bytes; returns true if a Telemetry frame was parsed (out is filled)
  bool pollTelemetry(TelemetryMsg &out);

  // Send helpers
  void sendTouch(uint16_t type, uint16_t x, uint16_t y);
  void sendSettingChange(uint8_t id, int32_t value);

private:
  HardwareSerial* ser_ = nullptr;
  // SLIP constants
  static constexpr uint8_t FEND = 0xC0;
  static constexpr uint8_t FESC = 0xDB;
  static constexpr uint8_t TFEND= 0xDC;
  static constexpr uint8_t TFESC= 0xDD;

  // rx state
  static constexpr size_t RX_MAX = 256;
  uint8_t rxbuf_[RX_MAX];
  size_t  rxlen_ = 0;
  bool    in_frame_ = false;

  // tx scratch
  void sendFrame(uint8_t type, const uint8_t* payload, size_t len);

  // utilities
  static uint16_t crc16_ccitt(const uint8_t* d, size_t n, uint16_t crc=0xFFFF);
  bool tryParseFrame(uint8_t*& p, size_t& n, uint8_t& outType, const uint8_t*& outPay, size_t& outLen);
};
