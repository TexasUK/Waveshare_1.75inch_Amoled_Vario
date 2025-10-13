#pragma once
#include <Arduino.h>
#include <Stream.h>

/*
  Frame:
  [0xC3][0x33][TYPE][LEN_L][LEN_H][PAYLOAD...][CRC16_L][CRC16_H]
  CRC16-CCITT(0x1021), init 0xFFFF, over TYPE, LEN_L, LEN_H, PAYLOAD
*/

enum : uint8_t {
  FT_TELEMETRY        = 0x01,   // Sensor -> Display
  FT_SETTINGS_SYNC    = 0x02,   // Sensor -> Display (optional)
  FT_TOUCH            = 0x81,   // Display -> Sensor
  FT_SETTING_CHANGE   = 0x82    // Display -> Sensor
};

enum : uint8_t {
  C3_SET_QNH_PA      = 10,
  C3_SET_POLAR       = 11,
  C3_TE_TOGGLE       = 12,
  C3_SET_VOLUME      = 13,  // 0..10
  C3_SET_BRIGHTNESS  = 14   // 0..10
};

#pragma pack(push, 1)
struct TelemetryMsg {
  float   vario_mps;
  float   te_vario_mps;
  float   alt_m;
  float   asi_kts;
  float   track_deg;
  uint8_t fix;
  uint8_t sats;
  uint8_t rsv[6];
};

struct TouchMsg {
  uint16_t type;
  uint16_t x;
  uint16_t y;
};

struct SettingChangeMsg {
  uint8_t  id;
  int32_t  value;
};
#pragma pack(pop)

class LinkProtocol {
public:
  LinkProtocol() : ser_(nullptr) {}
  void attach(Stream* s) { ser_ = s; }

  // Outbound
  void sendTelemetry(const TelemetryMsg& m);
  void sendSettingChange(uint8_t id, int32_t value);
  void sendTouch(const TouchMsg& t);

  inline void sendQNH(uint32_t pa)           { sendSettingChange(C3_SET_QNH_PA, (int32_t)pa); }
  inline void sendPolarSelect(uint8_t idx)   { sendSettingChange(C3_SET_POLAR,  (int32_t)idx); }
  inline void sendTEToggle(bool en)          { sendSettingChange(C3_TE_TOGGLE,  en ? 1 : 0);  }
  inline void sendVolume(uint8_t v)          { sendSettingChange(C3_SET_VOLUME, v); }
  inline void sendBrightness(uint8_t v)      { sendSettingChange(C3_SET_BRIGHTNESS, v); }

  // Inbound
  bool pollTelemetry(TelemetryMsg& out);        // Display uses this
  bool pollTouch(TouchMsg& out);                // Sensor uses this
  bool pollSettingChange(SettingChangeMsg& out);// Sensor uses this

private:
  void sendFrame(uint8_t type, const uint8_t* data, size_t len);
  bool readFrame(uint8_t& type, uint8_t* buf, size_t& len, size_t buflen);
  static uint16_t crc16(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF);

  Stream* ser_;
};
