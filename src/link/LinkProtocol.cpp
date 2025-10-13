#include "LinkProtocol.h"

void LinkProtocol::sendTelemetry(const TelemetryMsg& m) {
  sendFrame(FT_TELEMETRY, reinterpret_cast<const uint8_t*>(&m), sizeof(TelemetryMsg));
}
void LinkProtocol::sendSettingChange(uint8_t id, int32_t value) {
  SettingChangeMsg msg { id, value };
  sendFrame(FT_SETTING_CHANGE, reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}
void LinkProtocol::sendTouch(const TouchMsg& t) {
  sendFrame(FT_TOUCH, reinterpret_cast<const uint8_t*>(&t), sizeof(t));
}

bool LinkProtocol::pollTelemetry(TelemetryMsg& out) {
  uint8_t type = 0; uint8_t buf[sizeof(TelemetryMsg)]; size_t len = sizeof(buf);
  if (!readFrame(type, buf, len, sizeof(buf))) return false;
  if (type != FT_TELEMETRY || len != sizeof(TelemetryMsg)) return false;
  memcpy(&out, buf, sizeof(TelemetryMsg));
  return true;
}
bool LinkProtocol::pollTouch(TouchMsg& out) {
  uint8_t type = 0; uint8_t buf[sizeof(TouchMsg)]; size_t len = sizeof(buf);
  if (!readFrame(type, buf, len, sizeof(buf))) return false;
  if (type != FT_TOUCH || len != sizeof(TouchMsg)) return false;
  memcpy(&out, buf, sizeof(TouchMsg));
  return true;
}
bool LinkProtocol::pollSettingChange(SettingChangeMsg& out) {
  uint8_t type = 0; uint8_t buf[sizeof(SettingChangeMsg)]; size_t len = sizeof(buf);
  if (!readFrame(type, buf, len, sizeof(buf))) return false;
  if (type != FT_SETTING_CHANGE || len != sizeof(SettingChangeMsg)) return false;
  memcpy(&out, buf, sizeof(SettingChangeMsg));
  return true;
}

void LinkProtocol::sendFrame(uint8_t type, const uint8_t* data, size_t len) {
  if (!ser_) return;
  uint8_t hdr[5] = { 0xC3, 0x33, type, (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF) };

  uint16_t crc = crc16(&type, 1);
  uint8_t llo = hdr[3], lhi = hdr[4];
  crc = crc16(&llo, 1, crc);
  crc = crc16(&lhi, 1, crc);
  crc = crc16(data, len, crc);

  ser_->write(hdr, sizeof(hdr));
  ser_->write(data, len);
  uint8_t cbuf[2] = { (uint8_t)(crc & 0xFF), (uint8_t)((crc >> 8) & 0xFF) };
  ser_->write(cbuf, 2);
}

bool LinkProtocol::readFrame(uint8_t& type, uint8_t* buf, size_t& len, size_t buflen) {
  if (!ser_) return false;

  while (ser_->available() >= 2) {
    int b = ser_->peek();
    if (b == 0xC3) break;
    ser_->read();
  }
  if (ser_->available() < 5) return false;

  uint8_t hdr[5]; ser_->readBytes(hdr, 5);
  if (!(hdr[0] == 0xC3 && hdr[1] == 0x33)) return false;

  type = hdr[2];
  uint16_t need = (uint16_t)hdr[3] | ((uint16_t)hdr[4] << 8);

  if (need > buflen) {
    if (ser_->available() < (int)(need + 2)) return false;
    for (uint16_t i=0;i<need+2;i++) (void)ser_->read();
    return false;
  }
  if (ser_->available() < (int)(need + 2)) return false;

  ser_->readBytes(buf, need);
  uint8_t cbuf[2]; ser_->readBytes(cbuf, 2);
  uint16_t rxcrc = (uint16_t)cbuf[0] | ((uint16_t)cbuf[1] << 8);

  uint16_t crc = crc16(&type, 1);
  uint8_t llo = hdr[3], lhi = hdr[4];
  crc = crc16(&llo, 1, crc);
  crc = crc16(&lhi, 1, crc);
  crc = crc16(buf, need, crc);

  if (crc != rxcrc) return false;
  len = need;
  return true;
}

uint16_t LinkProtocol::crc16(const uint8_t* d, size_t n, uint16_t crc) {
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int i=0;i<8;i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}
