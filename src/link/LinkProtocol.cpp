#include "LinkProtocol.h"

// CRC-16/CCITT-FALSE
uint16_t LinkProtocol::crc16_ccitt(const uint8_t* d, size_t n, uint16_t crc) {
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int i=0;i<8;++i) crc = (crc & 0x8000) ? (crc<<1) ^ 0x1021 : (crc<<1);
  }
  return crc;
}

void LinkProtocol::begin() {
  static HardwareSerial linkSer(LINK_UART_NUM);
  ser_ = &linkSer;
  ser_->begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
}

bool LinkProtocol::tryParseFrame(uint8_t*& p, size_t& n, uint8_t& outType, const uint8_t*& outPay, size_t& outLen) {
  if (n < 1 + 2) return false; // type + crc
  outType = *p++;
  n--;
  if (n < 2) return false;
  outLen = n - 2; // rest minus CRC
  uint16_t got = (uint16_t)p[outLen] | ((uint16_t)p[outLen+1] << 8);
  uint16_t calc = crc16_ccitt((const uint8_t*)&outType, 1);
  calc = crc16_ccitt(p, outLen, calc);
  if (calc != got) return false;
  outPay = p;
  return true;
}

bool LinkProtocol::pollTelemetry(TelemetryMsg &out) {
  while (ser_ && ser_->available()) {
    uint8_t b = (uint8_t)ser_->read();
    if (!in_frame_) {
      if (b == FEND) { in_frame_ = true; rxlen_ = 0; }
      continue;
    }
    if (b == FEND) {
      // end of frame -> decode
      uint8_t buf[RX_MAX]; size_t blen=0;
      for (size_t i=0;i<rxlen_;++i) {
        uint8_t c = rxbuf_[i];
        if (c == FESC) {
          if (i+1 >= rxlen_) { blen=0; break; }
          uint8_t t = rxbuf_[++i];
          if      (t == TFEND) c = FEND;
          else if (t == TFESC) c = FESC;
          else { blen=0; break; }
        }
        buf[blen++] = c;
        if (blen >= sizeof(buf)) { blen=0; break; }
      }
      in_frame_ = false; rxlen_=0;
      if (blen >= 3) {
        uint8_t type; const uint8_t* pay; size_t plen;
        uint8_t* p = buf; size_t n = blen;
        if (tryParseFrame(p, n, type, pay, plen) && type == FT_TELEMETRY && plen >= sizeof(TelemetryMsg)) {
          memcpy(&out, pay, sizeof(TelemetryMsg));
          return true;
        }
      }
      continue;
    }
    if (b == FESC) {
      if (rxlen_ < RX_MAX) rxbuf_[rxlen_++] = b; // keep escape; handled at close
    } else {
      if (rxlen_ < RX_MAX) rxbuf_[rxlen_++] = b;
    }
  }
  return false;
}

void LinkProtocol::sendFrame(uint8_t type, const uint8_t* payload, size_t len) {
  if (!ser_) return;
  uint8_t head = type;
  uint16_t crc = crc16_ccitt(&head, 1);
  crc = crc16_ccitt(payload, len, crc);

  auto push = [&](uint8_t c){
    if      (c == FEND){ ser_->write(FESC); ser_->write(TFEND); }
    else if (c == FESC){ ser_->write(FESC); ser_->write(TFESC); }
    else               { ser_->write(c); }
  };

  ser_->write(FEND);
  push(head);
  for (size_t i=0;i<len;++i) push(payload[i]);
  push((uint8_t)(crc & 0xFF));
  push((uint8_t)(crc >> 8));
  ser_->write(FEND);
}

void LinkProtocol::sendTouch(uint16_t type, uint16_t x, uint16_t y) {
  TouchMsg m{type,x,y};
  sendFrame(FT_TOUCH, (uint8_t*)&m, sizeof(m));
}

void LinkProtocol::sendSettingChange(uint8_t id, int32_t value) {
  SettingChangeMsg m{id, value};
  sendFrame(FT_SETTING_CHANGE, (uint8_t*)&m, sizeof(m));
}
