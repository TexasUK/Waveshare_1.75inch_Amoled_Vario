// CsvSerial.h — tiny CSV-over-UART helper (no LinkProtocol)

#pragma once
#include <Arduino.h>

struct CsvTlm {
  float netto = 0, te = 0, alt_m = 0, asi_kts = 0;
  int   fix = 0, sats = 0, mode = 0;
  bool  fresh = false;
};

class CsvSerial {
public:
  CsvSerial(HardwareSerial& s, int rxPin, int txPin, uint32_t baud)
  : ser(s), RX(rxPin), TX(txPin), BAUD(baud) {}

  void begin() {
    ser.begin(BAUD, SERIAL_8N1, RX, TX);
    // purge any junk
    while (ser.available()) (void)ser.read();
  }

      // Call often in loop()
      void poll() {
        static uint32_t lastPollTime = 0;
        static int bytesReceived = 0;
        uint32_t now = millis();
        
        while (ser.available()) {
          int b = ser.read();
          bytesReceived++;
          
          // Debug every 100 bytes or every 5 seconds
          if (bytesReceived % 100 == 0 || now - lastPollTime > 5000) {
            Serial.printf("[CSV] Received %d bytes total, available: %d\n", bytesReceived, ser.available());
            lastPollTime = now;
          }
          
          if (b == '\r' || b == '\n') {
            if (rlen) { 
              line[ rlen ] = 0; 
              Serial.printf("[CSV] Processing line: %s\n", line); // Debug
              handleLine(line); 
              rlen = 0; 
            }
          } else {
            if (rlen < sizeof(line)-1) line[rlen++] = char(b);
            else {
              Serial.println("[CSV] Line buffer overflow - resetting");
              rlen = 0; // overflow -> reset
            }
          }
        }
      }

  // Display → Sensor
  inline void sendPing() { ser.println("PING"); }
  inline void sendSetTE(bool en){ ser.printf("SET,TE,%d\n", en?1:0); }
  inline void sendSetPolar(int idx){ ser.printf("SET,POLAR,%d\n", idx); }
  inline void sendSetQNH(uint32_t pa){ ser.printf("SET,QNH,%u\n", pa); }
  inline void sendSetVol(uint8_t v){ ser.printf("SET,VOL,%u\n", (unsigned)v); }
  inline void sendSetBri(uint8_t v){ ser.printf("SET,BRI,%u\n", (unsigned)v); }
  inline void sendGetPolars(){ ser.println("GET_POLARS"); }

  // Sensor → Display
  inline void sendHelloSensor(){ ser.println("HELLO,SENSOR"); }
  inline void sendPong(){ ser.println("PONG"); }
  inline void sendTelemetry(float netto, float te, float alt_m, float asi_kts, int fix, int sats, int mode){
    ser.printf("T,%.2f,%.2f,%.2f,%.2f,%d,%d,%d\n", netto, te, alt_m, asi_kts, fix, sats, mode);
  }
  inline void sendAck(const char* key, int32_t val){ ser.printf("ACK,%s,%ld\n", key, (long)val); }
  inline void sendPolars(const char* polarList){ ser.printf("POLARS,%s\n", polarList); }
  inline void sendPolarData(int index, const char* data){ ser.printf("POLAR_DATA,%d,%s\n", index, data); }
  inline void sendPolarDataChunk(const char* data){ ser.printf("POLAR_DATA_CHUNK,%s\n", data); } // New: Sensor sends chunked polar data

  // Display side: get latest parsed telemetry
  bool getLatest(CsvTlm& out){
    if (!tlm.fresh) return false;
    out = tlm; tlm.fresh = false; return true;
  }

  // Hooks the app can provide (optional)
  std::function<void()>          onHelloSensor; // display sees hello
  std::function<void()>          onPong;        // display sees pong
  std::function<void(const char*, long)> onAck; // display sees ACK
  std::function<void(const char*)> onPolarList; // display receives polar list
  std::function<void(const char*)> onPolarData; // display receives polar data
  std::function<void(const char*)> onPolarDataChunk; // display receives chunked polar data

  // Sensor-side SET handlers (set these on the sensor project)
  std::function<void(bool)>      onSetTE;
  std::function<void(int)>       onSetPolar;
  std::function<void(uint32_t)>  onSetQNH;
  std::function<void(uint8_t)>   onSetVol;
  std::function<void(uint8_t)>   onSetBri;
  std::function<void()>          onGetPolars;   // display requests polar list

private:
  HardwareSerial& ser;
  const int RX, TX; const uint32_t BAUD;
  char line[2048]; size_t rlen = 0;  // Increased for large polar lists
  CsvTlm tlm;

      void handleLine(const char* l){
        // Debug: Show important lines only
        if (strncmp(l, "POLAR_DATA,", 11) == 0 || strncmp(l, "POLARS,", 7) == 0) {
          Serial.printf("[CSV] Received line: %s\n", l);
        }
        
        // telemetry
        if (l[0]=='T' && l[1]==','){
      CsvTlm t{};
      if (sscanf(l, "T,%f,%f,%f,%f,%d,%d,%d", &t.netto, &t.te, &t.alt_m, &t.asi_kts, &t.fix, &t.sats, &t.mode)==7){
        t.fresh = true; tlm = t;
      }
      return;
    }
    // handshake
    if (!strcmp(l,"PING")) { sendPong(); return; }
    if (!strcmp(l,"PONG")) { if (onPong) onPong(); return; }
    if (!strcmp(l,"HELLO,SENSOR")) { if (onHelloSensor) onHelloSensor(); return; }
        if (!strcmp(l,"GET_POLARS")) { 
          Serial.println("[CSV] Received GET_POLARS command");
          if (onGetPolars) onGetPolars(); 
          return; 
        }

    // ACK
    if (!strncmp(l,"ACK,",4)){
      char key[12]; long v=0;
      if (sscanf(l,"ACK,%11[^,],%ld", key, &v)==2){ if(onAck) onAck(key,v); }
      return;
    }

    // Polar list and data
    if (!strncmp(l,"POLARS,",7)) { if (onPolarList) onPolarList(l); return; }
    if (!strncmp(l,"POLAR_DATA,",11)) { if (onPolarData) onPolarData(l); return; }
    if (!strncmp(l,"POLAR_DATA_CHUNK,",17)) { if (onPolarDataChunk) onPolarDataChunk(l); return; }

    // Sensor-side SETs
    if (!strncmp(l,"SET,",4)){
      char key[12]; long v=0;
      if (sscanf(l,"SET,%11[^,],%ld", key, &v)==2){
        if (!strcmp(key,"TE") && onSetTE){ onSetTE(v!=0); sendAck("TE", v); }
        else if(!strcmp(key,"POLAR") && onSetPolar){ onSetPolar((int)v); sendAck("POLAR", v); }
        else if(!strcmp(key,"QNH") && onSetQNH){ onSetQNH((uint32_t)v); sendAck("QNH", v); }
        else if(!strcmp(key,"VOL") && onSetVol){ onSetVol((uint8_t)v); sendAck("VOL", v); }
        else if(!strcmp(key,"BRI") && onSetBri){ onSetBri((uint8_t)v); sendAck("BRI", v); }
      }
    }
  }
};
