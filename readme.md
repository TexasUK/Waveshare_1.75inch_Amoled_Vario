# Waveshare 1.75" AMOLED Round TFT Display Vario - uses ESP32-S3 Sensor board for telemetry

A sophisticated flight instrument display system built for the Waveshare 1.75" AMOLED round TFT display, designed to consume telemetry data from a glider variometer sensor and present it with a professional aviation-style interface.

## 🚁 Project Overview

This project implements a **Vario** that receives flight data from a sensor unit via UART and displays it on a high-resolution round AMOLED display. The system is specifically designed for glider pilots and provides real-time variometer, altimeter, airspeed, and flight mode information with an intuitive touch interface.

![IMG_20251026_025933](https://github.com/user-attachments/assets/9de2dd73-4fdb-46ac-991c-a92b9061c463)

### Key Features

- **57mm Round Instrument Panel Display**: Professional aviation-style variometer with needle and digital readouts
- **Real-time Telemetry**: Receives and processes flight data from external sensor board
- **Touch Interface**: Swipe gestures for settings pages and baseline settings (QNH, Glider Polar)
- **Audio Feedback**: Integrated variometer audio with volume control
- **Polar Settings**: Configurable glider polar curves and TE compensation
- **Flight Mode Detection**: Automatic detection of thermal, cruise, climb, and descent modes
- **Settings Management**: Persistent storage of user preferences

## 🛠️ Vario Hardware Requirements

### Primary Hardware
- **Waveshare ESP32-S3 1.75" AMOLED Round TFT Display** (16MB flash, PSRAM enabled, 466x466 pixels)
   - Uses **CST92xx Touch Controller** (capacitive touch)
   - and **ES8311 Audio Codec** (I2S audio output)
   - **External Sensor Unit** (provides telemetry via UART)

### 🛩️ **Intelligent Flight Detection**
- **Automatic flight detection** when airspeed exceeds 20 kts for 3 seconds
- **Debounced logic** prevents false triggers from brief speed spikes
- **Multi-mode airspeed calculation** adapts to different flight phases

### 📊 **Advanced Airspeed Calculation**
- **Hybrid algorithm** combines GPS groundspeed with glider polar curves
- **Wind drift compensation** calculates airspeed from groundspeed and bearing
- **Flight mode detection** (Cruise, Thermal, Climb, Descent) optimizes calculations
- **Thermal optimization** uses GPS groundspeed when circling (wind cancels out)

### 📝 **Automatic Flight Logging**
- **IGC file generation** with standard format for flight analysis software
- **Automatic start/stop** based on flight detection
- **Comprehensive headers** including pilot, glider, and competition information
- **GPS track logging** with pressure altitude and vario data

### 🔧 **Configuration & Control**
- **BLE interface** for wireless configuration from mobile devices
- **Persistent settings** stored in NVS memory
- **Real-time telemetry** via CSV protocol to display units
- **Multiple glider polar curves** with automatic selection

## System Behavior

### **Startup Sequence**
1. **Sensor initialization** - BMP581 barometer, GPS, SD card
2. **QNH calibration** - Automatic pressure calibration for accurate altitude
3. **Configuration loading** - Restore pilot and glider settings
4. **BLE advertising** - Start "FlightCore" service for mobile configuration
5. **Boot logging** - Write system status to SD card

### **Flight Detection Logic**
- **Ground phase**: No flight detection, system monitors sensors
- **Takeoff detection**: When calculated airspeed exceeds 20 kts for 3 seconds
- **Flight phase**: IGC logging active, continuous telemetry transmission
- **Landing detection**: When airspeed drops below 20 kts for 10 seconds

### **Airspeed Calculation Modes**

#### **Cruise Mode**
- Uses glider polar curve to estimate airspeed from vario readings
- Applies wind drift compensation based on GPS bearing
- High confidence (80%) in calculations

#### **Thermal Mode**
- Detected when vario > 2.0 m/s and turning rate > 0.1 rad/s
- Uses GPS groundspeed (wind effects cancel out in circles)
- Maintains rolling average of recent airspeed values
- Medium confidence (60%) during active turning

#### **Climb Mode**
- Detected when vario > 1.0 m/s
- Uses GPS groundspeed (less wind effect in climbs)
- Medium confidence (70%)

#### **Descent Mode**
- Detected when vario < -1.0 m/s
- Uses polar curve calculation for accurate descent airspeed
- High confidence (80%)

## Performance Expectations

### **Accuracy**
- **Altitude**: ±1 meter (barometric with Kalman filtering)
- **Vario**: ±0.1 m/s (with thermal compensation)
- **Airspeed**: ±2 kts (improves with flight duration as wind data accumulates)
- **GPS**: Standard GPS accuracy (±3-5 meters)

### **Response Times**
- **Flight detection**: 3 seconds after exceeding 20 kts
- **Telemetry updates**: 25 Hz (40ms intervals)
- **IGC logging**: 1 Hz (1 second intervals)
- **BLE configuration**: Real-time updates

### **Reliability**
- **Automatic recovery** from sensor failures
- **Robust flight detection** with debounced logic
- **Persistent configuration** survives power cycles
- **Error handling** for SD card and GPS issues

## Data Outputs

### **IGC Flight Logs**
- **Standard format** compatible with flight analysis software
- **Automatic filename** with date/time stamp
- **Complete headers** with pilot, glider, and competition data
- **GPS track data** with pressure altitude and vario

### **CSV Telemetry**
- **Real-time data** to display units
- **Format**: `T,<netto>,<te>,<alt_m>,<asi_kts>,<fix>,<sats>,<mode>`
- **Flight mode** included for display unit processing
- **25 Hz update rate** for smooth display

### **BLE Configuration**
- **Bluetooth setup** from mobile devices
- **Real-time updates** of pilot, glider, and competition data
- **Polar curve selection** and thermal compensation settings
- **Audio and display preferences**

## System Requirements

### **Seperate Sensor Board Hardware**
- ESP32-S3 Mini microcontroller
- BMP581 barometric pressure sensor
- GPS module (UART2)
- SD card for flight logging
- BLE for configuration

### **Power**
- 5V operation
- Low power consumption during flight
- Automatic power management

### **Environmental**
- Operating temperature: -10°C to +60°C
- Altitude range: 0 to 15,000 meters
- Vario range: -10 to +10 kt/s

## Getting Started

1. **Power on** the system
2. **Wait for GPS fix** (1-5 minutes first time)
3. **Configure via BLE** using mobile app
4. **Set glider polar** and pilot information
5. **Ready for flight** - automatic detection and logging

## Support

For technical documentation, build instructions, wiring diagrams, and component specifications, see the [Technical Documentation](docs/technical.md) page.

---

*FlightCore - Intelligent variometer system for modern gliding*
## 📝 License

This project is open source. Please refer to the individual component licenses for specific terms.

## 🤝 Contributing

Contributions are welcome! Please ensure:
- Code follows the existing style
- New features include appropriate documentation
- Hardware compatibility is maintained
- Performance impact is considered

## 📞 Support

For technical support or questions:
- Check the troubleshooting section
- Review the hardware connections
- Verify software configuration
- Monitor serial debug output

---

**Note**: This is a telemetry consumer display system. It requires a compatible sensor unit that provides flight data via the CSV-over-UART protocol described in this documentation. This is available on this space and is named ESP32S3_Vario_SensorBoard, but you could use anything to provide the required data, so long as it follows the format requirements in this code. 
