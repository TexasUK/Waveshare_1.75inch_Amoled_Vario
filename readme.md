# Waveshare 1.75" AMOLED Round TFT Display - ESP32-S3 Telemetry Consumer

A sophisticated flight instrument display system built for the Waveshare 1.75" AMOLED round TFT display, designed to consume telemetry data from a glider variometer sensor and present it with a professional aviation-style interface.

## 🚁 Project Overview

This project implements a **telemetry consumer** that receives flight data from a sensor unit via UART and displays it on a high-resolution round AMOLED display. The system is specifically designed for glider pilots and provides real-time variometer, altimeter, airspeed, and flight mode information with an intuitive touch interface.

### Key Features

- **Round Instrument Display**: Professional aviation-style variometer with needle and digital readouts
- **Real-time Telemetry**: Receives and processes flight data from external sensor
- **Touch Interface**: Swipe gestures for settings navigation
- **Audio Feedback**: Integrated variometer audio with volume control
- **Polar Settings**: Configurable glider polar curves and TE compensation
- **Flight Mode Detection**: Automatic detection of thermal, cruise, climb, and descent modes
- **Settings Management**: Persistent storage of user preferences

## 🛠️ Hardware Requirements

### Primary Hardware
- **ESP32-S3** microcontroller (16MB flash, PSRAM enabled)
- **Waveshare 1.75" AMOLED Round TFT Display** (466x466 pixels)
- **CST92xx Touch Controller** (capacitive touch)
- **ES8311 Audio Codec** (I2S audio output)
- **External Sensor Unit** (provides telemetry via UART)

### Pin Configuration
```
Display (QSPI):
- CS: GPIO12
- SCK: GPIO38  
- D0-D3: GPIO4-7
- RST: GPIO39

Touch (I2C):
- SDA: GPIO15
- SCL: GPIO14
- INT: GPIO11
- RST: GPIO40

Audio (I2S):
- BCLK: GPIO9
- WS: GPIO45
- DO: GPIO8
- MCLK: GPIO42
- PA: GPIO46

UART Link:
- RX: GPIO44 (from sensor TX)
- TX: GPIO43 (to sensor RX)
```

## 📋 Software Architecture

### Core Components

1. **Main Application** (`main.cpp`)
   - Telemetry processing and filtering
   - UI rendering and touch handling
   - Settings management
   - Flight mode detection

2. **Display System** (`display/CO5300.*`)
   - AMOLED display driver
   - Brightness control
   - Color management

3. **Touch Interface** (`touch/`)
   - CST92xx touch controller
   - Gesture recognition (swipe, tap)
   - Settings navigation

4. **Audio System** (`driver/audio/`)
   - ES8311 codec integration
   - Variometer audio generation
   - Volume control

5. **Communication** (`CsvSerial.h`)
   - CSV-over-UART protocol
   - Bidirectional sensor communication
   - Settings synchronization

6. **Settings Management** (`system/`)
   - Persistent preferences storage
   - User configuration
   - Polar curve management

### Telemetry Data Structure
```cpp
struct CsvTlm {
  float netto = 0;      // Netto variometer (m/s)
  float te = 0;         // Total Energy variometer (m/s)
  float alt_m = 0;      // Altitude (meters)
  float asi_kts = 0;    // Airspeed (knots)
  int   fix = 0;        // GPS fix quality
  int   sats = 0;       // GPS satellites
  int   mode = 0;       // Flight mode
  bool  fresh = false;  // Data freshness flag
};
```

## 🎯 User Interface

### Main Display
- **Central Variometer**: Round gauge with needle indicating climb/descent rate
- **Digital Altimeter**: Rolling counter display in feet
- **Airspeed Indicator**: Digital readout in knots
- **Flight Mode**: Prominent display of current flight phase
- **GPS Status**: Satellite count and fix quality
- **TE/Polar Status**: Current polar curve and compensation mode

### Settings Interface
Access via **swipe right** from main screen:

1. **Volume Control**: Slider for audio volume (0-10)
2. **Brightness Control**: Slider for display brightness (0-10)
3. **Polar Settings**: 
   - TE Compensation toggle
   - Glider polar selection (LS8-b, DG-800, ASG-29, Discus)
4. **Quick TE Toggle**: Direct toggle for Total Energy compensation

### Navigation
- **Swipe Right**: Enter settings
- **Swipe Left**: Exit settings
- **Tap**: Select settings options
- **Touch Slider**: Adjust volume/brightness

## ⚙️ Configuration

### Build Configuration (`platformio.ini`)
```ini
[env:WAVESHARE_1_75INCH_AMOLED_ROUND_TFT]
platform = espressif32
board = esp32s3_flash_16MB
framework = arduino
upload_speed = 921600
board_build.flash_size = 16MB
board_build.psram = enabled
board_build.partitions = partitions.csv
board_build.filesystem = spiffs
```

### Pin Configuration (`pins_config.h`)
- QSPI display interface
- I2C touch and audio
- UART sensor communication
- I2S audio output

### Key Build Flags
- `DWAVESHARE_AMOLED_1_75`: Hardware identification
- `DES8311_AUDIO`: Audio codec support
- `ARDUINO_USB_CDC_ON_BOOT=1`: USB serial on boot

## 🔧 Dependencies

### Core Libraries
- **TFT_eSPI**: Display graphics library
- **Arduino Framework**: ESP32-S3 support
- **ESP-IDF Components**: I2S, I2C, UART drivers

### Custom Components
- **CO5300 Display Driver**: AMOLED-specific optimizations
- **CST92xx Touch Driver**: Capacitive touch support
- **ES8311 Audio Codec**: I2S audio processing
- **CSV Serial Protocol**: Sensor communication

## 🚀 Getting Started

### 1. Hardware Setup
1. Connect the Waveshare 1.75" AMOLED display to ESP32-S3
2. Wire the touch controller (I2C)
3. Connect audio codec (I2S)
4. Establish UART link to sensor unit

### 2. Software Setup
1. Install PlatformIO
2. Clone this repository
3. Install dependencies: `pio lib install`
4. Configure pins in `pins_config.h`
5. Build and upload: `pio run -t upload`

### 3. Sensor Integration
The display expects telemetry in CSV format over UART:
```
T,netto,te,alt_m,asi_kts,fix,sats,mode
```
Where:
- `netto`: Netto variometer (m/s)
- `te`: Total Energy variometer (m/s)  
- `alt_m`: Altitude in meters
- `asi_kts`: Airspeed in knots
- `fix`: GPS fix quality (0-3)
- `sats`: GPS satellite count
- `mode`: Flight mode (0=cruise, 1=thermal, 2=climb, 3=descent)

## 📊 Performance Characteristics

### Display Performance
- **Resolution**: 466x466 pixels
- **Refresh Rate**: ~60 FPS
- **Color Depth**: 16-bit RGB565
- **Touch Response**: <50ms

### Audio Performance
- **Sample Rate**: 44.1kHz
- **Bit Depth**: 16-bit
- **Latency**: <20ms
- **Volume Range**: 0-10 (software), 0-100% (hardware)

### Telemetry Processing
- **Update Rate**: 10-20Hz
- **Filtering**: Median + EMA smoothing
- **Latency**: <100ms end-to-end

## 🎛️ Advanced Features

### Variometer Processing
- **Dual Streams**: Netto and Total Energy (TE) compensation
- **Smoothing**: 9-sample median filter + exponential moving average
- **Deadband**: 0.15 m/s for audio stability
- **Range**: ±10 m/s with gamma correction

### Flight Mode Detection
- **Thermal Mode**: Sustained climb detection
- **Cruise Mode**: Level flight
- **Climb Mode**: Active climbing
- **Descent Mode**: Sinking flight

### Polar Curve Management
- **LS8-b**: Standard training glider
- **DG-800**: High-performance glider
- **ASG-29**: Competition glider  
- **Discus**: Classic glider

## 🔍 Troubleshooting

### Common Issues

1. **Display Not Working**
   - Check QSPI connections
   - Verify pin configuration
   - Ensure proper power supply

2. **Touch Not Responding**
   - Check I2C connections (SDA/SCL)
   - Verify touch controller address
   - Check interrupt pin configuration

3. **Audio Issues**
   - Verify I2S connections
   - Check ES8311 initialization
   - Ensure proper audio codec setup

4. **No Telemetry Data**
   - Check UART connections (RX/TX)
   - Verify baud rate (115200)
   - Check sensor unit communication

### Debug Output
Enable debug logging by setting:
```cpp
#define CORE_DEBUG_LEVEL=5
```

Monitor serial output at 115200 baud for diagnostic information.

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