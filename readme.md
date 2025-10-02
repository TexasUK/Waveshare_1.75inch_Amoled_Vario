# WaveShare ESP32‑S3-Touch-AMOLED-1.75 Vario

A compact, ESP32‑S3–based **aviation instrument cluster** for a 466×466 round TFT/AMOLED. It renders a smooth circular **variometer**, a rolling‑digit **altimeter**, a digital **airspeed indicator**, and drives a real‑time **audio vario** via an ES8311 codec. Touch gestures (CST92xx) switch between the main screen and settings.

> ⚠️ **Hobby/experimental only.** Not certified. Don’t use as a sole reference for flight decisions.

---

## Features

* **Primary instruments**

  * **Circular Variometer** (−10…+10 m/s) with chevron needle and color‑coded center readout
  * **Altimeter** with mechanical‑style rolling digits (feet)
  * **Airspeed Indicator** (digital readout; demo range 45–65 kt for now)
* **Touch UI**

  * Swipe **Left→Right** to open **Settings**; **Right→Left** to return
  * Sliders for **volume** and **brightness**; **units** toggle; **Calibration** placeholder
* **Audio vario**

  * ES8311 + I²S output (real‑time climb/descent tones)
* **Sensing**

  * **BMP388** baro on a **secondary I²C bus** (Wire1)
  * **Kalman filter** (z, v, bias) + **median‑of‑3** spike killer + **deadband** for rock‑steady zero
  * **BMP581** support planned (higher ODR & lower noise)
* **Performance**

  * Target ~60 FPS with TFT_eSPI sprite buffering
  * Low‑latency touch with swipe detection
  * Real‑time audio in a FreeRTOS task
* **Persistence**

  * Settings saved/loaded via ESP32 Preferences

---

## Hardware

* **MCU:** ESP32‑S3 module (tested with 16 MB flash, PSRAM enabled)
* **Display:** CO5300 round TFT/AMOLED, 466×466
* **Touch:** CST92xx capacitive controller (I²C)
* **Audio:** ES8311 DAC/codec (I²S + MCLK, PA enable)
* **Baro:**

  * **Now:** BMP388 (I²C, addr 0x76/0x77)
  * **Soon:** BMP581 (I²C, typical addr 0x47)

### Pin map (default build)

| Function                               | Pin             |
| -------------------------------------- | --------------- |
| **Primary I²C** (touch/codec ctrl) SDA | 15              |
| **Primary I²C** (touch/codec ctrl) SCL | 14              |
| **Secondary I²C** (baro) SDA           | 18              |
| **Secondary I²C** (baro) SCL           | 17              |
| I²S BCLK                               | 9               |
| I²S WS/LRCK                            | 45              |
| I²S DATA (SDOUT)                       | 8               |
| I²S MCLK                               | 42              |
| PA enable                              | 46              |
| Touch RST                              | board‑dependent |
| Touch INT                              | board‑dependent |

> If you change pins, edit `pins_config.h` and/or the defines in `main.cpp` accordingly.

---

## Project Layout

```
project/
├─ src/
│  └─ main.cpp                   # App logic, UI, filters, sensors
├─ include/
│  └─ pins_config.h              # Hardware pin definitions
├─ system/
│  └─ SettingsManager.*          # Preferences (volume, brightness, units …)
├─ driver/
│  ├─ audio/
│  │  ├─ AudioManager.*          # Audio pipeline + ES8311 control
│  │  └─ es8311.*                # Codec driver
│  └─ i2s/                       # I²S configuration
├─ display/
│  └─ CO5300.*                   # Round TFT driver + panel helpers
├─ touch/
│  └─ TouchDrvCST92xx.*          # Touch controller driver
└─ platformio.ini
```

---

## Build & Flash (PlatformIO)

1. **Install**: VS Code + PlatformIO extension
2. **Clone**: `git clone … && cd project`
3. **Select env**: `WAVESHARE_1_75INCH_AMOLED_ROUND_TFT` (preconfigured for ESP32‑S3, 16 MB, PSRAM)
4. **USB**: connect board (CDC enabled), then:

   * **Build**: `pio run`
   * **Upload**: `pio run -t upload`
   * **Monitor**: `pio device monitor -b 115200`

Key `platformio.ini` highlights:

* `board = esp32s3_flash_16MB`
* `board_build.psram = enabled`
* TFT_eSPI + Adafruit BMP3xx lib deps
* CDC (USB‑Serial) enabled

---

## How it works (quick tour)

**Boot sequence**

1. Serial init → Load **Preferences** → Init **Display** (CO5300) → Init **Touch** (CST92xx) → Init **Audio** (ES8311/I²S) → Init **Baro**

**Main loop**

* Read baro → **Kalman** (z,v,b) → median‑of‑3 → **deadband** → UI + audio
* Process touch (swipes, sliders, toggles)
* Render main or settings screen → push sprite to panel

**Filtering**

* **Kalman states:** altitude (m), vertical speed (m/s), measurement bias (m)
* Tunables: `qz`, `qv`, `qb`, `r` (see `main.cpp` `BaroKalman`)
* UI/Audio deadband default **0.15 m/s** (adjust `V_DEADBAND`)

**Units / Datum**

* Display shows feet; vario is m/s.
* Altitude is computed from **baro** using `readAltitude(QNH)`; default QNH **1013.25 hPa** → expect offset vs local field elevation.
* **Planned:** Calibration panel to set **QNH/QFE/field offset** with persistence.

---

## Wiring notes

* **Baro on Wire1** (SDA=18, SCL=17). BMP388 at 0x76/0x77; future BMP581 typically at 0x47.
* Qwiic BMP581 boards include 3.3 V pull‑ups; avoid too many parallel pull‑ups on the same bus.
* ES8311 requires **MCLK**; ensure the pin supports output on your module.

---

## Controls

* **Swipe L→R**: open **Settings**
* **Swipe R→L**: return to main
* **Settings**: volume (0–10), brightness (0–10 scaled to 0–255), units toggle (feet/metres), calibration placeholder

---

## Tuning

* **Deadband**: change `V_DEADBAND` (e.g., 0.10…0.20) to taste
* **Kalman**: start with defaults; if still jittery at rest, increase `r` or reduce `qv`; if sluggish in climbs, raise `qv`
* **BMP581 (future)**: lower `r` (better sensor), possibly reduce deadband to ~0.10 m/s

---

## Troubleshooting

* **Touch warning:** `TFT_eSPI: TOUCH_CS pin not defined` → expected (we use a custom CST92xx driver)
* **Baro not found:** check SDA/SCL 18/17, 3.3 V, address (0x76/0x77), pull‑ups
* **Stray UTF‑8** compile error: remove any pasted en‑dashes/Unicode
* **BMP280 begin() overload**: call `begin(addr)` if you constructed with `&Wire1`
* **Standby enum missing:** some BMP280 lib versions don’t have `STANDBY_MS_62_5`; omit the standby arg or use a supported one (e.g., `STANDBY_MS_125`)

Serial status lines (every ~5 s) confirm Touch/Screen/Baro state and current v/alt.

---

## Roadmap

* Smooth fonts (GFXFF/FreeFonts) with fallback to bitmap fonts
* QNH/QFE/field‑elevation calibration UI + persistence
* **BMP581** backend option + build flag, higher ODR presets
* Log flight traces (SPIFFS or SD), optional BLE stream
* Power management: adaptive frame‑rate + backlight dimming

---

## Contributing

PRs welcome—UI polish, sensor backends (BMP581), performance tweaks, docs. Please format with clang‑format (default LLVM style) and keep frames under ~16 ms where possible.

---

## License

GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007

---

## Credits

* **TFT_eSPI** by Bodmer
* **Adafruit BMP388 Library**, Adafruit Unified Sensor, BusIO
* **SparkFun BMP581** library (planned)
* **ES8311** codec drivers and examples
* **CST92xx** touch controller driver

---

## Safety

This is an enthusiast project. Always fly with certified instruments and redundancy. Use this as a learning tool and for experimental fun, not as your only source of truth.
