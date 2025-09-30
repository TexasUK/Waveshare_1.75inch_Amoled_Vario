# Waveshare 1.75″ AMOLED Vario & Airspace Warning

A compact vario (vertical speed instrument) and airspace-proximity alert system built to run on a microcontroller driving a **Waveshare 1.75″ round AMOLED display**.

---

## 🚀 Features

- Real-time vertical speed / variometer from a barometric sensor  
- Altitude / climb/sink display  
- Airspace boundary detection and warning  
- On-device storage of airspace data, fonts, icons, etc.  
- Support for OTA / file system partitions  
- Modular code architecture for display, sensors, airspace, warnings  

---

## 🛠️ Requirements & Setup

1. Install [PlatformIO](https://platformio.org/) (as VS Code extension or via CLI)  
2. Clone this repo:
   ```bash
   git clone https://github.com/TexasUK/Waveshare_1.75inch_Amoled_Vario_Airspace_Warning.git
   cd Waveshare_1.75inch_Amoled_Vario_Airspace_Warning
   ```
3. Prepare the `data/` folder with your airspace files, fonts, and other assets (if not already present)  
4. Build targets:
   ```bash
   pio run
   pio run --target uploadfs     # upload filesystem (assets / airspace files)
   pio run --target upload       # flash firmware
   ```
   *(Exact target names depend on settings in `platformio.ini` — adjust accordingly.)*

---

## 🧠 How It Works (High Level)

1. **Sensors & Data Acquisition**  
   Reads barometric sensor(s) and optionally GPS / position data.  
2. **Processing & Logic**  
   Computes vertical speed, monitors if the device is inside or near airspace boundaries.  
3. **Display & Alerting**  
   Renders the UI (altitude, variometer bar, warnings) on the AMOLED display; triggers alerts when entering/approaching restricted airspace.  
4. **Storage**  
   Uses internal flash partition(s) for file storage (airspace definitions, icons, assets, logs).  
5. **Recovery / Backup**  
   A `factory_backup.bin` is available to restore base firmware or fallback.  

---

## 💡 Extending & Customizing

- Add additional display modes or pages (e.g., flight log, map view)  
- Integrate BLE or serial to communicate with smartphone apps (e.g., live airspace updates)  
- Enhance filtering algorithms for smoother variometer sound/response  
- Support multiple airspace formats (OpenAir, JSON, KML) and dynamic downloads  
- Power optimization: duty-cycle sensor reads and display refresh

---

## 🧩 Pin / Board Configuration

See `pins_config.h` and the board definitions under `boards/` for the exact MCU pinouts used. If you port this to a different microcontroller or display, you’ll need to adjust these settings.

---

## ⚠️ Warnings & Safety Notes

- This device is intended for **augmentation**, not replacement, of certified instruments. Use it as a backup/aid only.  
- Always validate your airspace dataset accuracy before use in real flight situations.  
- Be mindful of power consumption and RF interference if internally integrated with other avionics.

---

## 📜 License

Specify your preferred open source license (e.g., MIT, Apache 2.0, GPL).  
Add a `LICENSE` file so users know usage/modification rights.

---

## 🙋 Acknowledgments

- Based on the community of open vario / glider-instrument projects  
- Thanks to display driver / sensor library authors  
- Inspired by devices such as [insert references you drew from]
````0