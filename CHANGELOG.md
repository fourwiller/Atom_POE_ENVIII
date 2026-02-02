# ATOM POE ENV III Sensor - Changelog

## Version 1.1.0 (2026-02-02)

### Fixed
- **Pressure sensor accuracy**: Replaced manual QMP6988 calibration with official M5Unit-ENV library. Readings now match reference sensors (~871 hPa vs previous incorrect 809 hPa)

### Added
- **CPU Load Monitoring**: Accurate CPU percentage using FreeRTOS idle hooks with self-calibrating baseline
  - Shows 3 decimal precision (e.g., 2.456%)
  - Displays load state: idle/light/moderate/heavy/max
  - Uses `esp_register_freertos_idle_hook_for_cpu()` for proper measurement

- **Sensor Read Speed Configuration**: 6 profiles stored in NVS (non-volatile storage)
  - Adaptive Normal (200-2000ms based on CPU load)
  - Adaptive Fast (100-1000ms)
  - Adaptive Fastest (50-500ms)
  - Fixed 200ms, 100ms, 50ms options
  - Selectable via Settings dropdown on web interface

- **Pressure in Inches Water Column (inWC)**: Additional pressure unit
  - Formula: `pressure_inwc = pressure_hpa × 0.401463`
  - Info icon tooltip on web page explains conversion

- **Dual API Format Support**:
  - JSON: `/api/json/status`, `/api/json/sensors`, `/api/json/system`
  - XML: `/api/xml/status`, `/api/xml/sensors`, `/api/xml/system`
  - Default `/api/status` returns JSON

- **Nested API Structure**: Scalable format for future multi-sensor support
  ```json
  {
    "device": { "name", "firmware", "ip", "mac", "uptime" },
    "sensors": [{
      "id": "env3",
      "type": "SHT30+QMP6988",
      "connected": true,
      "temperature": { "c", "f" },
      "humidity": { "percent" },
      "pressure": { "hpa", "inwc" }
    }],
    "system": { "cpu_percent", "cpu_state" }
  }
  ```

- **API Documentation Page**: Updated with clickable endpoint links

### Technical Details
- Hardware: M5Stack ATOM Lite + AtomPOE (W5500) + ENV III Unit
- Sensors: SHT30 (temp/humidity) + QMP6988 (pressure) via I2C
- Library: M5Unit-ENV (https://github.com/m5stack/M5Unit-ENV)
- Features: Two-Stage OTA, Web Interface, REST API, NVS Settings
