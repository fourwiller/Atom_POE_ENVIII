# ATOM POE ENV III Sensor

ESP32 Arduino firmware for M5Stack ATOM Lite with POE and ENV III environmental sensor unit.

## Hardware

- **M5Stack ATOM Lite** - ESP32-based controller
- **AtomPOE** - W5500 Ethernet with POE power
- **ENV III Unit** - SHT30 (temperature/humidity) + QMP6988 (pressure)

## Features

- Temperature, humidity, and barometric pressure readings
- Web interface for real-time monitoring
- REST API with JSON and XML formats
- Accurate CPU load monitoring via FreeRTOS idle hooks
- Configurable sensor read speeds (adaptive and fixed modes)
- Two-Stage OTA firmware updates (LittleFS-based)
- Persistent settings via ESP32 NVS

## API Endpoints

### JSON Format
- `GET /api/json/status` - Full device status
- `GET /api/json/sensors` - Sensor data only
- `GET /api/json/system` - System info only

### XML Format
- `GET /api/xml/status` - Full device status
- `GET /api/xml/sensors` - Sensor data only
- `GET /api/xml/system` - System info only

### Default (JSON)
- `GET /api/status` - Full device status

## Dependencies

Install via Arduino CLI or Library Manager:
```bash
arduino-cli lib install "M5Atom"
arduino-cli lib install "M5Unit-ENV"
arduino-cli lib install "Ethernet"
```

## Pin Configuration

| Function | Pin |
|----------|-----|
| I2C SDA  | 26  |
| I2C SCL  | 32  |
| SPI SCK  | 22  |
| SPI MISO | 23  |
| SPI MOSI | 33  |
| SPI CS   | 19  |

## Building

```bash
arduino-cli compile --fqbn m5stack:esp32:m5stack_atom Atom_POE_ENVIII.ino
arduino-cli upload -p /dev/cu.usbserial-* --fqbn m5stack:esp32:m5stack_atom Atom_POE_ENVIII.ino
```

## License

MIT
