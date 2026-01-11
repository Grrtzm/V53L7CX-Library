# Example 01 — Ranging Basic (VL53L7CX)

Minimal bring-up example for the VL53L7CX Ultra Lite Driver (ULD):
- I²C init (ESP-IDF v5.5+ new I²C master driver)
- `vl53l7cx_is_alive()`
- `vl53l7cx_init()`
- start ranging, read a few frames, stop

## Hardware
Default pins:
- SDA: GPIO 21
- SCL: GPIO 22

Default I²C device address:
- `VL53L7CX_DEFAULT_I2C_ADDRESS` is **0x52** (8-bit form, ST style)
- ESP-IDF uses 7-bit address **0x29** internally (`0x52 >> 1`)

## Build and run

```bash
idf.py set-target esp32
idf.py build flash monitor
```

## Configuration

### Main task stack size
ULD init and large buffers can overflow the default main stack.

Increase:
`idf.py menuconfig` → `Component config` → `ESP System Settings` → `Main task stack size`

Suggested values:
- **7168** or higher (safe starting point)

## Expected output (example)
You should see logs similar to:
- Sensor alive
- ULD ready (version)
- Ranging started / stopped
