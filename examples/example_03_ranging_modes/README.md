# Example 03 — Ranging Modes (VL53L7CX)

Demonstrates switching between the VL53L7CX ranging modes and retrieving frames:
- continuous / autonomous (depending on the ULD version and API)
- integration time and/or ranging frequency settings
- start ranging → read frames → stop ranging

## Hardware
Default pins:
- SDA: GPIO 21
- SCL: GPIO 22

## Build and run

```bash
idf.py set-target esp32
idf.py build flash monitor
```

## Configuration

### Main task stack size
Increase:
`idf.py menuconfig` → `Component config` → `ESP System Settings` → `Main task stack size`

Suggested values:
- **7168** or higher

## Notes
- Mode names and supported combinations depend on the ULD release (e.g., 1.2.x vs 2.0.x).
- If you changed I²C pins/speed in `main.c`, ensure the same values are used consistently in all examples.
