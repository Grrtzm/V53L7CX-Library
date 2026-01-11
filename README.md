# V53L7CX-Library (ESP-IDF)

VL53L7CX Time-of-Flight (8x8 multizone) driver as an ESP-IDF component, using the ESP-IDF v5.5+ **new I2C master driver** (`driver/i2c_master.h`).

This project is a clean ESP32 port of [ST’s Ultra Lite Driver (ULD) for the VL53L7CX](https://www.st.com/content/st_com/en/products/embedded-software/imaging-software/stsw-img023.html), with a robust ESP-IDF platform layer and safer firmware upload (chunked writes).

I started with a VL53L8CX chip and the `RJRP44/VL53L8CX-Library`. Then i needed a driver for a VL53L7CX.
It started as a fork from the `Twistx77/V53L7CX-Library`, but i ended up building it from the ground up.  
You can use my library as a drop-in replacement for the `RJRP44/VL53L8CX-Library` by replacing every occurrence of `L8CX` with `L7CX` (note: case sensitive). RJPR wrote his code as `Camel_Case`, you can still use that style with this library, but you can also use ST’s standard `snake_case`.
All of ST's example are re-used with minimal adjustments to get them working with ESP-IDF.

> Not compatible with Arduino framework.

## Highlights

- ESP-IDF **v5.5.x** compatible (ESP32 classic tested)
- Uses **new I²C master API** (`i2c_new_master_bus`, `i2c_master_bus_add_device`, `i2c_master_transmit_receive`, `i2c_master_multi_buffer_transmit`)
- ULD-compatible API: `vl53l7cx_is_alive()`, `vl53l7cx_init()`, `vl53l7cx_start_ranging()`, `vl53l7cx_get_ranging_data()`, …
- RJRP44-compatible API: `VL53L8CX_RdMulti()`, `VL53L8CX_RdMulti()`, `VL53L8CX_Reset_Sensor()`, `VL53L8CX_SwapBuffer()`, etc.
- Firmware upload **chunking** for robustness on ESP32
- Component-style layout (`include/`, `src/`, `examples/`)

## Requirements

- ESP-IDF v5.5.x (or newer)
- VL53L7CX connected over I²C
- External pullups on SDA/SCL are recommended (e.g. 2.2k–4.7k to 3V3), depending on your wiring length.

## Wiring (ESP32 classic example)

Default example uses:

- SDA = GPIO 21
- SCL = GPIO 22
- LPn = GPIO 19 (see Note 1 below)
- IN  = GPIO 18 (see Note 2 below)

Note 1: Only examples 1, 9 and 11 use a pin for controlling the LPn pin. You should do that if you're building a proper application. Connect the LPn pin to your GPIO pin using a 10k resistor to prevent damage to the chip.

Note 2: Examples 9 and 11 use a hardware interrupt.

Power:

- 3V3 and GND (check your module’s AVDD/IOVDD requirements)

## I²C address note (0x52 vs 0x29)

ST ULD refers to the default address as **0x52** (8-bit form).
ESP-IDF `i2c_device_config_t.device_address` expects the **7-bit** address (**0x29**).

This component keeps `VL53L7CX_DEFAULT_I2C_ADDRESS` as **0x52** (ST style), and shifts right (`>> 1`) when configuring ESP-IDF.

## Installation

### Option A — Use as local component

Clone this repository into your project:

```
your_project/
  components/
    V53L7CX-Library/
  main/
  CMakeLists.txt
```

Include in code:

```c
#include "vl53l7cx_api.h"
```

### Option B — Use ESP-IDF Component Manager

Add to `idf_component.yml`:

```yaml
dependencies:
  grrtzm/v53l7cx:
    git: https://github.com/Grrtzm/V53L7CX-Library.git
```

## Examples

- `examples/example_01_ranging_basic`: minimal example demonstrating init, ranging, and data readout.

Build example:

```
idf.py set-target esp32
idf.py build flash monitor
```

## Common issues

### Stack overflow in main task

Increase stack size via:

```
idf.py menuconfig
Component config → ESP System Settings → Main task stack size
```

### GPIO “not usable” warnings

Usually caused by:

- Duplicate I²C bus initialization
- SDA/SCL pins used elsewhere as GPIO

Ensure `i2c_new_master_bus()` is called only once.

## Licensing / attribution

- Sensor driver logic based on [**ST VL53L7CX Ultra Lite Driver (STSW-IMG036)**](https://www.st.com/en/embedded-software/stsw-img036.html).
- This repository provides an ESP-IDF-specific platform adaptation.
