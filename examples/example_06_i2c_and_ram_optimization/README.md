# I²C and RAM optimization
This example shows the possibility of VL53L7CX to reduce I2C transactions and RAM footprint. It initializes the VL53L7CX ULD, and starts a ranging to capture 10 frames.
## Get started
In order to run this example you need to increase the main stack size else you will get a stack overflow error.

Run `idf.py menuconfig`. Go to Component Config -> ESP System settings and increase the Main task stack size to at least `7168`.



## Hardware notes

Default I²C pins used by the converted examples:
- SDA: GPIO 21
- SCL: GPIO 22

Adjust in `main.c` if needed.
