# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
cmake_minimum_required(VERSION 3.16)

# Build this example against the component in the repository root
set(EXTRA_COMPONENT_DIRS "../../")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(vl53l7cx_example_04_power_modes)
