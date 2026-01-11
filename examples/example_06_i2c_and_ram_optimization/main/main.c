#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2c_master.h"

#include "vl53l7cx_api.h"
#include "platform.h"

static const char *TAG = "vl53l7cx_example_06_i2c_and_ram_optimization";

/* I2C defaults (ESP32 classic) */
#define I2C_PORT I2C_NUM_0
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_HZ 1000000

static i2c_master_bus_handle_t s_bus = NULL;

static void tof_platform_init(VL53L7CX_Configuration *dev)
{
	if (s_bus == NULL)
	{
		i2c_master_bus_config_t bus_cfg = {
			.clk_source = I2C_CLK_SRC_DEFAULT,
			.i2c_port = I2C_PORT,
			.sda_io_num = I2C_SDA,
			.scl_io_num = I2C_SCL,
			.glitch_ignore_cnt = 7,
			.flags.enable_internal_pullup = true,
		};
		ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));
	}

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = (VL53L7CX_DEFAULT_I2C_ADDRESS >> 1), /* 0x52 -> 0x29 */
		.scl_speed_hz = I2C_HZ,
	};

	/* Platform struct is owned by the component; we only set the device handle. */
	ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &dev->platform.handle));
}

/*******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 *
 * This file is part of the VL53L7CX Ultra Lite Driver and
 * is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 *******************************************************************************/

/**************************************/
/*  VL53L7CX ULD I2C/RAM optimization */
/**************************************/
/*
 * This example shows the possibility of VL53L7CX to reduce I2C transactions
 * and RAM footprint. It initializes the VL53L7CX ULD, and starts
 * a ranging to capture 10 frames.
 *
 * In this example, we also suppose that the number of target per zone is
 * set to 1 , and all output are enabled (see file platform.h).
 */

static int run_example(void)
{

	/*********************************/
	/*   VL53L7CX ranging variables  */
	/*********************************/

	uint8_t status, loop, isAlive, isReady, i;
	VL53L7CX_Configuration Dev;	  /* Sensor configuration */
	VL53L7CX_ResultsData Results; /* Results data from VL53L7CX */

	/*********************************/
	/*      Customer platform        */
	/*********************************/

	/* Fill the platform structure with customer's implementation. For this
	 * example, only the I2C address is used.
	 */
	Dev.platform.address = VL53L7CX_DEFAULT_I2C_ADDRESS;
	tof_platform_init(&Dev);
	
	/* (Optional) Reset sensor toggling PINs (see platform, not in API) */
	// VL53L7CX_Reset_Sensor(&(Dev.platform));

	/* (Optional) Set a new I2C address if the wanted address is different
	 * from the default one (filled with 0x20 for this example).
	 */
	// status = vl53l7cx_set_i2c_address(&Dev, 0x20);

	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L7CX sensor connected */
	status = vl53l7cx_is_alive(&Dev, &isAlive);
	if (!isAlive || status)
	{
		printf("VL53L7CX not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53L7CX sensor */
	status = vl53l7cx_init(&Dev);
	if (status)
	{
		printf("VL53L7CX ULD Loading failed\n");
		return status;
	}

	printf("VL53L7CX ULD ready ! (Version : %s)\n",
		   VL53L7CX_API_REVISION);

	/*********************************/
	/*   Reduce RAM & I2C access	 */
	/*********************************/

	/* Results can be tuned in order to reduce I2C access and RAM footprints.
	 * The 'platform.h' file contains macros used to disable output. If user declare
	 * one of these macros, the results will not be sent through I2C, and the array will not
	 * be created into the VL53L7CX_ResultsData structure.
	 * For the minimum size, ST recommends 1 targets per zone, and only keep distance_mm,
	 * target_status, and nb_target_detected. The following macros can be defined into file 'platform.h':
	 *
	 * #define VL53L7CX_DISABLE_AMBIENT_PER_SPAD
	 * #define VL53L7CX_DISABLE_NB_SPADS_ENABLED
	 * #define VL53L7CX_DISABLE_SIGNAL_PER_SPAD
	 * #define VL53L7CX_DISABLE_RANGE_SIGMA_MM
	 * #define VL53L7CX_DISABLE_REFLECTANCE_PERCENT
	 * #define VL53L7CX_DISABLE_MOTION_INDICATOR
	 */

	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53l7cx_start_ranging(&Dev);

	loop = 0;
	while (loop < 10)
	{
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A3
		 * (GPIO 1) when a new measurement is ready */

		status = vl53l7cx_check_data_ready(&Dev, &isReady);

		if (isReady)
		{
			vl53l7cx_get_ranging_data(&Dev, &Results);

			/* As the sensor is set in 4x4 mode by default, we have a total
			 * of 16 zones to print. For this example, only the data of first zone are
			 * print */
			printf("Print data no : %3u\n", Dev.streamcount);
			for (i = 0; i < 16; i++)
			{
				printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
					   i,
					   Results.target_status[VL53L7CX_NB_TARGET_PER_ZONE * i],
					   Results.distance_mm[VL53L7CX_NB_TARGET_PER_ZONE * i]);
			}
			printf("\n");
			loop++;
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		VL53L7CX_WaitMs(&(Dev.platform), 5);
	}

	status = vl53l7cx_stop_ranging(&Dev);
	printf("End of ULD demo\n");
	return status;
}

void app_main(void)
{
	int st = run_example();
	ESP_LOGI(TAG, "Example finished (status=%d)", st);
}
