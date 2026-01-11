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

#include "vl53l7cx_plugin_motion_indicator.h"

static const char *TAG = "vl53l7cx_example_10_motion_indicator";

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

/***********************************/
/*  VL53L7CX ULD motion indicator  */
/***********************************/
/*
 * This example shows the VL53L7CX motion indicator capabilities.
 * To use this example, user needs to be sure that macro
 * VL53L7CX_DISABLE_MOTION_INDICATOR is NOT enabled (see file platform.h).
 */

static int run_example(void)
{

	/*********************************/
	/*   VL53L7CX ranging variables  */
	/*********************************/

	uint8_t status, loop, isAlive, isReady, i;
	VL53L7CX_Configuration Dev;					 /* Sensor configuration */
	VL53L7CX_Motion_Configuration motion_config; /* Motion configuration*/
	VL53L7CX_ResultsData Results;				 /* Results data from VL53L7CX */

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
	/*   Program motion indicator    */
	/*********************************/

	/* Create motion indicator with resolution 4x4 */
	status = vl53l7cx_motion_indicator_init(&Dev, &motion_config, VL53L7CX_RESOLUTION_4X4);
	if (status)
	{
		printf("Motion indicator init failed with status : %u\n", status);
		return status;
	}

	/* (Optional) Change the min and max distance used to detect motions. The
	 * difference between min and max must never be >1500mm, and minimum never be <400mm,
	 * otherwise the function below returns error 127 */
	status = vl53l7cx_motion_indicator_set_distance_motion(&Dev, &motion_config, 1000, 2000);
	if (status)
	{
		printf("Motion indicator set distance motion failed with status : %u\n", status);
		return status;
	}

	/* If user want to change the resolution, he also needs to update the motion indicator resolution */
	// status = vl53l7cx_set_resolution(&Dev, VL53L7CX_RESOLUTION_4X4);
	// status = vl53l7cx_motion_indicator_set_resolution(&Dev, &motion_config, VL53L7CX_RESOLUTION_4X4);

	/* Increase ranging frequency for the example */
	status = vl53l7cx_set_ranging_frequency_hz(&Dev, 2);

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
				printf("Zone : %3d, Motion power : %3lu\n",
					   i,
					   Results.motion_indicator.motion[motion_config.map_id[i]]);
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
