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

#include "vl53l7cx_plugin_xtalk.h"

static const char *TAG = "vl53l7cx_example_08_vizualize_xtalk_data";

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
/*   VL53L7CX ULD visualize Xtalk  */
/***********************************/
/*
* This example shows the possibility of VL53L7CX to visualize Xtalk data. It
* initializes the VL53L7CX ULD, perform a Xtalk calibration, and starts
* a ranging to capture 10 frames.

* In this example, we also suppose that the number of target per zone is
* set to 1 , and all output are enabled (see file platform.h).
*/

static int run_example(void)
{

	/*********************************/
	/*   VL53L7CX ranging variables  */
	/*********************************/

	uint8_t status, loop, isAlive, isReady;
	VL53L7CX_Configuration Dev;	  /* Sensor configuration */
	VL53L7CX_ResultsData Results; /* Results data from VL53L7CX */

	/* Buffer containing Xtalk data */
	uint8_t xtalk_data[VL53L7CX_XTALK_BUFFER_SIZE];

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
	/*    Start Xtalk calibration    */
	/*********************************/

	/* Start Xtalk calibration with a 3% reflective target at 600mm for the
	 * sensor, using 4 samples.
	 */

	printf("Running Xtalk calibration...\n");

	status = vl53l7cx_calibrate_xtalk(&Dev, 3, 4, 600);
	if (status)
	{
		printf("vl53l7cx_calibrate_xtalk failed, status %u\n", status);
		return status;
	}
	else
	{
		printf("Xtalk calibration done\n");

		/* Get Xtalk calibration data, in order to use them later */
		status = vl53l7cx_get_caldata_xtalk(&Dev, xtalk_data);

		/* Set Xtalk calibration data */
		status = vl53l7cx_set_caldata_xtalk(&Dev, xtalk_data);
	}

	/* (Optional) Visualize Xtalk grid and Xtalk shape */
	uint32_t i, j;
	union Block_header *bh_ptr;
	uint32_t xtalk_signal_kcps_grid[VL53L7CX_RESOLUTION_8X8];
	uint16_t xtalk_shape_bins[144];

	/* Swap buffer */
	VL53L7CX_SwapBuffer(xtalk_data, VL53L7CX_XTALK_BUFFER_SIZE);

	/* Get data */
	for (i = 0; i < VL53L7CX_XTALK_BUFFER_SIZE; i = i + 4)
	{
		bh_ptr = (union Block_header *)&(xtalk_data[i]);
		if (bh_ptr->idx == 0xA128)
		{
			printf("Xtalk shape bins located at position %#06x\n", (int)i);
			for (j = 0; j < 144; j++)
			{
				memcpy(&(xtalk_shape_bins[j]), &(xtalk_data[i + 4 + j * 2]), 2);
				printf("xtalk_shape_bins[%d] = %d\n", (int)j, (int)xtalk_shape_bins[j]);
			}
		}
		if (bh_ptr->idx == 0x9FFC)
		{
			printf("Xtalk signal kcps located at position %#06x\n", (int)i);
			for (j = 0; j < VL53L7CX_RESOLUTION_8X8; j++)
			{
				memcpy(&(xtalk_signal_kcps_grid[j]), &(xtalk_data[i + 4 + j * 4]), 4);
				xtalk_signal_kcps_grid[j] /= 2048;
				printf("xtalk_signal_kcps_grid[%d] = %d\n", (int)j, (int)xtalk_signal_kcps_grid[j]);
			}
		}
	}

	/* Re-Swap buffer (in case of re-using data later) */
	VL53L7CX_SwapBuffer(xtalk_data, VL53L7CX_XTALK_BUFFER_SIZE);

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
					   (int)i,
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
