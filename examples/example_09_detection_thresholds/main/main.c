#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"

#include "vl53l7cx_api.h"
#include "platform.h"

#include "vl53l7cx_plugin_detection_thresholds.h"

static const char *TAG = "vl53l7cx_example_09_detection_thresholds";

/* I2C defaults (ESP32 classic) */
#define I2C_PORT I2C_NUM_0
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_HZ 1000000

#define LED_GPIO GPIO_NUM_2	 // ESP32 internal LED on GPIO2 (to show interrupt received)
#define INT_GPIO GPIO_NUM_18 // Interrupt (IN) pin from VL53L7CX
#define LPN_GPIO GPIO_NUM_19 // LPn pin from VL53L7CX for proper power-up sequencing. Put 10k resistor between LPN_GPIO and LPn pin to prevent chip damage.

static i2c_master_bus_handle_t s_bus = NULL;
static int counter = 0;
static int previous_counter = 0;

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

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
	gpio_set_level(LED_GPIO, 1);
	counter++;
}

static void gpio_init(void)
{
	// LPn low first (essential!)
	gpio_config_t lpn_pin_conf = {
		.pin_bit_mask = (1ULL << LPN_GPIO),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE};
	gpio_config(&lpn_pin_conf);
	gpio_set_level(LPN_GPIO, 0); // Set LPn LOW during boot and i2c comms (for proper power-up sequencing)

	gpio_config_t led_pin_conf = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (1ULL << LED_GPIO),
	};
	ESP_ERROR_CHECK(gpio_config(&led_pin_conf));

	gpio_config_t int_pin_conf = {
		.intr_type = GPIO_INTR_NEGEDGE, // Interrupt on VL53L7CX INT_GPIO falling edge
		.mode = GPIO_MODE_INPUT,
		.pin_bit_mask = (1ULL << INT_GPIO),
		.pull_up_en = GPIO_PULLUP_ENABLE, // esp32 has 40k internal pull-up, necessary for VL53L7CX interrupt line
	};
	ESP_ERROR_CHECK(gpio_config(&int_pin_conf));

	ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM));
	gpio_isr_handler_add(INT_GPIO, gpio_isr_handler, (void *)INT_GPIO);

	// Enable after delay
	vTaskDelay(pdMS_TO_TICKS(100));
	gpio_set_intr_type(INT_GPIO, GPIO_INTR_NEGEDGE);
	gpio_intr_enable(INT_GPIO);
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
/* VL53L7CX ULD interrupt checkers */
/***********************************/
/*
* This example shows the possibility of VL53L7CX to program detection thresholds. It
* initializes the VL53L7CX ULD, create 2 thresholds per zone for a 4x4 resolution,
* and starts a ranging to capture 10 frames.

* In this example, we also suppose that the number of target per zone is
* set to 1 , and all output are enabled (see file platform.h).
*/

// #define UNUSED(x) (void)(x)

/* This function needs to be filled by the customer. It allows knowing when
 * the VL53L7CX interrupt is raised on INT_GPIO. This is the only way to use detection thresholds.
 */

int WaitForL7Interrupt(VL53L7CX_Configuration *pDev)
{
	// Add your implementation here ...
	//  UNUSED(pDev);
	while (previous_counter == counter) // WAIT FOR INTERRUPT
	{
		vTaskDelay(pdMS_TO_TICKS(10)); // to reset task_wdt
	}

	gpio_set_level(LED_GPIO, 1); // Blink LED to show interrupt received
	printf("Interrupt #%d\n", counter);
	vTaskDelay(pdMS_TO_TICKS(50));
	gpio_set_level(LED_GPIO, 0);
	previous_counter = counter;

	// Clear interrupt via API (essential!)
	uint8_t is_ready;
	vl53l7cx_check_data_ready(pDev, &is_ready);

	return 1;
}

// extern int WaitForL7Interrupt(VL53L7CX_Configuration *pDev);
// extern int IntCount;

static int IntCount = 0;

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

	// 1. Keep LPn low during init
	gpio_set_level(LPN_GPIO, 0);

	// Wait for power-up
	vTaskDelay(pdMS_TO_TICKS(100));

	/* (Optional) Reset sensor toggling PINs (see platform, not in API) */
	// VL53L7CX_Reset_Sensor(&(Dev.platform));

	/* (Optional) Set a new I2C address if the wanted address is different
	 * from the default one (filled with 0x20 for this example).
	 */
	// status = vl53l7cx_set_i2c_address(&Dev, 0x20);

	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	// 2. Sensor alive check
	status = vl53l7cx_is_alive(&Dev, &isAlive);
	if (!isAlive || status)
	{
		printf("VL53L7CX not detected at requested address\n");
		return status;
	}

	// 3. (Mandatory) Init VL53L7CX sensor while LPn is low.
	status = vl53l7cx_init(&Dev);
	if (status)
	{
		printf("VL53L7CX ULD Loading failed\n");
		return status;
	}

	// 4. Set LPn high to start ranging
	gpio_set_level(LPN_GPIO, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	printf("VL53L7CX ULD ready ! (Version : %s)\n",
		   VL53L7CX_API_REVISION);

	/*********************************/
	/*  Program detection thresholds */
	/*********************************/

	/* In this example, we want 2 thresholds per zone for a 4x4 resolution */
	/* Create array of thresholds (size cannot be changed) */
	VL53L7CX_DetectionThresholds thresholds[VL53L7CX_NB_THRESHOLDS];

	/* Set all values to 0 */
	memset(&thresholds, 0, sizeof(thresholds));

	/* Add thresholds for all zones (16 zones in resolution 4x4, or 64 in 8x8) */
	for (i = 0; i < 16; i++)
	{
		/* The first wanted thresholds is GREATER_THAN mode. Please note that the
		 * first one must always be set with a mathematic_operation
		 * VL53L7CX_OPERATION_NONE.
		 * For this example, the signal thresholds is set to 150 kcps/spads
		 * (the format is automatically updated inside driver)
		 */
		thresholds[2 * i].zone_num = i;
		thresholds[2 * i].measurement = VL53L7CX_SIGNAL_PER_SPAD_KCPS;
		thresholds[2 * i].type = VL53L7CX_GREATER_THAN_MAX_CHECKER;
		thresholds[2 * i].mathematic_operation = VL53L7CX_OPERATION_NONE;
		thresholds[2 * i].param_low_thresh = 150;
		thresholds[2 * i].param_high_thresh = 150;

		/* The second wanted checker is IN_WINDOW mode. We will set a
		 * mathematical thresholds VL53L7CX_OPERATION_OR, to add the previous
		 * checker to this one.
		 * For this example, distance thresholds are set between 200mm and
		 * 400mm (the format is automatically updated inside driver).
		 */
		thresholds[2 * i + 1].zone_num = i;
		thresholds[2 * i + 1].measurement = VL53L7CX_DISTANCE_MM;
		thresholds[2 * i + 1].type = VL53L7CX_IN_WINDOW;
		thresholds[2 * i + 1].mathematic_operation = VL53L7CX_OPERATION_OR;
		thresholds[2 * i + 1].param_low_thresh = 200;
		thresholds[2 * i + 1].param_high_thresh = 400;
	}

	/* The last thresholds must be clearly indicated. As we have 32
	 * checkers (16 zones x 2), the last one is the 31 */
	thresholds[31].zone_num = VL53L7CX_LAST_THRESHOLD | thresholds[31].zone_num;

	/* Send array of thresholds to the sensor */
	vl53l7cx_set_detection_thresholds(&Dev, thresholds);

	/* Enable detection thresholds */
	vl53l7cx_set_detection_thresholds_enable(&Dev, 1);

	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53l7cx_set_ranging_frequency_hz(&Dev, 10);

	IntCount = 0;
	status = vl53l7cx_start_ranging(&Dev);
	printf("Put an object between 200mm and 400mm to catch an interrupt\n");

	loop = 0;
	while (loop < 100)
	{
		/* Function WaitForL7Interrupt() does not exists, and must be
		 * implemented by user. It allows catching the interrupt raised on
		 * pin INT_GPIO (INT), when the checkers detect the programmed
		 * conditions.
		 */

		isReady = WaitForL7Interrupt(&Dev);
		if (isReady)
		{
			vl53l7cx_get_ranging_data(&Dev, &Results);

			/* As the sensor is set in 4x4 mode by default, we have a total
			 * of 16 zones to print. For this example, only the data of
			 * first zone are print */
			printf("Print data no : %3u\n", Dev.streamcount);
			for (i = 0; i < 16; i++)
			{
				printf("Zone : %3d, Status : %3u, Distance : %4d mm, Signal : %5lu kcps/SPADs\n",
					   i,
					   Results.target_status[VL53L7CX_NB_TARGET_PER_ZONE * i],
					   Results.distance_mm[VL53L7CX_NB_TARGET_PER_ZONE * i],
					   Results.signal_per_spad[VL53L7CX_NB_TARGET_PER_ZONE * i]);
			}
			printf("\n");
			loop++;
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		VL53L7CX_WaitMs(&(Dev.platform), 5);
	}

	// Stop ranging before shutdown:
	gpio_set_level(LPN_GPIO, 0); // LPn = LOW (GND) → I²C off, sensor goes to sleep → no interrupts
	status = vl53l7cx_stop_ranging(&Dev);
	gpio_set_level(LED_GPIO, 0); // Ensure LED is off

	printf("End of ULD demo\n");
	return status;
}

void app_main(void)
{
	gpio_init();
	int st = run_example();
	ESP_LOGI(TAG, "Example finished (status=%d)", st);
}
