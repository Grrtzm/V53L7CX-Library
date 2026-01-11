/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#ifndef VL53L7CX_BUFFERS_H_
#define VL53L7CX_BUFFERS_H_

#include "platform.h"

/**
 * @brief Inner internal number of targets.
 */

#if VL53L7CX_NB_TARGET_PER_ZONE == 1
#define VL53L7CX_FW_NBTAR_RANGING	2
#else
#define VL53L7CX_FW_NBTAR_RANGING	VL53L7CX_NB_TARGET_PER_ZONE
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Firmware was moved to the vl53l7cx_buffers.c file to reduce RAM/flash usage when not needed.
extern const uint8_t VL53L7CX_FIRMWARE[];
extern const uint32_t VL53L7CX_FIRMWARE_SIZE;

extern const uint8_t VL53L7CX_DEFAULT_CONFIGURATION[];
extern const uint32_t VL53L7CX_DEFAULT_CONFIGURATION_SIZE;

extern const uint8_t VL53L7CX_DEFAULT_XTALK[];
extern const uint32_t VL53L7CX_DEFAULT_XTALK_SIZE;

extern const uint8_t VL53L7CX_GET_NVM_CMD[];
extern const uint32_t VL53L7CX_GET_NVM_CMD_SIZE;

#ifdef __cplusplus
}
#endif

#endif /* VL53L7CX_BUFFERS_H_ */
	
