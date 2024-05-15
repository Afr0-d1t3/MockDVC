/*
 * TOF_App.c
 *
 *  Created on: Jun 27, 2023
 *      Author: E1441590
 *  Summary: Will hold all functions for the TOF sensor in background
 */


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "main.h"
#include "TOF_App.h"
#include "vl53lx_api.h"
#include "vl53l4cx.h"
#include "custom_tof_conf.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TIMING_BUDGET (30U) /* 10 ms < TimingBudget < 200 ms */
#define POLLING_PERIOD (250U) /* refresh rate for polling mode (milliseconds, shall be consistent with TimingBudget value) */

/* Private variables ---------------------------------------------------------*/
static VL53L4CX_Object_t dev_sat[USE_CUSTOM_RANGING_VL53L4CX];  // The main TOF sensor object
static VL53L4CX_Capabilities_t Cap;  							// Capabilities of Sensor
static VL53L4CX_ProfileConfig_t Profile;  						// Profile for sensor
static int32_t BSP_Status = 0;  								// In case of any communication errors
static VL53LX_Error TOF_Status = VL53LX_ERROR_NONE; 			// In case of sensor errors

// UART Variables
char buf[100];  												// sprintf buffer
extern UART_HandleTypeDef huart2;  								//HUART2 typedef
/* Private function prototypes -----------------------------------------------*/
static void print_result(VL53L4CX_Result_t *Result);
static int32_t decimal_part(float_t x);

void MX_TOF_Init(void)
{
	VL53L4CX_IO_t IOCtx;
	uint32_t Id;

	/* reset XSHUT (XSDN) pin */
	HAL_GPIO_WritePin(CUSTOM_VL53L4CX_XSHUT_PORT, CUSTOM_VL53L4CX_XSHUT_PIN, GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(CUSTOM_VL53L4CX_XSHUT_PORT, CUSTOM_VL53L4CX_XSHUT_PIN, GPIO_PIN_SET);
	HAL_Delay(2);

	BSP_Status = BSP_I2C1_Init();  // initialize the i2c
	if (BSP_Status != BSP_ERROR_NONE)
	{
		printf("VL53L4CX_RANGING_SENSOR_Init failed\n");
		while(1);
	}

	/* Configure the ranging sensor driver */
	IOCtx.Address     = (VL53L4CX_DEVICE_ADDRESS); // Check it is advertised as 0x52, but shifted left is 0x29 becuz of 7-bit address
	IOCtx.Init        = CUSTOM_VL53L4CX_I2C_Init;
	IOCtx.DeInit      = CUSTOM_VL53L4CX_I2C_DeInit;
	IOCtx.WriteReg    = CUSTOM_VL53L4CX_I2C_WriteReg;
	IOCtx.ReadReg     = CUSTOM_VL53L4CX_I2C_ReadReg;
	IOCtx.GetTick     = (long int(*)(void))CUSTOM_VL53L4CX_GetTick;

	// Register BusIO
	TOF_Status = VL53L4CX_RegisterBusIO(dev_sat, &IOCtx);
	if (TOF_Status != VL53LX_ERROR_NONE) { while(1); }

	// Data Init (Most important)
	TOF_Status = VL53LX_WaitDeviceBooted(dev_sat);
	if (TOF_Status == VL53LX_ERROR_NONE)
	{
		TOF_Status = VL53LX_DataInit(dev_sat);
	}
	else { while(1); }

	// Read device credentials
	VL53L4CX_ReadID(dev_sat, &Id);
	VL53L4CX_GetCapabilities(dev_sat, &Cap);

	// Configure Profile
	Profile.RangingProfile = VL53L4CX_PROFILE_MEDIUM;  // May change to short based on needs
	Profile.TimingBudget = TIMING_BUDGET; /* 10 ms < TimingBudget < 200 ms */
	Profile.Frequency = 0; /* Induces intermeasurement period, NOT USED for normal ranging */
	Profile.EnableAmbient = 1; /* Enable: 1, Disable: 0 */
	Profile.EnableSignal = 1; /* Enable: 1, Disable: 0 */
	VL53L4CX_ConfigProfile(dev_sat, &Profile);

	// Start ranging process
	VL53L4CX_Start(dev_sat, VL53L4CX_MODE_BLOCKING_CONTINUOUS);
}

void MX_TOF_Process(void)
{
	VL53L4CX_Result_t Result;
	/* polling mode */
	// Get the distance
	BSP_Status = VL53L4CX_GetDistance(dev_sat, &Result);

	if (BSP_Status == BSP_ERROR_NONE)
	{
		print_result(&Result);  // Print to serial port
	}
	HAL_Delay(POLLING_PERIOD);
}

static void print_result(VL53L4CX_Result_t *Result)
{
  uint8_t i, j;

  for (i = 0; i < VL53L4CX_MAX_NB_ZONES; i++)
  {
    sprintf(buf, "\nTargets = %lu", (unsigned long)Result->ZoneResult[i].NumberOfTargets);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

    for (j = 0; j < Result->ZoneResult[i].NumberOfTargets; j++)
    {
      sprintf(buf, "\n |---> ");
      HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

      sprintf(buf, "BSP_Status = %ld, Distance = %5ld mm ",
        (long)Result->ZoneResult[i].Status[j],
        (long)Result->ZoneResult[i].Distance[j]);
      HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

      if (Profile.EnableAmbient)
        sprintf(buf, ", Ambient = %ld.%02ld kcps/spad",
          (long)Result->ZoneResult[i].Ambient[j],
          (long)decimal_part(Result->ZoneResult[i].Ambient[j]));
      HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

      if (Profile.EnableSignal)
        sprintf(buf, ", Signal = %ld.%02ld kcps/spad\r",
          (long)Result->ZoneResult[i].Signal[j],
          (long)decimal_part(Result->ZoneResult[i].Signal[j]));
      HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    }
  }
  sprintf (buf, "\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}

static int32_t decimal_part(float_t x)
{
  int32_t int_part = (int32_t) x;
  return (int32_t)((x - int_part) * 100);
}

#ifdef __cplusplus
}
#endif
