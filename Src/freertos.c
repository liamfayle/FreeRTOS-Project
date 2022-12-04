/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  * 		-- Liam Fayle modified on 2022-11-25
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdarg.h>

/* USER CODE BEGIN Includes */     
#include "usbd_cdc_if.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "spi.h"
#include "i2c.h"
#include "camera.h"


uint8_t SIZE = 10;

/* Variables -----------------------------------------------------------------*/
osMessageQId myQueue01Handle;
osMutexId myMutex01Handle;
osSemaphoreId myBinarySem01Handle;
osSemaphoreId alertSemHandle;

/* USER CODE BEGIN Variables */
uint8_t RxData[256];
uint32_t data_received;
uint8_t Str4Display[50];
osThreadId TaskGUIHandle;
osThreadId TaskCameraHandle;
osThreadId TaskAlertHandle;
osMessageQId CommQueueHandle;
osMutexId dataMutexHandle;
osSemaphoreId printSemHandle;
osSemaphoreId lcdSemHandle;

//OUR THREADS
osThreadId TaskMotionHandle;

typedef struct
{
	uint8_t Value[10];
	uint8_t Source;
}data;

data DataToSend={"Hello\0", 1};
data DataVCP={"VCP\0",2};
uint8_t sensed[10];
uint8_t currentIndex = 0;
uint8_t detected = 0;
uint8_t guiUpdate = 0;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void);
int myprintf(const char *format, ...);
void StartTaskGUI(void const * argument);
void StartTaskCamera(void const * argument);
void StartTaskMotion(void const * argument);
void StartTaskAlert(void const * argument);


void addValues(uint8_t output) {
	if (currentIndex == SIZE) {
		memmove(&sensed[0], &sensed[1], (SIZE - 1) * sizeof(sensed[0]));
		sensed[SIZE-1] = output;
		currentIndex = SIZE-1;
	} else {
		sensed[currentIndex] = output;
	}

	currentIndex++;

}


uint8_t checkMotion() {
	for (uint8_t i = 0; i < SIZE; i++) {
		if (sensed[i] == 0) {
			return 0;
		}
	}

	return 1;
}


void clearSensed() {
	for (uint8_t i = 0; i < SIZE; i++) {
		sensed[i] = 0;
	}
}



void detectedMessage(uint8_t c) {
	if (c % 2 == 0) {
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"Intruder Detected!");
	} else {
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"Intruder Detected!");
	}
}

void clearLine1() {
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"                        ");
}

void clearPicture() {
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	for (uint8_t i = 0; i < 13; i++) {
		BSP_LCD_DisplayStringAtLine(i+2, (uint8_t *)"                        ");
	}
}


void clearMessage() {
	clearLine1();
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"   All Clear! ");
}


void MX_FREERTOS_Init(void) {

	osMutexDef(dataMutex);
	dataMutexHandle = osMutexCreate(osMutex(dataMutex));

	/* definition and creation of printSem */
	osSemaphoreDef(printSem);
	printSemHandle = osSemaphoreCreate(osSemaphore(printSem), 1);

	/* definition and creation of lcdSem */
	osSemaphoreDef(lcdSem);
	lcdSemHandle = osSemaphoreCreate(osSemaphore(lcdSem), 1);

	osSemaphoreDef(alertSem);
	alertSemHandle = osSemaphoreCreate(osSemaphore(alertSem), 1);

	osThreadDef(TaskGUI, StartTaskGUI, osPriorityNormal, 0, 128);
	TaskGUIHandle = osThreadCreate(osThread(TaskGUI), NULL);

	osThreadDef(TaskCamera, StartTaskCamera, osPriorityNormal, 0, 1024);
	TaskCameraHandle = osThreadCreate(osThread(TaskCamera), NULL);

	osThreadDef(TaskMotion, StartTaskMotion, osPriorityHigh, 0, 128);
	TaskMotionHandle = osThreadCreate(osThread(TaskMotion), NULL);

	osThreadDef(TaskAlert, StartTaskAlert, osPriorityNormal, 0, 128);
	TaskAlertHandle = osThreadCreate(osThread(TaskAlert), NULL);

	/* Comm QUEUE, don't delete */
	osMessageQDef(CommQueue, 1, &DataVCP);
	CommQueueHandle = osMessageCreate(osMessageQ(CommQueue), NULL);
}


void StartTaskGUI(void const * argument)
{
	uint8_t count = 0;
  while(1)
  {

	  osSemaphoreWait(lcdSemHandle, osWaitForever);
	  if (detected) {
		  guiUpdate=1;
		  detectedMessage(count);
		  count++;
	  } else {
		  guiUpdate=0;
		  clearMessage();
		  clearPicture();
	  }
	  osSemaphoreRelease(lcdSemHandle);

	  osDelay(500);
  }
}


void StartTaskCamera(void const * argument)
{

	camera_setup();

	while (1) {

		osSemaphoreWait(lcdSemHandle, osWaitForever);
		if (detected && guiUpdate)
			camera_initiate_capture();
		osSemaphoreRelease(lcdSemHandle);

		osDelay(50);
	}
}

int myprintf(const char *format, ...)
{
	osSemaphoreWait(printSemHandle, osWaitForever);
	va_list alist;
	va_start(alist, format);
	vprintf(format, alist);
    va_end(alist);
	osSemaphoreRelease(printSemHandle);
	return 0;
}



void StartTaskMotion(void const * argument)
{

	clearSensed();

  while(1)
  {
	 detected = 0;
	 if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET){
		  addValues(1);
	  } else {
		  addValues(0);
	  }

	  if (checkMotion()) {
		  myprintf("Intruder Detected! \n\r");
		  detected = 1;
		  clearSensed();
	  }
	  if (detected) {
		  osDelay(5000); //time alert stays active!
	  } else {
		  osDelay(50);
	  }
  }
}



void StartTaskAlert(void const * argument)
{

	uint8_t i = 0;
  while(1)
  {
	  if (detected) {
		  if (i % 2 == 0) {
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 1);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
		  } else {
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 0);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
		  }
		  i++;
	  } else {
		  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 0);
		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
	  }
	  osDelay(500);
  }
}
