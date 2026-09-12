/******************************************************************************
* @file    gps.h
* @brief   Stores global GPS functions
******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPS_DRIVER_H__
#define __GPS_DRIVER_H__

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "i2c.h"
#include "main.h"
#include "usart.h"
#include "gps_app.h"
#include "nmea_parse.h"

extern uint8_t g_gps_data[GPS_MESSAGE_LEN];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void gps_task();

void read_uart_gps_module(uint8_t* receive_buffer);

HAL_StatusTypeDef gps_config_meas_rate();

#endif /* __GPS_DRIVER_H__ */