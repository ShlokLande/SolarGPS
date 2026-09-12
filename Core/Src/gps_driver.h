/******************************************************************************
* @file    gps.h
* @brief   Stores global GPS functions
******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPS__H__
#define __GPS__H__

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "i2c.h"
#include "main.h"
#include "usart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void gps_task();

void read_uart_gps_module(uint8_t* receive_buffer);

HAL_StatusTypeDef gps_config_meas_rate();

/** CAN SENDING FUNCTION DECLARATIONS */
void CAN_tx_lat_lon_msg(float latitude, float longitude);
void CAN_tx_alt_geod_msg(float altitude, float geodHeight);
void CAN_tx_hdop_vdop_msg(float hdop, float vdop);
void CAN_tx_pdop_speedKmh_msg(float pdop, float speedKmh);
void CAN_tx_true_magnetic_heading_msg(float trueHeading, float magneticHeading);
void CAN_tx_sat_count_view_fix_snr_msg(int satelliteCount, int satInView, int fix, int snr);
void CAN_tx_lon_side_date_msg(char lonSide, char latSide, char date[7], char utcTime[7]);

void CAN_tx_gps_data_msg(GPS* gps_data);


#endif /* __GPS__H__ */