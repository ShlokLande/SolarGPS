
#include "main.h"
#include "usart.h"

#define GPS_DEVICE_ADDRESS ((0x42)<<1)

/* CONFIGURATION HEADERS */
#define STANDARD              0x00
#define TRANSACTION           0x01

#define UBX_HEAD1             0xB5
#define UBX_HEAD2             0x62
#define UBX_CLASS_CFG         0x06
#define UBX_ID_VALSET         0x8A

#define UBX_LAYER_RAM         0x01
#define UBX_LAYER_BBR         0x02
#define UBX_LAYER_FLASH       0x04

#define UBX_KEY_CFG_RATE_MEAS 0x30210001u
#define MEAS_MS               250

#define GPS_CONFIG_DELAY      1000

bool g_gps_read_okay = false;
uint8_t g_gps_data[GPS_MESSAGE_LEN];
char gps_parse_data[GPS_MESSAGE_LEN];

/* CAN HEADERS */
#define GPS_CAN_MESSAGE_LENGTH                         8
#define GPS_CAN_MESSAGE_INT_LENGTH                     4

/**
 * @brief CAN message header for GPS messages
 
CAN_TxHeaderTypeDef gps_sat_count_view_fix_snr_rmc = {
  .StdId = GPS_DATA_SAT_COUNT_VIEW_FIX_SNR_CAN_MESSAGE_ID,
  .IDE = CAN_ID_STD,
  .RTR = CAN_RTR_DATA,
  .DLC = GPS_CAN_MESSAGE_INT_LENGTH
};

CAN_TxHeaderTypeDef gps_lon_lat = {
  .StdId = GPS_DATA_LON_LAT_CAN_MESSAGE_ID,
  .ExtId = 0x0000,
  .IDE = CAN_ID_STD,
  .RTR = CAN_RTR_DATA,
  .DLC = GPS_CAN_MESSAGE_LENGTH
};

CAN_TxHeaderTypeDef gps_hdop_vdop = {
  .StdId = GPS_DATA_HDOP_VDOP_CAN_MESSAGE_ID,
  .ExtId = 0x0000,
  .IDE = CAN_ID_STD,
  .RTR = CAN_RTR_DATA,
  .DLC = GPS_CAN_MESSAGE_LENGTH
};

CAN_TxHeaderTypeDef gps_alt_geod = {
  .StdId = GPS_DATA_ALT_GEOD_CAN_MESSAGE_ID,
  .ExtId = 0x0000,
  .IDE = CAN_ID_STD,
  .RTR = CAN_RTR_DATA,
  .DLC = GPS_CAN_MESSAGE_LENGTH
};

CAN_TxHeaderTypeDef gps_lon_side_date = {
  .StdId = GPS_DATA_LON_SIDE_DATE_CAN_MESSAGE_ID,
  .ExtId = 0x0000,
  .IDE = CAN_ID_STD,
  .RTR = CAN_RTR_DATA,
  .DLC = GPS_CAN_MESSAGE_LENGTH
};

CAN_TxHeaderTypeDef gps_pdop_speedkmh = {
  .StdId = GPS_DATA_PDOP_SPEEDKMH_CAN_MESSAGE_ID,
  .ExtId = 0x0000,
  .IDE = CAN_ID_STD,
  .RTR = CAN_RTR_DATA,
  .DLC = GPS_CAN_MESSAGE_LENGTH
};

CAN_TxHeaderTypeDef gps_true_mag_heading = {
  .StdId = GPS_DATA_TRUE_MAG_HEADING_CAN_MESSAGE_ID,
  .ExtId = 0x0000,
  .IDE = CAN_ID_STD,
  .RTR = CAN_RTR_DATA,
  .DLC = GPS_CAN_MESSAGE_LENGTH
};
*/

/**
 * @brief Continually tries to get a fix and sets if the GPS messages are read or not
 * @param receive_buffer The buffer to store the received data
 */
 void read_uart_gps_module(uint8_t* receive_buffer)
 {
     g_gps_read_okay = false;
     HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, receive_buffer, GPS_MESSAGE_LEN);
     if(status == HAL_OK)
     {
         g_tel_diagnostic_flags.bits.gps_read_fail = false;
     }
     else
     {
         g_tel_diagnostic_flags.bits.gps_read_fail = true;
     }
 }

 /**
 * @brief Reads the GPS data and confirms if it is ready to be parsed into gps_data
 */
void gps_task()
{
    if (g_gps_read_okay)
    {
        GPS gps_data = {0};

        nmea_parse(&gps_data, g_gps_data);

        CAN_tx_gps_data_msg(&gps_data);

        HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    }

    memset(g_gps_data, 0, GPS_MESSAGE_LEN);
    read_uart_gps_module(g_gps_data);
}

/**
 * @brief GPS measurement rate configuration message sent over I2C
 *
 * This function builds a UBX CFG-VALSET message that sets the receiver’s
 * measurement period (CFG-RATE-MEAS) to MEAS_MS in a selected memory layer.
 * It assembles the frame, computes the UBX checksum, then sends it over I2C.
 *
 * @return Sends the configured MEAS_MS over I2C
 */
 HAL_StatusTypeDef gps_config_meas_rate() {

    uint8_t payload[10];
    uint32_t key = UBX_KEY_CFG_RATE_MEAS;

    payload[0] = STANDARD;  // standard or transaction version

    payload[1] = UBX_LAYER_RAM; // memory layer

    payload[2] = 0x00;  // reserved
    payload[3] = 0x00;  // reserved

    payload[4] = (uint8_t)(key & 0xFF);
    payload[5] = (uint8_t)((key >> 8) & 0xFF);
    payload[6] = (uint8_t)((key >> 16) & 0xFF);
    payload[7] = (uint8_t)((key >> 24) & 0xFF);

    payload[8] = (uint8_t)(MEAS_MS & 0xFF);
    payload[9] = (uint8_t)((MEAS_MS >> 8) & 0xFF);

    uint16_t len = sizeof(payload);

    uint8_t frame[8 + len];

    frame[0] = UBX_HEAD1;
    frame[1] = UBX_HEAD2;
    frame[2] = UBX_CLASS_CFG;
    frame[3] = UBX_ID_VALSET;
    frame[4] = (uint8_t)(len & 0xFF);
    frame[5] = (uint8_t)((len >> 8) & 0xFF);
    
    memcpy(&frame[6], payload, len);

    uint8_t ckA;
    uint8_t ckB;

    ubx_cksum(&frame[2], 4 + len, &ckA, &ckB); // len + 4 the length of the payload + 4 bytes of header in frame

    frame[6 + len] = ckA;
    frame[7 + len] = ckB;

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, GPS_DEVICE_ADDRESS, frame, sizeof(frame), GPS_CONFIG_DELAY);

    return status;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        g_gps_read_okay = true;
    }
}