#ifndef __INFRARED_HOST_H
#define __INFRARED_HOST_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>

#define IR_HOST_CAN_TIMEOUT_MS     500
#define IR_HOST_MAX_RETRY_COUNT     3
#define IR_HOST_FRAME_INTERVAL_MS   50

#define IR_HOST_MODULE_ID            0x01

#define IR_ACK_MAGIC                0xA5
#define IR_NACK_MAGIC               0x5A

#define IR_HOST_CAN_ID_COMMAND      0x101
#define IR_HOST_CAN_ID_DATA         0x102
#define IR_HOST_CAN_ID_ACK          0x103

typedef enum {
    IR_HOST_CMD_PING = 0x01,
    IR_HOST_CMD_SEND_DATA = 0x02,
    IR_HOST_CMD_READ_STATUS = 0x03,
    IR_HOST_CMD_RESET = 0x04
} IR_Host_Command_t;

typedef enum {
    IR_HOST_STATUS_IDLE = 0x00,
    IR_HOST_STATUS_SENDING = 0x01,
    IR_HOST_STATUS_WAIT_ACK = 0x02,
    IR_HOST_STATUS_SUCCESS = 0x03,
    IR_HOST_STATUS_TIMEOUT = 0x04,
    IR_HOST_STATUS_NACK = 0x05,
    IR_HOST_STATUS_ERROR = 0x06
} IR_Host_Status_t;

typedef struct {
    uint8_t module_id;
    uint8_t command;
    uint8_t data[6];
    uint8_t length;
} IR_Host_CommandFrame_t;

typedef struct {
    uint8_t module_id;
    uint8_t status;
    uint8_t data[8];
    uint8_t length;
} IR_Host_ResponseFrame_t;

typedef struct {
    IR_Host_Status_t status;
    IR_Host_ResponseFrame_t last_response;
    uint32_t last_tx_time;
    bool busy;
} IR_Host_Context_t;

extern IR_Host_Context_t ir_host_context;

void IR_Host_Init(void);
bool IR_Host_SendCommand(IR_Host_Command_t cmd, uint8_t *data, uint8_t length);
bool IR_Host_SendDataWithRetry(uint8_t *data, uint8_t length, uint8_t max_retry);
IR_Host_Status_t IR_Host_GetStatus(void);
void IR_Host_ProcessRxFrame(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
bool IR_Host_WaitResponse(uint32_t timeout_ms);
void IR_Host_ClearStatus(void);
uint8_t IR_Host_CRC8(uint8_t *data, uint8_t length);

bool IR_Host_Ping(uint32_t timeout_ms);
bool IR_Host_ReadStatus(uint8_t *status, uint32_t timeout_ms);
bool IR_Host_ResetModule(uint32_t timeout_ms);

void IR_Host_ConfigCanFilter(void);
void IR_Host_StartCan(void);
void IR_Host_TxMailboxCompleteCallback(CAN_HandleTypeDef *hcan);
void IR_Host_RxFifo0Callback(CAN_HandleTypeDef *hcan);

#endif