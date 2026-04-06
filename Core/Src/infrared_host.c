#include "infrared_host.h"
#include "can.h"

extern CAN_HandleTypeDef hcan1;

static volatile bool tx_complete_flag = false;
static volatile bool rx_received_flag = false;

IR_Host_Context_t ir_host_context = {
    .status = IR_HOST_STATUS_IDLE,
    .busy = false,
    .last_tx_time = 0
};

static void IR_Host_TxCompleteCallback(void)
{
    tx_complete_flag = true;
}

void IR_Host_Init(void)
{
    ir_host_context.status = IR_HOST_STATUS_IDLE;
    ir_host_context.busy = false;
    ir_host_context.last_tx_time = 0;
    memset(&ir_host_context.last_response, 0, sizeof(IR_Host_ResponseFrame_t));
    tx_complete_flag = false;
    rx_received_flag = false;
}

uint8_t IR_Host_CRC8(uint8_t *data, uint8_t length)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static bool IR_Host_TransmitFrame(uint32_t can_id, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    if (length > 8) {
        return false;
    }

    memcpy(tx_data, data, length);

    tx_header.StdId = can_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_complete_flag = false;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
        return false;
    }

    uint32_t start_time = HAL_GetTick();
    while (!tx_complete_flag) {
        if ((HAL_GetTick() - start_time) > 100) {
            return false;
        }
    }

    return true;
}

bool IR_Host_SendCommand(IR_Host_Command_t cmd, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    if (length > 6) {
        return false;
    }

    if (ir_host_context.busy) {
        return false;
    }

    uint32_t time_since_last_tx = HAL_GetTick() - ir_host_context.last_tx_time;
    if (time_since_last_tx < IR_HOST_FRAME_INTERVAL_MS) {
        return false;
    }

    tx_data[0] = IR_HOST_MODULE_ID;
    tx_data[1] = cmd;
    if (data != NULL && length > 0) {
        memcpy(&tx_data[2], data, length);
    }
    tx_data[7] = IR_Host_CRC8(tx_data, 7);

    tx_header.StdId = IR_HOST_CAN_ID_COMMAND;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    ir_host_context.busy = true;
    ir_host_context.status = IR_HOST_STATUS_SENDING;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
        ir_host_context.busy = false;
        ir_host_context.status = IR_HOST_STATUS_ERROR;
        return false;
    }

    ir_host_context.last_tx_time = HAL_GetTick();
    ir_host_context.status = IR_HOST_STATUS_WAIT_ACK;

    return true;
}

bool IR_Host_SendDataWithRetry(uint8_t *data, uint8_t length, uint8_t max_retry)
{
    if (length > 8) {
        return false;
    }

    for (uint8_t retry = 0; retry < max_retry; retry++) {
        while (ir_host_context.busy) {
            HAL_Delay(1);
        }

        ir_host_context.status = IR_HOST_STATUS_IDLE;

        CAN_TxHeaderTypeDef tx_header;
        uint32_t tx_mailbox;
        uint8_t tx_data[9];

        tx_data[0] = IR_HOST_MODULE_ID;
        memcpy(&tx_data[1], data, length);
        tx_data[length + 1] = IR_Host_CRC8(tx_data, length + 1);

        tx_header.StdId = IR_HOST_CAN_ID_DATA;
        tx_header.ExtId = 0;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = length + 2;
        tx_header.TransmitGlobalTime = DISABLE;

        ir_host_context.busy = true;
        ir_host_context.status = IR_HOST_STATUS_SENDING;

        if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) == HAL_OK) {
            ir_host_context.last_tx_time = HAL_GetTick();

            uint32_t start_time = HAL_GetTick();
            while ((HAL_GetTick() - start_time) < IR_HOST_CAN_TIMEOUT_MS) {
                if (rx_received_flag) {
                    rx_received_flag = false;
                    if (ir_host_context.last_response.status == IR_HOST_STATUS_SUCCESS) {
                        ir_host_context.busy = false;
                        return true;
                    } else if (ir_host_context.last_response.status == IR_HOST_STATUS_NACK) {
                        break;
                    }
                }
                HAL_Delay(10);
            }
        }

        ir_host_context.busy = false;
        ir_host_context.status = IR_HOST_STATUS_TIMEOUT;
        HAL_Delay(50);
    }

    return false;
}

IR_Host_Status_t IR_Host_GetStatus(void)
{
    return ir_host_context.status;
}

void IR_Host_ProcessRxFrame(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    if (rx_header->StdId == IR_HOST_CAN_ID_ACK) {
        if (rx_data[0] == IR_ACK_MAGIC && rx_data[1] == IR_ACK_MAGIC) {
            ir_host_context.last_response.status = IR_HOST_STATUS_SUCCESS;
        } else if (rx_data[0] == IR_NACK_MAGIC && rx_data[1] == IR_NACK_MAGIC) {
            ir_host_context.last_response.status = IR_HOST_STATUS_NACK;
        }
        rx_received_flag = true;
        return;
    }

    if (rx_header->StdId == IR_HOST_CAN_ID_DATA) {
        uint8_t module_id = rx_data[0];
        uint8_t received_crc = rx_data[rx_header->DLC - 1];
        uint8_t calculated_crc = IR_Host_CRC8(rx_data, rx_header->DLC - 1);

        if (received_crc == calculated_crc) {
            ir_host_context.last_response.module_id = module_id;
            ir_host_context.last_response.status = IR_HOST_STATUS_SUCCESS;
            ir_host_context.last_response.length = rx_header->DLC - 2;
            memcpy(ir_host_context.last_response.data, &rx_data[1], rx_header->DLC - 2);
        } else {
            ir_host_context.last_response.status = IR_HOST_STATUS_ERROR;
        }
        rx_received_flag = true;
    }
}

bool IR_Host_WaitResponse(uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();

    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (rx_received_flag) {
            rx_received_flag = false;
            return true;
        }
        HAL_Delay(10);
    }

    return false;
}

void IR_Host_ClearStatus(void)
{
    ir_host_context.status = IR_HOST_STATUS_IDLE;
    ir_host_context.busy = false;
    memset(&ir_host_context.last_response, 0, sizeof(IR_Host_ResponseFrame_t));
    rx_received_flag = false;
}

void IR_Host_TxMailboxCompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        tx_complete_flag = true;
    }
}

void IR_Host_RxFifo0Callback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            IR_Host_ProcessRxFrame(&rx_header, rx_data);
        }
    }
}

bool IR_Host_Ping(uint32_t timeout_ms)
{
    if (!IR_Host_SendCommand(IR_HOST_CMD_PING, NULL, 0)) {
        return false;
    }

    if (IR_Host_WaitResponse(timeout_ms)) {
        return (ir_host_context.last_response.status == IR_HOST_STATUS_SUCCESS);
    }

    return false;
}

bool IR_Host_ReadStatus(uint8_t *status, uint32_t timeout_ms)
{
    if (!IR_Host_SendCommand(IR_HOST_CMD_READ_STATUS, NULL, 0)) {
        return false;
    }

    if (IR_Host_WaitResponse(timeout_ms)) {
        if (status != NULL && ir_host_context.last_response.length > 0) {
            *status = ir_host_context.last_response.data[0];
        }
        return true;
    }

    return false;
}

bool IR_Host_ResetModule(uint32_t timeout_ms)
{
    if (!IR_Host_SendCommand(IR_HOST_CMD_RESET, NULL, 0)) {
        return false;
    }

    return IR_Host_WaitResponse(timeout_ms);
}