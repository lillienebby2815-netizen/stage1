#include "can.h"

#include "stm32f4xx_hal_can.h"

CAN_HandleTypeDef hcan1;
volatile uint32_t cboard_can_last_error = HAL_CAN_ERROR_NONE;
volatile uint32_t cboard_can_rx_count = 0U;
volatile uint16_t cboard_can_last_rx_id = 0U;
volatile uint32_t cboard_can_tx_count = 0U;
volatile uint32_t cboard_can_last_tx_status = HAL_OK;

/* Implemented in the C++ motor-state layer. The callback only forwards
 * received standard data frames; it does not issue a motor command. */
extern void cboard_motor_can_on_feedback(
  uint16_t standard_id, const uint8_t * data, uint8_t length);

void MX_CAN1_Init(void)
{
  CAN_FilterTypeDef filter = {0};

  hcan1.Instance = CAN1;
  /* APB1 is 42 MHz. 3 * (1 + 10 + 3) time quanta gives 1 Mbit/s. */
  hcan1.Init.Prescaler = 3U;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }

  /* Receive every standard frame while identifying the real motor IDs. */
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0U;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0U;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.SlaveStartFilterBank = 14U;
  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
}

HAL_StatusTypeDef cboard_can_send_standard(
  uint16_t standard_id, const uint8_t * data, uint8_t length)
{
  if (standard_id > 0x7FFU || data == NULL || length > 8U) {
    return HAL_ERROR;
  }

  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox = 0U;
  header.StdId = standard_id;
  header.ExtId = 0U;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = length;
  header.TransmitGlobalTime = DISABLE;
  const HAL_StatusTypeDef status =
    HAL_CAN_AddTxMessage(&hcan1, &header, (uint8_t *)data, &mailbox);
  cboard_can_last_tx_status = (uint32_t)status;
  if (status == HAL_OK) {
    cboard_can_tx_count++;
  }
  return status;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  CAN_RxHeaderTypeDef header = {0};
  uint8_t data[8] = {0};

  if (hcan != &hcan1) {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      cboard_can_last_error = hcan->ErrorCode;
      return;
    }
    cboard_can_last_rx_id = (uint16_t)header.StdId;
    cboard_can_rx_count++;
    if (header.IDE == CAN_ID_STD && header.RTR == CAN_RTR_DATA) {
      cboard_motor_can_on_feedback(
        (uint16_t)header.StdId, data, (uint8_t)header.DLC);
    }
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan == &hcan1) {
    cboard_can_last_error = hcan->ErrorCode;
  }
}
