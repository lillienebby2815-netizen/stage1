#ifndef CBOARD_CORE_INC_CAN_H_
#define CBOARD_CORE_INC_CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern CAN_HandleTypeDef hcan1;

/**
 * Initialise CAN1 at 1 Mbit/s and enable FIFO0 receive interrupts.
 *
 * The acceptance filter is intentionally open during the first hardware
 * bring-up so that the real GM6020 identifiers can be observed before the
 * filter is narrowed.
 */
void MX_CAN1_Init(void);

/**
 * Send one standard CAN data frame through CAN1.
 *
 * This is an interface only at this stage. No task calls it automatically,
 * so configuring CAN cannot energise a motor by itself.
 */
HAL_StatusTypeDef cboard_can_send_standard(
  uint16_t standard_id, const uint8_t * data, uint8_t length);

extern volatile uint32_t cboard_can_last_error;
extern volatile uint32_t cboard_can_rx_count;
extern volatile uint16_t cboard_can_last_rx_id;
extern volatile uint32_t cboard_can_tx_count;
extern volatile uint32_t cboard_can_last_tx_status;

#ifdef __cplusplus
}
#endif

#endif  // CBOARD_CORE_INC_CAN_H_
