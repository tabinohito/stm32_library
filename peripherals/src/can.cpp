#include "../Inc/can.hpp"

#if defined(HAL_CAN_MODULE_ENABLED)
// CAN初期化
stm32_library::stm32_peripherals::Can::Can(CanHandleType *handle, uint32_t filter_id, uint32_t filter_mask)
    : handle_(handle) {
  if (handle_->State == HAL_CAN_STATE_READY) {
    if (start(filter_id, filter_mask) != HAL_OK) {
      return;
    }
  }
}

HAL_StatusTypeDef stm32_library::stm32_peripherals::Can::start(uint32_t filter_id, uint32_t filter_mask) {
  CAN_FilterTypeDef filter;
  filter.FilterIdHigh = filter_id << 5;
  filter.FilterIdLow = filter_id << 21;
  filter.FilterMaskIdHigh = filter_mask << 5;
  filter.FilterMaskIdLow = filter_mask << 21;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterBank = 0;

#if defined(CAN2)
  if (handle_->Instance == CAN2)
    filter.FilterBank = 14;
#endif

  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(handle_, &filter);
  if (status != HAL_OK) {
    return status;
  }

  status = HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
  if (status != HAL_OK) {
    return status;
  }

  return HAL_CAN_Start(handle_);
}

HAL_StatusTypeDef stm32_library::stm32_peripherals::Can::restart(
    uint32_t mode,
    uint32_t filter_id,
    uint32_t filter_mask
) {
  BitTimingConfig config{};
  config.prescaler = handle_ != nullptr ? handle_->Init.Prescaler : 1U;
  config.mode = mode;
  config.sync_jump_width =
      handle_ != nullptr ? handle_->Init.SyncJumpWidth : CAN_SJW_1TQ;
  config.time_segment1 =
      handle_ != nullptr ? handle_->Init.TimeSeg1 : CAN_BS1_1TQ;
  config.time_segment2 =
      handle_ != nullptr ? handle_->Init.TimeSeg2 : CAN_BS2_1TQ;
  config.time_triggered_mode =
      handle_ != nullptr ? handle_->Init.TimeTriggeredMode : DISABLE;
  config.auto_bus_off =
      handle_ != nullptr ? handle_->Init.AutoBusOff : DISABLE;
  config.auto_wakeup =
      handle_ != nullptr ? handle_->Init.AutoWakeUp : DISABLE;
  config.auto_retransmission =
      handle_ != nullptr ? handle_->Init.AutoRetransmission : ENABLE;
  config.receive_fifo_locked =
      handle_ != nullptr ? handle_->Init.ReceiveFifoLocked : DISABLE;
  config.transmit_fifo_priority =
      handle_ != nullptr ? handle_->Init.TransmitFifoPriority : DISABLE;
  config.filter_id = filter_id;
  config.filter_mask = filter_mask;
  return configure(config);
}

HAL_StatusTypeDef stm32_library::stm32_peripherals::Can::stop() {
  if (handle_ == nullptr) {
    return HAL_ERROR;
  }

  if (handle_->State == HAL_CAN_STATE_LISTENING) {
    return HAL_CAN_Stop(handle_);
  }

  return HAL_OK;
}

HAL_StatusTypeDef stm32_library::stm32_peripherals::Can::configure(
    const BitTimingConfig& config
) {
  if (handle_ == nullptr) {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = stop();
  if (status != HAL_OK) {
    return status;
  }

  status = HAL_CAN_DeInit(handle_);
  if (status != HAL_OK) {
    return status;
  }

  handle_->Init.Prescaler = config.prescaler;
  handle_->Init.Mode = config.mode;
  handle_->Init.SyncJumpWidth = config.sync_jump_width;
  handle_->Init.TimeSeg1 = config.time_segment1;
  handle_->Init.TimeSeg2 = config.time_segment2;
  handle_->Init.TimeTriggeredMode = config.time_triggered_mode;
  handle_->Init.AutoBusOff = config.auto_bus_off;
  handle_->Init.AutoWakeUp = config.auto_wakeup;
  handle_->Init.AutoRetransmission = config.auto_retransmission;
  handle_->Init.ReceiveFifoLocked = config.receive_fifo_locked;
  handle_->Init.TransmitFifoPriority = config.transmit_fifo_priority;

  status = HAL_CAN_Init(handle_);
  if (status != HAL_OK) {
    return status;
  }

  return start(config.filter_id, config.filter_mask);
}

// CAN送信
HAL_StatusTypeDef stm32_library::stm32_peripherals::Can::write(uint32_t id, uint8_t *data, uint32_t size, bool blocking) {
  CAN_TxHeaderTypeDef tx_header;
  const bool is_extended = ((id & CanEffFlag) != 0U) || ((id & ~CanEffFlag) > CanSffMask);

  if (is_extended) {
    tx_header.StdId = 0;
    tx_header.ExtId = id & CanEffMask;
    tx_header.IDE = CAN_ID_EXT;
  } else {
    tx_header.StdId = id & CanSffMask;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
  }

  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = size > 8 ? 8 : size;
  tx_header.TransmitGlobalTime = DISABLE;
  uint32_t tx_mailbox;

  while (blocking && HAL_CAN_GetTxMailboxesFreeLevel(handle_) == 0)
    ;

  return HAL_CAN_AddTxMessage(handle_, &tx_header, data, &tx_mailbox);
}

#ifdef CAN_RX_FIFO0
// コールバックの実装
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  using namespace stm32_library::stm32_peripherals;
  CAN_RxHeaderTypeDef rx_header;
  stm32_library::stm32_peripherals::CanMessage msg;

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, msg.data.data()) == HAL_OK) {
    if (rx_header.IDE == CAN_ID_EXT) {
      msg.id = CanEffFlag | (rx_header.ExtId & CanEffMask);
    } else {
      msg.id = rx_header.StdId & CanSffMask;
    }
    msg.size = rx_header.DLC;
    callback::callback<Can::CallbackFnType>(reinterpret_cast<intptr_t>(hcan), msg);
  }
}
#endif

#elif defined(HAL_FDCAN_MODULE_ENABLED)
// FDCAN初期化
stm32_library::stm32_peripherals::Can::Can(CanHandleType *handle, uint32_t filter_id, uint32_t filter_mask)
    : handle_(handle) {
  if (handle_->State == HAL_FDCAN_STATE_READY) {
    FDCAN_FilterTypeDef filter;
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = filter_id;
    filter.FilterID2 = filter_mask;

    if (HAL_FDCAN_ConfigFilter(handle_, &filter) != HAL_OK) {
      Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(handle_, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) !=
        HAL_OK) {
      Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(handle_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
      Error_Handler();
    }

    if (HAL_FDCAN_Start(handle_) != HAL_OK) {
      Error_Handler();
    }
  }
}

// FDCAN送信
HAL_StatusTypeDef stm32_library::stm32_peripherals::Can::write(uint32_t id, uint8_t *data, uint32_t size, bool blocking) {
  FDCAN_TxHeaderTypeDef tx_header;
  const bool is_extended = ((id & CanEffFlag) != 0U) || ((id & ~CanEffFlag) > CanSffMask);

  tx_header.Identifier = is_extended ? (id & CanEffMask) : (id & CanSffMask);
  tx_header.IdType = is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = (size > 8 ? 8 : size) << 16;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  while (blocking && HAL_FDCAN_GetTxFifoFreeLevel(handle_) == 0)
    ;

  return HAL_FDCAN_AddMessageToTxFifoQ(handle_, &tx_header, data);
}

#ifdef FDCAN_RX_FIFO0
// コールバックの実装
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  using namespace stm32_library::stm32_peripherals;
  FDCAN_RxHeaderTypeDef rx_header;
  stm32_library::stm32_peripherals::CanMessage msg;

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, msg.data.data()) == HAL_OK) {
    if (rx_header.IdType == FDCAN_EXTENDED_ID) {
      msg.id = CanEffFlag | (rx_header.Identifier & CanEffMask);
    } else {
      msg.id = rx_header.Identifier & CanSffMask;
    }
    msg.size = rx_header.DataLength >> 16;
    callback::callback<Can::CallbackFnType>(reinterpret_cast<intptr_t>(hfdcan), msg);
  }
}
#endif
#endif
