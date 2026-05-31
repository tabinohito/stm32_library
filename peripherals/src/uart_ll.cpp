#include "../Inc/uart_ll.hpp"

#ifdef HAL_UART_MODULE_ENABLED

extern "C" void uart_ll_irq_callback_c(UART_HandleTypeDef *huart)
{
    if (huart == nullptr) {
        return;
    }

    stm32_library::stm32_peripherals::uart_ll_irq_handler(huart->Instance);
}

extern "C" void uart_ll_irq_callback_with_hal_rx_c(UART_HandleTypeDef *huart)
{
    if (huart == nullptr) {
        return;
    }

    stm32_library::stm32_peripherals::uart_ll_irq_handler_with_hal_rx(huart->Instance);
}

extern "C" void uart_ll_dma_tx_complete_callback_c(UART_HandleTypeDef *huart)
{
    if (huart == nullptr) {
        return;
    }

    stm32_library::stm32_peripherals::uart_ll_dma_tx_complete_handler(huart);
}

//extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//    uart_ll_dma_tx_complete_callback_c(huart);
//
//}

#endif
