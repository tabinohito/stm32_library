#pragma once

#include <stdio.h>

namespace stm32_library::stm32_modules
{
	static inline void dwt_init()
	{
	  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	  DWT->CYCCNT = 0;
	  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}

	static inline uint32_t HAL_GetTick_us()
	{
	  return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000U);
	}

	static inline void HAL_Delay_us(uint32_t ms)
	{
		uint32_t tim = HAL_GetTick_us();
		while(HAL_GetTick_us() - tim < ms){
			continue;
		}
		return;
	}

}
