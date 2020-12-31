/**
 * @brief   This file provides all the RCC(reset and clock control) firmware functions.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <inc/flash.h>

namespace stm32f10x_driver_lib {

void Flash::setLatency(uint32_t latency) const
{
	FLASH->ACR |= latency;
}

void Flash::enablePrefetchBuffer() const
{
	FLASH->ACR |= FLASH_ACR_PRFTBE;
}

void Flash::disablePrefetchBuffer() const
{
	FLASH->ACR &= ~FLASH_ACR_PRFTBE;
}

} // namespace stm32f10x_driver_lib
