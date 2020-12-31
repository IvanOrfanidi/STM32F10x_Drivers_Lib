/**
 * @brief   This is file realise SPI interface.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_H
#define __FLASH_H

/* Includes ------------------------------------------------------------------*/
/* Standard lib */
#include <stdint.h>

/* Driver MCU */
#include <CMSIS/Include/stm32f10x.h>

#ifdef __cplusplus

namespace stm32f10x_driver_lib {

class Flash
{
  public:
	void setLatency(uint32_t) const;

	void enablePrefetchBuffer() const;

	void disablePrefetchBuffer() const;
};

} // namespace stm32f10x_driver_lib
extern "C" {
}

#endif // __cplusplus

#endif // __FLASH_H
