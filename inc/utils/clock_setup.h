/**
 * @brief   This is file realise SPI interface.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CLOCK_SETUP_H
#define __CLOCK_SETUP_H

/* Includes ------------------------------------------------------------------*/
/* Standard lib */
#include <stdint.h>

/* Drivers */
#include <inc/clock.h>
#include <inc/flash.h>

#ifdef __cplusplus

namespace stm32f10x_driver_lib {

class ClockSetup
  : public stm32f10x_driver_lib::Clock
  , public stm32f10x_driver_lib::Flash
{
  public:
	void inHse8MHzOut72MHz() const
	{
		Clock::enableHighSpeedInternalClock();
		while(!Clock::isReadyClockSource(ClockSource::HSI)) {
		}
		Clock::setSystemSource(ClockSource::HSI);

		enableHighSpeedExternalClock();
		while(!Clock::isReadyClockSource(ClockSource::HSE)) {
		}
		Clock::setSystemSource(ClockSource::HSI);

		// Set. 72MHz Max. 72MHz
		Clock::setAhbPrescaleFactor(RCC_CFGR_HPRE_DIV1);
		// Set. 36MHz Max. 36MHz
		Clock::setApb1PrescaleFactor(RCC_CFGR_PPRE1_DIV2);
		// Set. 64MHz Max. 72MHz
		Clock::setApb2PrescaleFactor(RCC_CFGR_PPRE2_DIV1);

		/* Sysclk runs with 72MHz -> 2 waitstates.
    	 * 0WS from 0-24MHz
    	 * 1WS from 24-48MHz
    	 * 2WS from 48-72MHz */
		Flash::setLatency(FLASH_ACR_LATENCY_2);
	}
};

} // namespace stm32f10x_driver_lib

extern "C" {
}

#endif //__cplusplus

#endif
