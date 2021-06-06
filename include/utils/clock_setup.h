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
#include <include/clock.h>
#include <include/flash.h>

#ifdef __cplusplus

namespace stm32f10x_driver_lib {

class ClockSetup
  : public SingletonStatic<ClockSetup>
  , public stm32f10x_driver_lib::Clock
  , public stm32f10x_driver_lib::Flash
{
  public:
	friend class SingletonStatic<ClockSetup>;

	void setInHse8MHzOut72MHz() const
	{
		Clock::enableHighSpeedInternalClock();
		while(!Clock::isReadyClockSource(ClockSource::HSI)) {
		}
		Clock::setSystemSource(ClockSource::HSI);

		Clock::enableHighSpeedExternalClock();
		while(!Clock::isReadyClockSource(ClockSource::HSE)) {
		}
		Clock::setSystemSource(ClockSource::HSE);

		// Enable Prefetch Buffer
		Flash::enablePrefetchBuffer();
		/* Sysclk runs with 72MHz -> 2 waitstates.
		 * 0WS from 0-24MHz
		 * 1WS from 24-48MHz
		 * 2WS from 48-72MHz */
		Flash::setLatency(FLASH_ACR_LATENCY_2);

		// Set. 72MHz Max. 72MHz
		Clock::setAhbPrescaleFactor(RCC_CFGR_HPRE_DIV1);
		// Set. 72MHz Max. 72MHz
		Clock::setApb2PrescaleFactor(RCC_CFGR_PPRE2_DIV1);
		// Set. 36MHz Max. 36MHz
		Clock::setApb1PrescaleFactor(RCC_CFGR_PPRE1_DIV2);

		// PLL configuration: PLLCLK = HSE * 9 = 72 MHz
		Clock::setPllMultiplicationFactor(RCC_CFGR_PLLMULL9);

		// Select HSE as PLL source
		Clock::setPllSource(RCC_CFGR_PLLSRC_HSE);

		// Enable PLL
		Clock::enablePhaseLockedLoopsClock();
		while(!Clock::isReadyClockSource(ClockSource::PLL)) {
		}

		// Select PLL as system clock source
		Clock::setSystemSource(ClockSource::PLL);
	}

  private:
	ClockSetup() = default;
	~ClockSetup() = default;
};

} // namespace stm32f10x_driver_lib

extern "C" {
}

#endif // __cplusplus

#endif
