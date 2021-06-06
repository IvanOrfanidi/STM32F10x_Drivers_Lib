/**
 * @brief   This is file realise SPI interface.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DWT_TIMER_H
#define __DWT_TIMER_H

/* Includes ------------------------------------------------------------------*/
/* Standart lib */
#include <stdint.h>
#include <time.h>

/* Driver MCU */
#include <CMSIS/Include/stm32f10x.h>

/* Utils */
#include <include/utils/singleton_static.h>

#ifdef __cplusplus

namespace stm32f10x_driver_lib {

class Dwttick : public SingletonStatic<Dwttick>
{
  public:
	friend class SingletonStatic<Dwttick>;

	void init(uint32_t, time_t);

	static time_t getCounter();

	void reset() const;

  private:
	Dwttick() = default;

	static time_t _prescaler;
};

} // namespace stm32f10x_driver_lib

extern "C" {
}

#endif // __cplusplus

#endif // __DWT_TIMER_H
