/**
 * @brief   This is file realise Data Watchpoint and Trace unit timer.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <include/dwt_timer.h>

namespace stm32f10x_driver_lib {

time_t Dwttick::_prescaler = 1;

void Dwttick::init(uint32_t sysClock, time_t ticks)
{
	_prescaler = sysClock / ticks;
	if(!(DWT->CTRL & 0x00000001)) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		reset();
		DWT->CTRL |= 0x00000001;
	}
}

time_t Dwttick::getCounter()
{
	const time_t cnt = DWT->CYCCNT / _prescaler;
	return cnt;
}

void Dwttick::reset() const
{
	DWT->CYCCNT = 0;
}

} // namespace stm32f10x_driver_lib