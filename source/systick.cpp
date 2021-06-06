/**
 * @brief   This is file realise System Timer.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <include/systick.h>

namespace stm32f10x_driver_lib {

time_t Systick::_counter = 0;

/**
 * @bref Destructor
 */
Systick::~Systick()
{
	// Disable counter and interrupt
	disable();
	disableInterrupt();
}

/**
 * @bref Configures System Tick
 * @param [in] sysClock - system clock core
 * @param [in] ticks  Number of ticks between two interrupts
 *                      / 1000     |   1ms  /
 *                      / 100000   |   10us /
 *                      / 1000000  |   1us  /
 */
void Systick::init(uint32_t sysClock, time_t ticks)
{
	disable();
	disableInterrupt();
	while(SysTick_Config(sysClock / ticks)) {
		// Wait running System Tick
	}
}

/**
 * @bref Increment counter
 */
void Systick::incrementCounter()
{
	++_counter;
}

/**
 * @bref Delay
 * @param [in] value - value delay
 */
void Systick::delay(time_t value)
{
	const time_t count = getCounter();
	while((getCounter() - count) < value) {
		// Wait
	}
}

/**
 * @brief Enable SysTick
 */
void Systick::enable() const
{
	SysTick->CTRL |= STK_CSR_ENABLE;
}

/**
 * @brief Disable SysTick
 */
void Systick::disable() const
{
	SysTick->CTRL &= ~STK_CSR_ENABLE;
}

/**
 * @brief Enable interrupt
 */
void Systick::enableInterrupt() const
{
	SysTick->CTRL |= STK_CSR_TICKINT;
}

/**
 * @brief Disable interrupt
 */
void Systick::disableInterrupt() const
{
	SysTick->CTRL &= ~STK_CSR_TICKINT;
}

/**
 * @brief SysTick Clear counter value
 */
void Systick::clear() const
{
	SysTick->VAL = 0;
}

/**
 * @brief SysTick Clear counter value
 */
void Systick::clearCounter()
{
	_counter = 0;
}

/**
 * @brief SysTick Get Calibration Value
 * @retval current calibration value
 */
uint32_t Systick::getCalibration() const
{
	return (SysTick->CALIB & 0x00FFFFFF);
}

/**
 * @brief Get the current SysTick counter value
 * @retval 24 bit current value as uint32_t
 */
uint32_t Systick::get() const
{
	return (SysTick->VAL & 0x00FFFFFF);
}

/**
 * @brief Get the current SysTick counter value
 * @retval value counter as time_t
 */
time_t Systick::getCounter()
{
	return _counter;
}

} // namespace stm32f10x_driver_lib

/**
 * @brief Interrupt
 */
void SysTick_Handler(void)
{
	stm32f10x_driver_lib::Systick::incrementCounter();
}
