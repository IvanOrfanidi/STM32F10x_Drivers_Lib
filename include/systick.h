/**
 * @brief   This is file realise System Timer.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSTICK_H
#define __SYSTICK_H

/* Includes ------------------------------------------------------------------*/
/* Standart lib */
#include <stdint.h>
#include <time.h>

/* Driver MCU */
#include <CMSIS/Include/stm32f10x.h>

/* Utils */
#include <include/utils/singleton_static.h>

#ifdef __cplusplus

/* C++ lib */
#include <functional>

namespace stm32f10x_driver_lib {

class Systick : public SingletonStatic<Systick>
{
  public:
	friend class SingletonStatic<Systick>;

	// Configures System Tick
	void init(uint32_t, time_t);

	// Delay
	void delay(time_t);

	// Enable SysTick
	void enable() const;

	// Disable SysTick
	void disable() const;

	// Enable interrupt
	void enableInterrupt() const;

	// Disable interrupt
	void disableInterrupt() const;

	// SysTick Clear counter value
	void clear() const;

	// Clear counter value
	void clearCounter();

	// SysTick Get Calibration Value
	uint32_t getCalibration() const;

	// Get the counter value
	uint32_t get() const;

	// Get the current SysTick counter value
	static time_t getCounter();

	// Increment counter
	static void incrementCounter();

  private:
	enum { STK_CSR_ENABLE = (1 << 0), STK_CSR_TICKINT = (1 << 1) };

	Systick() = default;
	~Systick();

	static time_t _counter; //< Counter
};

} // namespace stm32f10x_driver_lib

extern "C" {
/* SYSTICK INTERRUPT */
void SysTick_Handler(void);
}

#endif // __cplusplus

#endif // __SYSTICK_H
