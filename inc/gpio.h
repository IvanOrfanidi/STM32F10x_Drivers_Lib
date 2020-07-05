/**
 * @brief   This file contains all the methods prototypes for External 
 *          interrupt/event controller (EXTI)
 *          firmware library.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H
#define __GPIO_H

/* Includes ------------------------------------------------------------------*/
/* Standart lib */
#include <stdint.h>

/* Driver MCU */
#include <CMSIS/Include/stm32f10x.h>

/* Driver */
#include <inc/clock.h>

/* Utils */
#include <inc/utils/non_copyable.h>
#include <inc/utils/non_movable.h>

#ifdef __cplusplus

namespace stm32f10x_driver_lib {

class Gpio
  : private NonCopyable<Gpio>
  , private NonMovable<Gpio>
  , public Clock
{
  public:
	// Specifies the GPIO pins to be configured
	enum class Mode {
		INPUT_FLOATING = 0x04,
		INPUT_PULL_DOWN = 0x28,
		INPUT_PULL_UP = 0x48,
		OUTPUT_OPEN_DRAIN = 0x14,
		OUTPUT_PUSH_PULL = 0x10
	};

	// Specifies the speed for the selected pins
	enum class Speed { _10mhz = 1, _2mhz = 2, _50mhz = 3 };

	// Config data
	struct Config
	{
		Mode mode;
		Speed speed = Speed::_10mhz;
	};

	// Constructor
	explicit Gpio(const GPIO_TypeDef*, uint8_t);
	explicit Gpio(const GPIO_TypeDef*, uint8_t, const Config&);

	// Destruction
	~Gpio();

	// Init pin
	void init(GPIO_TypeDef*, uint8_t, const Config&);

	// Init pin
	void init(const Config&) const;

	// Init port clock
	void initGpioClock(GPIO_TypeDef*) const;

	// Set pin
	void set() const;

	// Reset pin
	void reset() const;

	// Read pin
	bool get() const;

  private:
	GPIO_TypeDef* _port; //< Port GPIO

	uint16_t _pin; //< Pin GPIO
};

} // namespace stm32f10x_driver_lib

extern "C" {
}

#endif // __cplusplus

#endif // __GPIO_H
