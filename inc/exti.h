/**
 * @brief   This file contains all the methods prototypes for External 
 *          interrupt/event controller (EXTI)
 *          firmware library.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXTI_H
#define __EXTI_H

#include <stdint.h>

/* Driver MCU */
#include <CMSIS/Include/stm32f10x.h>

/* Drivers */
#include <inc/clock.h>
#include <inc/interrupt.h>

/* Utils */
#include <inc/utils/non_copyable.h>
#include <inc/utils/non_movable.h>
#include <inc/utils/observer.h>

#ifdef __cplusplus

/* C++ lib */
#include <functional>

namespace stm32f10x_driver_lib {

class Exti
  : private NonCopyable<Exti>
  , private NonMovable<Exti>
  , public Clock
  , public Interrupt
  , public Observer
  , public InterruptObserver
{
  public:
	enum ExtiNum {
		EXTI_0 = 0,
		EXTI_1,
		EXTI_2,
		EXTI_3,
		EXTI_4,
		EXTI_5,
		EXTI_6,
		EXTI_7,
		EXTI_8,
		EXTI_9,
		EXTI_10,
		EXTI_11,
		EXTI_12,
		EXTI_13,
		EXTI_14,
		EXTI_15,

		EXTI_MAX_COUNT
	};

	// GPIO Pull Up Pull Down
	enum class GpioPuPd { NOPULL, UP, DOWN };

	enum class Trigger { RISING = 0x08, FALLING = 0x0C, RISING_FALLING = 0x10 };

	enum class Mode { INTERRUPT = 0x00, EVENT = 0x04 };

	struct Config
	{
		GpioPuPd gpioPuPd;
		Trigger trigger;
		Mode mode;
	};

	enum Priority {
		PREEMPTION_PRIORITY = 0,
		SUB_PRIORITY = 0,
	};

	Exti() = default;
	explicit Exti(GPIO_TypeDef*, uint8_t, const Config&);
	~Exti();

	void init(GPIO_TypeDef*, uint8_t, const Config&);

	// Initialization alarm handler
	void createInterrupt(uint8_t preemption = PREEMPTION_PRIORITY,
	                     uint8_t sub = SUB_PRIORITY);

	void enable() const; // Enable interrupt line

	void disable() const; // Disable interrupt line

	// Clears the EXTI's line pending flags
	void clearFlag() const;

	// Checks whether the specified EXTI line flag is set or reset
	bool isFlagStatus() const;

	// Interrupt Handler
	virtual void handler() override;

  private:
	enum PortSources {
		PORT_SOURCE_GPIOA = 0,
		PORT_SOURCE_GPIOB = 1,
		PORT_SOURCE_GPIOC = 2,
		PORT_SOURCE_GPIOD = 3,
		PORT_SOURCE_GPIOE = 4
	};

	// Init source channel interrupt
	void initSourceChannel(const GPIO_TypeDef*, uint8_t);

	void initGpioPuPd(uint8_t, GpioPuPd) const; // Configure GPIO

	void deInitGpio() const; // Deinitialization GPIO

	void initExti(const Config&) const; // Initialization EXTI Line

	void deInitExti() const; // Deinitialization EXTI Line

	GPIO_TypeDef* _port; //< Port GPIO

	uint8_t _pin; //< Pin GPIO

	size_t _channel; //< Interrupt channel

	size_t _number; //< Interrupt number

	uint32_t _line; //< Interrupt line

	PortSources _source; //< Port source

	Mode _mode; //< Mode (Event or Interrupt)
};

} // namespace stm32f10x_driver_lib

extern "C" {
}

#endif //__cplusplus

#endif // __EXTI_H
