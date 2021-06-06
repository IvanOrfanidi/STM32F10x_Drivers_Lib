/**
 * @brief   This is file realize lock interrupt.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOCK_INTERRUPT_H
#define __LOCK_INTERRUPT_H

/* Includes ------------------------------------------------------------------*/
/* Drivers */
#include <include/interrupt.h>

/* Utils */
#include <include/utils/non_copyable.h>
#include <include/utils/non_movable.h>

#ifdef __cplusplus

// Class lock interrupt
class LockInterrupt
  : private NonCopyable<LockInterrupt>
  , private NonMovable<LockInterrupt>
{
  public:
	LockInterrupt()
	{
		_isEnabled = stm32f10x_driver_lib::Interrupt::isEnabledGlobally();
		stm32f10x_driver_lib::Interrupt::disableGlobally();
	}

	~LockInterrupt()
	{
		// Enable interrupts back
		if(_isEnabled) {
			stm32f10x_driver_lib::Interrupt::enableGlobally();
		}
	}

  private:
	bool _isEnabled;
};

extern "C" {
}

#endif // __cplusplus

#endif // __LOCK_INTERRUPT_H
