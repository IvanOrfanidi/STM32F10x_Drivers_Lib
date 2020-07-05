/**
 * @brief   This is file realize Singleton pattern.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SINGLETON_DYNAMIC_H
#define __SINGLETON_DYNAMIC_H

/* Includes ------------------------------------------------------------------*/
/* Drivers */
#include <inc/interrupt.h>

/* Utils */
#include <inc/utils/non_copyable.h>
#include <inc/utils/non_movable.h>

#ifdef __cplusplus

template<typename T>
class SingletonDynamic
  : private NonCopyable<T>
  , private NonMovable<T>
{
  public:
	// Get instance class
	static T* getInstance()
	{
		// Check interrupt status enable
		LockInterrupt lockInterrupt;
		// Create class instance
		if(nullptr == _instance) {
			_instance = new T();
		}

		return _instance;
	}

	static void destroyInstance()
	{
		delete _instance;
		_instance = nullptr;
	}

  protected:
	SingletonDynamic() = default;
	~SingletonDynamic()
	{
		destroyInstance();
	}

  private:
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

	static T* _instance;
};

template<typename T> T* SingletonDynamic<T>::_instance = nullptr;

extern "C" {
}

#endif //__cplusplus

#endif // __SINGLETON_DYNAMIC_H
