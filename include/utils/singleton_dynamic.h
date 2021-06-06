/**
 * @brief   This is file realize Singleton pattern.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SINGLETON_DYNAMIC_H
#define __SINGLETON_DYNAMIC_H

/* Includes ------------------------------------------------------------------*/
/* Drivers */
#include <include/interrupt.h>

/* Utils */
#include <include/utils/non_copyable.h>
#include <include/utils/non_movable.h>
#include <include/utils/lock_interrupt.h>

#ifdef __cplusplus

template<class T>
class SingletonDynamic
  : private NonCopyable<T>
  , private NonMovable<T>
{
  public:
	// Get instance class
	static T* getInstance()
	{
		if(nullptr == _instance) {
			LockInterrupt lockInterrupt;
			// Create class instance
			if(nullptr == _instance) {
				_instance = new T();
			}
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
	static T* _instance;
};

template<class T> T* SingletonDynamic<T>::_instance = nullptr;

extern "C" {
}

#endif // __cplusplus

#endif // __SINGLETON_DYNAMIC_H
