/**
 * @brief   This is file realize Singleton pattern.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SINGLETON_STATIC_H
#define __SINGLETON_STATIC_H

/* Includes ------------------------------------------------------------------*/
/* Utils */
#include <include/utils/non_copyable.h>
#include <include/utils/non_movable.h>

#ifdef __cplusplus

template<typename T>
class SingletonStatic
  : public NonCopyable<T>
  , public NonMovable<T>
{
  public:
	// Get instance class
	static T& getInstance()
	{
		// Create class instance
		static T instance;
		return instance;
	}

  protected:
	SingletonStatic() = default;
	~SingletonStatic() = default;
};

extern "C" {
}

#endif // __cplusplus

#endif // __SINGLETON_STATIC_H
