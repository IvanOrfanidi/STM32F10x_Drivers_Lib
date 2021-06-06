/**
 * @brief   This is file realise Observer.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OBSERVER_H
#define __OBSERVER_H

/* Includes ------------------------------------------------------------------*/
/* Standart lib */
#include <stdint.h>

/* Utils */
#include <include/utils/lock_interrupt.h>

#ifdef __cplusplus

/* C++ lib*/
#include <functional>

/**
 * @brief Listener Observer
 */
class Listener
{
  public:
	virtual ~Listener() = default;
	virtual void update() = 0;
};

/**
 * @brief Observer
 */
class Observer
{
  public:
	Observer()
	  : _index(0)
	{
		for(auto& listener : _listeners) {
			listener = nullptr;
		}
	}

	virtual ~Observer() = default;

	/**
	 * @brief Notify listeners
	 */
	void notify()
	{
		for(size_t i = 0; i < _index; ++i) {
			_listeners[i]->update();
		}
	}

	/**
	 * @brief Adding listener in observer
	 * @param [in] listener - callback listener
	 */
	bool attach(Listener* listener)
	{
		if(_index < MAXIMUM_QUANTITY_OF_LISTENERS) {
			// Check interrupt status enable
			LockInterrupt lockInterrupt;

			// Listener shouldn't be already subscribed
			for(size_t i = 0; i < _index; ++i) {
				if(_listeners[i] == listener) {
					return false;
				}
			}
			_listeners[_index] = listener;
			++_index;
			return true;
		}
		return false;
	}

	/**
	 * @brief Detach listener in observer
	 * @param [in] listener - callback listener
	 */
	bool detach(Listener* listener)
	{
		if(_index > 0) {
			// Check interrupt status enable
			LockInterrupt lockInterrupt;

			for(size_t i = 0; i < _index; ++i) {
				if(_listeners[i] == listener) {
					for(size_t n = i; n < _index; ++n) {
						const size_t next = n + 1;
						if(next >= MAXIMUM_QUANTITY_OF_LISTENERS) {
							_listeners[n] = nullptr;
						} else {
							_listeners[n] = _listeners[next];
						}
					}
					--_index;
					return true;
				}
			}
		}
		return false;
	}

	bool isListeners() const noexcept
	{
		return _index != 0;
	}

  private:
	static constexpr size_t MAXIMUM_QUANTITY_OF_LISTENERS = 10;
	class Listener* _listeners[MAXIMUM_QUANTITY_OF_LISTENERS];
	size_t _index;
};

extern "C" {
}

#endif // __cplusplus

#endif // __OBSERVER_H
