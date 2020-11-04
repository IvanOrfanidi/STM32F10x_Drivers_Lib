/**
 * @brief   This is file realise GPIO interface.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PIN_INTERFACE_H
#define __PIN_INTERFACE_H

/* Includes ------------------------------------------------------------------*/
/* Standard lib */
#include <stdint.h>

/* Drivers periphl */
#include <inc/gpio.h>

/* Utils */
#include <inc/utils/non_copyable.h>
#include <inc/utils/non_movable.h>

#ifdef __cplusplus

/* C++ lib */
#include <functional>

namespace stm32f10x_driver_lib {

class PinInterface
  : private NonCopyable<PinInterface>
  , private NonMovable<PinInterface>
{
  public:
	struct Config
	{
		GPIO_TypeDef* port;
		uint8_t pin;
		uint32_t enableTimeout;
		uint32_t disableTimeout;
		std::function<uint32_t()> softTimer;
	};

	PinInterface() = default;

	/**
     * @param [in] config - configure struct
     * @param [in] gpio - configure gpio 
     */
	PinInterface(const Config& config, const Gpio::Config& gpio)
	  : _gpio(config.port, config.pin, gpio)
	  , _enableTimeout(config.enableTimeout)
	  , _disableTimeout(config.disableTimeout)
	  , _softTimer(config.softTimer)
	  , _isUse(false)
	{
		if(config.port != nullptr && config.softTimer != nullptr) {
			_isUse = true;
		}
	}

	bool get()
	{
		if(_gpio.get()) {
			_softTimer.start(_enableTimeout);
		} else {
			_softTimer.start(_disableTimeout);
		}

		while(!_softTimer.isMatch()) {
		}

		return _gpio.get();
	}

	void reset(uint32_t timeout = 0)
	{
		if(_isUse) {
			_gpio.reset();

			if(0 == timeout) {
				_softTimer.start(_enableTimeout);
			} else {
				_softTimer.start(timeout);
			}

			while(!_softTimer.isMatch()) {
			}
		}
	}

	void set(uint32_t timeout = 0)
	{
		if(_isUse) {
			_gpio.set();

			if(0 == timeout) {
				_softTimer.start(_disableTimeout);
			} else {
				_softTimer.start(timeout);
			}

			while(!_softTimer.isMatch()) {
			}
		}
	}

  private:
	Gpio _gpio;
	uint32_t _enableTimeout;
	uint32_t _disableTimeout;
	SoftTimer _softTimer;
	bool _isUse;
};

} // namespace stm32f10x_driver_lib

extern "C" {
}

#endif // __cplusplus

#endif // __PIN_INTERFACE_H
