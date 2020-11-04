/**
 * @brief   This is file realize SPI interface.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_INTERFACE_H
#define __SPI_INTERFACE_H

/* Includes ------------------------------------------------------------------*/
/* Standard lib */
#include <stdint.h>
#include <time.h>

/* Drivers */
#include <inc/exti.h>
#include <inc/gpio.h>
#include <inc/spi.h>

/* Utils */
#include <inc/utils/non_copyable.h>
#include <inc/utils/non_movable.h>
#include <inc/utils/observer.h>
#include <inc/utils/soft_timer.h>
#include <inc/utils/pin_interface.h>

#ifdef __cplusplus

namespace stm32f10x_driver_lib {

class SpiInterface
  : private NonCopyable<SpiInterface>
  , private NonMovable<SpiInterface>
  , public Exti
  , public Listener
{
  public:
	struct Config
	{
		VirtualPort* virtualPort;

		struct Interrupt
		{
			GPIO_TypeDef* port = nullptr;
			uint8_t pin;
		};
		Interrupt interrupt;

		PinInterface::Config reset = {nullptr, 0, 0, 0, nullptr};
		PinInterface::Config cs;

		std::function<uint32_t()> softTimerMs;
		std::function<uint32_t()> softTimerUs;
	};

	explicit SpiInterface(const Config& config)
	  : Exti(config.interrupt.port,
	         config.interrupt.pin,
	         {Exti::GpioPuPd::NOPULL,
	          Exti::Trigger::FALLING,
	          Exti::Mode::INTERRUPT})
	  , _port(config.virtualPort)
	  , _handlerInterrupt(nullptr)
	  , _reset(config.reset,
	           {Gpio::Mode::OUTPUT_PUSH_PULL, Gpio::Speed::_50mhz})
	  , _cs(config.cs, {Gpio::Mode::OUTPUT_PUSH_PULL, Gpio::Speed::_50mhz})
	  , _softTimerMs(config.softTimerMs)
	  , _softTimerUs(config.softTimerUs)
	  , _isIrq(false)
	{
		setSelect(false);
		resetPulse();

		// Create interrupt
		if(config.interrupt.port != nullptr) {
			Exti::disable();
			Exti::createInterrupt();
			Exti::attach(this);
		}
	}

	~SpiInterface() = default;

	virtual void update()
	{
		_isIrq = true;
		if(nullptr != _handlerInterrupt) {
			_handlerInterrupt->update();
		}
	}

	void interruptEnable() const
	{
		Exti::enable();
	}

	void interruptDisable() const
	{
		Exti::disable();
	}

	void attach(Listener* subject)
	{
		_handlerInterrupt = subject;
	}

	void resetPulse()
	{
		_reset.reset();
		_reset.set();
	}

	void setSelect(bool state)
	{
		if(state) {
			_cs.reset();
		} else {
			_cs.set();
		}
	}

	void cleanReceive()
	{
		_port->cleanReceive();
	}

	uint8_t get() const
	{
		uint16_t data = 0;
		_port->transmit(&data, 1);
		while(_port->isEmpty()) {
		}
		_port->receive(&data, 1);

		_port->cleanReceive();
		return data;
	}

	uint8_t send(uint8_t byte) const
	{
		uint16_t data = byte;
		_port->transmit(&data, 1);

		while(_port->isEmpty()) {
		}
		_port->receive(&data, 1);

		return data;
	}

	bool isIrq() const
	{
		return _isIrq;
	}

	void resetIrq()
	{
		_isIrq = false;
	}

	/**
     * @brief  Get type virtual port
     * @retval type UART
     */
	VirtualPort::Type getType()
	{
		return _port->getType();
	}

	void delayMs(uint32_t timeout)
	{
		_softTimerMs.start(timeout);
		while(!_softTimerMs.isMatch()) {
		}
	}

	void delayUs(uint32_t timeout)
	{
		_softTimerUs.start(timeout);
		while(!_softTimerUs.isMatch()) {
		}
	}

  private:
	SpiInterface() = delete;

	VirtualPort* _port;
	Listener* _handlerInterrupt;

	PinInterface _reset;
	PinInterface _cs;

	uint32_t _csEnableTimeoutUs;
	uint32_t _csDisableTimeoutUs;

	SoftTimer _softTimerMs;
	SoftTimer _softTimerUs;

	bool _isIrq;
};

} // namespace stm32f10x_driver_lib

extern "C" {
}

#endif // __cplusplus

#endif // __SPI_INTERFACE_H
