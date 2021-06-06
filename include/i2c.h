/**
 * @brief   This is file realise usart.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H
#define __I2C_H

/* Includes ------------------------------------------------------------------*/
/* Standart lib */
#include <stdint.h>

/* Driver MCU */
#include <CMSIS/Include/stm32f10x.h>

/* Driver Interrupt */
#include <include/clock.h>
#include <include/gpio.h>

/* Utils */
#include <include/utils/non_copyable.h>
#include <include/utils/non_movable.h>
#include <include/utils/virtual_port.h>

#ifdef __cplusplus

namespace stm32f10x_driver_lib {

/*
 * @brief Class Uart
 */
class I2c
  : private NonCopyable<I2c>
  , private NonMovable<I2c>
  , public Clock
  , public VirtualPort
{
  public:
	enum class DutyCycle : uint16_t { _2tt = 0xBFFF, _16_9tt = 0x4000 };

	enum class AcknowledgedAddress : uint16_t { _7B = 0x4000, _10B = 0xC000 };

	// General config
	struct Config
	{
		uint16_t address;
		DutyCycle dutyCycle;
		bool isAck;
		AcknowledgedAddress acknowledgedAddress;
		uint32_t frequency;
		uint32_t sysClock;
	};

	static I2c& getInstance(I2C_TypeDef* const);

	~I2c();

	// Initialization UART and interrupt this
	void init(const Config&);

	// Get type virtual port
	virtual VirtualPort::Type getType() override;

	virtual size_t getLength() override;

	virtual bool isEmpty() override;

	virtual void waitingCompleteTransfer() override;

	virtual void transmit(const void*, size_t) override;

	virtual size_t receive(void*, size_t) override;

	virtual void cleanTransmit() override;

	virtual void cleanReceive() override;

	// Enable transmit interrupt
	virtual void enableTransmit() override;

	// Disable transmit interrupt
	virtual void disableTransmit() override;

	// Enable USART Receive interrupt
	virtual void enableReceive() override;

	// Disable USART Receive interrupt
	virtual void disableReceive() override;

	void setFrequency(uint32_t);

	void setAcknowledgedAddress(AcknowledgedAddress) const;

	void setAddress(uint16_t) const;

	bool isSetStart() const;

	bool isSetAddress(uint8_t) const;

	void setStart() const;

	void resetStart() const;

	void setStop() const;

	void resetStop() const;

	void enableAck() const;

	void disableAck() const;

	// Write data
	void write(uint8_t) const;

	// Read data
	uint8_t read() const;

  private:
	static constexpr int CHECK_EVENT_TIMEOUT = 1000;

	I2c();

	explicit I2c(I2C_TypeDef* const);

	void gpioInitialization();

	// Enable I2C
	void enable() const;

	// Disable I2C
	void disable() const;

	// Get APB clock
	uint32_t getApbClock(uint32_t) const;

	bool _isError;

	I2C_TypeDef* _i2c; //< Work I2C

	uint32_t _sysClock; //< System clock

	DutyCycle _dutyCycle;

	bool _isCreated;

	Gpio _scl;
	Gpio _sda;
};

} // namespace stm32f10x_driver_lib

extern "C" {
}

#endif //__cplusplus

#endif