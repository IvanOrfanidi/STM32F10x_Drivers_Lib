/**
 * @brief   This is file realise SPI interface.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H
#define __SPI_H

/* Includes ------------------------------------------------------------------*/
/* Standart lib */
#include <stdint.h>

/* Driver MCU */
#include <CMSIS/Include/stm32f10x.h>

/* Driver */
#include <include/clock.h>
#include <include/interrupt.h>

/* Utils */
#include <include/utils/non_copyable.h>
#include <include/utils/non_movable.h>
#include <include/utils/virtual_port.h>
#include <include/utils/ring_buffer.h>

#ifdef __cplusplus

namespace stm32f10x_driver_lib {

/*
 * @brief Class Spi
 */
class Spi
  : private NonCopyable<Spi>
  , private NonMovable<Spi>
  , public Clock
  , public VirtualPort
{
  public:
	// SPI Mode
	enum Mode : uint16_t { SLAVE = 0, MASTER = 0x0104 };

	// Data Direction
	enum Direction : uint16_t {
		TWO_LINE_FULL_DUPLEX = 0,
		TWO_LINE_RX_ONLY = 0x0400,
		TWO_LINE_RX = 0x8000,
		TWO_LINE_TX = 0xC000
	};

	// BaudRate Prescaler
	enum Prescaler : uint16_t {
		_2P = 0,
		_4P = 0x0008,
		_8P = 0x0010,
		_16P = 0x0018,
		_32P = 0x0020,
		_64P = 0x0028,
		_128P = 0x0030,
		_256P = 0x0038
	};

	// Data size
	enum Size : uint16_t { _8B = 0, _16B = 0x0800 };

	// Clock Polarity
	enum Polarity : uint16_t {
		LOW = 0,
		HIGH = 0x0002,
	};

	// Clock Phase
	enum Phase : uint16_t { _1E = 0, _2E = 1 };

	// MSB LSB transmission
	enum FirstBit : uint16_t { MSB = 0, LSB = 0x0080 };

	// General config
	struct Config
	{
		Mode mode;
		Direction direction;
		Prescaler prescaler;
		Size size;
		Polarity polarity;
		Phase phase;
		FirstBit firstBit;
		uint16_t crcPolynomial;
	};

	enum Default {
		PREEMPTION_PRIORITY = 0,
		SUB_PRIORITY = 0,

		RX_BUFFER_SIZE = 1,
	};

	// Get instance class
	static Spi& getInstance(SPI_TypeDef* const);

	virtual ~Spi() = default;

	// Initialization SPI
	void init(const Config&, size_t rxSize = Default::RX_BUFFER_SIZE);

	void deInit();

	// Get type virtual port
	virtual VirtualPort::Type getType() override;

	virtual size_t getLength() override;

	virtual bool isEmpty() override;

	virtual void waitingCompleteTransfer() override;

	// Transmit data
	virtual void transmit(const void*, size_t) override;

	virtual size_t receive(void*, size_t) override;

	// Clean transmiter
	virtual void cleanTransmit() override;

	// Clean receive
	virtual void cleanReceive() override;

	// Enable transmit interrupt
	virtual void enableTransmit() override;

	// Disable transmit interrupt
	virtual void disableTransmit() override;

	// Enable USART Receive interrupt
	virtual void enableReceive() override;

	// Disable USART Receive interrupt
	virtual void disableReceive() override;

	void waitingReceive() const;

	// Enable SPI
	void enable() const;

	// Disable SPI
	void disable() const;

	// Enable CRC value calculation of the transferred bytes
	void enableCalculateCrc() const;

	// Disable CRC value calculation of the transferred bytes
	void disableCalculateCrc() const;

	// Configures the data size for the selected SPI
	void setSize(Size);

	// Write data
	void write(uint16_t) const;

	// Read data
	uint16_t read() const;

	// Is Set Flag
	bool isSetStatusFlag(uint16_t) const;

  private:
	enum Mask : uint16_t {
		CR1_CLEAR = 0x3040,
	};

	Spi();

	void init(SPI_TypeDef* const);

	// Сonstructor private
	explicit Spi(SPI_TypeDef* const);

	// Init Buffers
	void initBuffers(size_t);

	// Deinit Buffers
	void deInitBuffers();

	// Initialization GPIO for SPI.
	void initGpio() const;

	// Deinitialization GPIO
	void deInitGpio() const;

	// Clear Flag
	void clearFlag(uint16_t) const;

	SPI_TypeDef* _spi; //< Work SPI

	Mode _mode; //< SPI mode

	// Receive ring buffer
	RingBuffer<uint16_t>* _receiveBuffer;

	bool _isError;

	bool _isCreated;
};

} // namespace stm32f10x_driver_lib

extern "C" {
}

#endif // __cplusplus

#endif
