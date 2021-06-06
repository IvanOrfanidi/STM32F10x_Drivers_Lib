/**
 * @brief   This is file realise USART interface.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_H
#define __UART_H

/* Includes ------------------------------------------------------------------*/
/* Standart lib */
#include <stdint.h>

/* Driver MCU */
#include <CMSIS/Include/stm32f10x.h>

/* Driver */
#include <include/interrupt.h>

/* Utils */
#include <include/utils/non_copyable.h>
#include <include/utils/non_movable.h>
#include <include/utils/virtual_port.h>
#include <include/utils/ring_buffer.h>
#include <include/utils/observer.h>

#ifdef __cplusplus

/* C++ lib*/
#include <functional>

namespace stm32f10x_driver_lib {
/*
 * @brief Class Uart
 */
class Uart
  : private NonCopyable<Uart>
  , private NonMovable<Uart>
  , public VirtualPort
  , public Interrupt
  , public InterruptObserver
{
  public:
	// USART Word Length
	enum WordLength : uint16_t { _8B = 0, _9B = USART_CR1_M };

	// USART Stop Bits
	enum StopBits : uint16_t {
		_1B = 0,
		_0_5B = USART_CR2_STOP_0,
		_2B = USART_CR2_STOP_1,
		_1_5B = USART_CR2_STOP
	};

	// USART Parity
	enum Parity : uint16_t {
		NO = 0,
		EVEN = USART_CR1_PCE,
		ODD = USART_CR1_PS | USART_CR1_PCE,
	};

	enum Mode : uint16_t {
		RX = USART_CR1_RE,
		TX = USART_CR1_TE,
		TX_RX = TX | RX
	};

	enum HardwareFlowControl : uint16_t {
		NONE = 0,
		RTS = USART_CR3_RTSE,
		CTS = USART_CR3_CTSE,
		RTS_CTS = RTS | CTS
	};

	// General config
	struct Config
	{
		WordLength wordLength;
		StopBits stopBits;
		Parity parity;
		HardwareFlowControl hardFlowControl;
		Mode mode;
		uint32_t baudRate;
		uint32_t sysClock;
	};

	enum Default : size_t {
		PREEMPTION_PRIORITY = 0,
		SUB_PRIORITY = 0,

		TX_BUFFER_SIZE = 256,
		RX_BUFFER_SIZE = 256,
	};

	// Get instance class
	static Uart* getInstance(USART_TypeDef* const);

	virtual ~Uart(); // Destructor

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

	// Interrupt Handler
	virtual void handler() override;

	// Initialization UART and interrupt this
	void init(const Config&,
	          size_t txSize = Default::TX_BUFFER_SIZE,
	          size_t rxSize = Default::RX_BUFFER_SIZE);

	void setBaudRate(uint32_t);

	// Init UART Interrupt
	void createInterrupt(uint8_t preemption = PREEMPTION_PRIORITY,
	                     uint8_t sub = SUB_PRIORITY);

	void write(uint16_t) const;

	uint16_t read() const;

	// Get overflow rx buffer
	bool isOverflow() const;

	// Get is error to interface
	bool isError() const;

	// Clean error
	void cleanError();

	// Enable USART
	void enable() const;

	// Disable USART
	void disable() const;

  private:
	enum InterruptDefinition : uint16_t {
		USART_IT_PE = 0x0028,
		USART_IT_TXE = 0x0727,
		USART_IT_TC = 0x0626,
		USART_IT_RXNE = 0x0525,
		USART_IT_IDLE = 0x0424,
		USART_IT_LBD = 0x0846,
		USART_IT_CTS = 0x096A,
		USART_IT_ERR = 0x0060,
		USART_IT_ORE = 0x0360,
		USART_IT_NE = 0x0260,
		USART_IT_FE = 0x0160
	};

	enum Mask : uint16_t {
		CR2_CLEAR = 0xCFFF,
		CR1_CLEAR = 0xE9F3,
		CR3_CLEAR = 0xFCFF
	};

	// Constructor private
	explicit Uart(USART_TypeDef* const);

	// Init Buffers
	void initBuffers(size_t, size_t);

	// Deinit Buffers
	void deInitBuffers();

	// Enable USART clock
	void enableUartClock() const;

	// Disable USART clock
	void disableUartClock() const;

	// Initialization GPIO for UART.
	void initGpio() const;

	// Deinitialization GPIO
	void deInitGpio() const;

	void initHardwareFlowControl(HardwareFlowControl) const;

	void deInitHardwareFlowControl() const;

	void initUart(const Config&);

	// Deinitialization USART
	void deInitUart() const;

	// Transmit data
	void pushBuffer(uint8_t);

	// Receive data
	bool popBuffer(uint8_t* const);

	bool isSetStatusFlag(uint16_t) const;

	bool isSetInterruptFlag(uint16_t) const;

	void clearFlag(uint16_t) const;

	// Get APB clock
	uint32_t getApbClock(uint32_t) const;

	USART_TypeDef* _uart; //< Work USART

	// Transmit ring buffer
	RingBuffer<uint8_t>* _transmitBuffer;

	// Receive ring buffer
	RingBuffer<uint8_t>* _receiveBuffer;

	bool _isError;

	uint32_t _sysClock; //< System clock

	enum UartInterface : size_t {
		_1 = 0,
		_2 = 1,
		_3 = 2,

		MAX_COUNT_UART
	};

	// Main array pointers of classes Uarts
	static Uart* _uarts[UartInterface::MAX_COUNT_UART];
};

} // namespace stm32f10x_driver_lib

extern "C" {
}

#endif // __cplusplus

#endif
