/**
 * @brief   This is file realise interface USART.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <include/uart.h>

namespace stm32f10x_driver_lib {

/**
 * Main array pointers of classes Uarts
 * MAX_COUNT_UART max quantity Uarts
 */
Uart* Uart::_uarts[UartInterface::MAX_COUNT_UART];

/**
 * @brief Get instance class
 * @param [in] uart - number object USART
 * retval instance class
*/
Uart* Uart::getInstance(USART_TypeDef* const uart)
{
	// Check interrupt status enable
	const auto isEnabledInterrupts = Interrupt::isEnabledGlobally();

	// Disable interrupts for multithreading
	Interrupt::disableGlobally();

	size_t interface; // USART number
	if(uart == USART1) {
		interface = UartInterface::_1;
	} else if(uart == USART2) {
		interface = UartInterface::_2;
	} else if(uart == USART3) {
		interface = UartInterface::_3;
	} else {
		return nullptr;
	}

	if(nullptr == _uarts[interface]) {
		_uarts[interface] = ::new Uart(uart);
		_uarts[interface]->_uart = uart;
	}

	// Enable interrupts back
	if(isEnabledInterrupts) {
		Interrupt::enableGlobally();
	}

	return _uarts[interface];
}

/**
 * @brief Private constructor class Uart
 * @param [in] uart - number object USART
 */
Uart::Uart(USART_TypeDef* const uart)
  : _uart(uart)
  , _transmitBuffer(nullptr)
  , _receiveBuffer(nullptr)
  , _isError(false)
  , _sysClock(0)
{
	// Init GPIO USART
	initGpio();

	// Enable USARTs clock
	enableUartClock();
}

/**
 * @brief Destructor class Uart
 */
Uart::~Uart()
{
	// Waiting for complete the transfer of
	waitingCompleteTransfer();

	// Deinit interrupt
	disableTransmit();
	disableReceive();

	deInitUart();

	disableUartClock();

	deInitHardwareFlowControl();

	deInitGpio();

	deInitBuffers();

	if(_uart == USART1) {
		_uarts[UartInterface::_1] = nullptr;
	} else if(_uart == USART2) {
		_uarts[UartInterface::_2] = nullptr;
	} else if(_uart == USART3) {
		_uarts[UartInterface::_3] = nullptr;
	} else {
	}
}

/**
 * @brief  Initialization UART and interrupt this
 * @param [in] config - config USART
 */
void Uart::init(const Config& config, size_t txSize, size_t rxSize)
{
	initBuffers(txSize, rxSize);

	_sysClock = config.sysClock;

	initUart(config);

	enable();
}

/**
 * @brief  Initialization Nested Vectored Interrupt Controller (NVIC)
 * @param [in] preemption - preemption priority
 * @param [in] sub- sub priority
 */
void Uart::createInterrupt(uint8_t preemption, uint8_t sub)
{
	size_t irq;
	if(_uart == USART1) {
		irq = USART1_IRQn;
	} else if(_uart == USART2) {
		irq = USART2_IRQn;
	} else if(_uart == USART3) {
		irq = USART3_IRQn;
	} else {
		return;
	}
	create(irq, this, preemption, sub);

	enableReceive();
}

/**
 * @brief  Initialization Tx buffer and Rx buffer
 * @param [in] txSize - Tx size
 * @param [in] rxSize - Rx size
 */
void Uart::initBuffers(size_t txSize, size_t rxSize)
{
	if(nullptr == _transmitBuffer) {
		_transmitBuffer = ::new RingBuffer<uint8_t>(txSize);
	}
	if(nullptr == _receiveBuffer) {
		_receiveBuffer = ::new RingBuffer<uint8_t>(rxSize);
	}
}

/**
 * @brief  Deinitialization Tx buffer and Rx buffer
 */
void Uart::deInitBuffers()
{
	::delete _transmitBuffer;
	::delete _receiveBuffer;
}

/**
 * @brief  Initialization UART.
 * @param [in] config - config USART
 */
void Uart::initUart(const Config& config)
{
	// USART CR2 Configuration
	const uint16_t cr2 = _uart->CR2 & Mask::CR2_CLEAR;
	_uart->CR2 = (cr2 | config.stopBits);

	// USART CR1 Configuration
	const uint16_t cr1 = _uart->CR1 & Mask::CR1_CLEAR;
	_uart->CR1 = cr1 | config.wordLength | config.parity | config.mode;

	// USART CR3 Configuration
	const uint16_t cr3 = _uart->CR3 & CR3_CLEAR;
	_uart->CR3 = cr3 | config.hardFlowControl;

	setBaudRate(config.baudRate);
}

/**
 * @brief  Set Baud Rate.
 * @param [in] baudRate - baud rate
 */
void Uart::setBaudRate(uint32_t baudRate)
{
	const uint32_t clock = getApbClock(_sysClock);

	// Integer part computing in case Oversampling mode is 16 Samples
	const uint32_t integerdivider = ((25 * clock) / (4 * baudRate));

	uint32_t brr = (integerdivider / 100) << 4;
	const uint32_t fractionaldivider = integerdivider - (100 * (brr >> 4));
	brr |= ((((fractionaldivider * 16) + 50) / 100)) & 0x0F;
	_uart->BRR = static_cast<uint16_t>(brr);
}

/**
 * @brief  Get APB1/APB2 clock frequency for USART.
 * @param [in] sysClock - system clock in Hz
 * @retval clock frequency in Hz
 */
uint32_t Uart::getApbClock(uint32_t sysClock) const
{
	// Get HCLK prescaler
	static constexpr uint32_t CFGR_HPRE_SET_MASK = 0x000000F0;
	uint32_t temp = RCC->CFGR & CFGR_HPRE_SET_MASK;
	temp >>= 4;

	// Get HCLK clock frequency
	static constexpr uint8_t APB_AHB_PRESC_TABLE[16] = {
	    0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
	uint32_t presc = APB_AHB_PRESC_TABLE[temp];
	const uint32_t hclk = sysClock >> presc;

	if(_uart == USART1) {
		// Get APB2 prescaler
		static constexpr uint32_t CFGR_PPRE2_SET_MASK = 0x00003800;
		temp = RCC->CFGR & CFGR_PPRE2_SET_MASK;
		temp >>= 11;
		presc = APB_AHB_PRESC_TABLE[temp];
	} else {
		// Get APB1 prescaler
		static constexpr uint32_t CFGR_PPRE1_SET_MASK = 0x00000700;
		temp = RCC->CFGR & CFGR_PPRE1_SET_MASK;
		temp >>= 8;
		presc = APB_AHB_PRESC_TABLE[temp];
	}

	// Get APB1/APB2 clock frequency
	const uint32_t clock = hclk >> presc;
	return clock;
}

/**
 * @brief Enable USART
 */
void Uart::enable() const
{
	_uart->CR1 |= USART_CR1_UE;
}

/**
 * @brief Disable USART
 */
void Uart::disable() const
{
	_uart->CR1 &= ~USART_CR1_UE;
}

/**
 * @brief Deinitialization USART
 */
void Uart::deInitUart() const
{
	if(_uart == USART1) {
		RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
	} else if(_uart == USART2) {
		RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
	} else if(_uart == USART3) {
		RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;
	} else {
	}
}

/**
 * @brief  Initialization GPIO for UART.
 */
void Uart::initGpio() const
{
	if(_uart == USART1) { // USART1
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

		// Tx pin AF Push-Pull out and speed 50MHz
		GPIOA->CRH &= ~GPIO_CRH_CNF9;
		GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9);

		// Rx pin Input Pull-up
		GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
		GPIOA->CRH |= GPIO_CRH_CNF10_1;
		GPIOA->BSRR |= GPIO_BSRR_BS10;
	} else if(_uart == USART2) { // USART2
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

		// Tx pin AF Push-Pull out and speed 50MHz
		GPIOA->CRL &= ~GPIO_CRL_CNF2;
		GPIOA->CRL |= (GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2);

		// Rx pin Input Pull-up
		GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
		GPIOA->CRL |= GPIO_CRL_CNF3_1;
		GPIOA->BSRR |= GPIO_BSRR_BS3;
	} else if(_uart == USART3) { // USART3
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

		// Tx pin AF Push-Pull out and speed 50MHz
		GPIOB->CRH &= ~GPIO_CRH_CNF10;
		GPIOB->CRH |= (GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10);

		// Rx pin Input Pull-up
		GPIOB->CRH &= ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
		GPIOB->CRH |= GPIO_CRH_CNF11_1;
		GPIOB->BSRR |= GPIO_BSRR_BS11;
	} else {
	}
}

/**
 * @brief  Initialization Hard Hardware Flow Control for UART.
 * @param [in] uart - number object USART
 */
void Uart::initHardwareFlowControl(HardwareFlowControl hardFlowControl) const
{
	if(_uart == USART1) { // USART1
		if(hardFlowControl == RTS || hardFlowControl == RTS_CTS) {
			// RTS pin AF Push-Pull out and speed 50MHz
			GPIOA->CRH &= ~GPIO_CRH_CNF12;
			GPIOA->CRH |= (GPIO_CRH_CNF12_1 | GPIO_CRH_MODE12);
		}

		if(hardFlowControl == CTS || hardFlowControl == RTS_CTS) {
			// Rx pin Input Pull-up
			GPIOA->CRH &= ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
			GPIOA->CRH |= GPIO_CRH_CNF11_1;
			GPIOA->BSRR |= GPIO_ODR_ODR11;
		}
	} else if(_uart == USART2) { // USART2
		if(hardFlowControl == RTS || hardFlowControl == RTS_CTS) {
			// RTS pin AF Push-Pull out and speed 50MHz
			GPIOA->CRL &= ~GPIO_CRL_CNF1;
			GPIOA->CRL |= (GPIO_CRL_CNF1_1 | GPIO_CRL_MODE1);
		}

		if(hardFlowControl == CTS || hardFlowControl == RTS_CTS) {
			// Rx pin Input Pull-up
			GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
			GPIOA->CRL |= GPIO_CRL_CNF0_1;
			GPIOA->BSRR |= GPIO_ODR_ODR0;
		}
	} else if(_uart == USART3) { // USART3
		if(hardFlowControl == RTS || hardFlowControl == RTS_CTS) {
			// RTS pin AF Push-Pull out and speed 50MHz
			GPIOB->CRH &= ~GPIO_CRH_CNF14;
			GPIOB->CRH |= (GPIO_CRH_CNF14_1 | GPIO_CRH_MODE14);
		}

		if(hardFlowControl == CTS || hardFlowControl == RTS_CTS) {
			// Rx pin Input Pull-up
			GPIOB->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
			GPIOB->CRH |= GPIO_CRH_CNF13_1;
			GPIOB->BSRR |= GPIO_ODR_ODR13;
		}
	} else {
	}
}

/**
 * @brief  Deinitialization GPIO for UART.
 */
void Uart::deInitGpio() const
{
	if(_uart == USART1) {
		GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
		GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
	} else if(_uart == USART2) {
		GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2);
		GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
	} else if(_uart == USART3) {
		GPIOB->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
		GPIOB->CRH &= ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
	} else {
	}
}

/**
 * @brief  Deinitialization Hardware Flow Control for UART.
 */
void Uart::deInitHardwareFlowControl() const
{
	if(_uart == USART1) {
		GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
		GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
	} else if(_uart == USART2) {
		GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2);
		GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
	} else if(_uart == USART3) {
		GPIOB->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
		GPIOB->CRH &= ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
	} else {
	}
}

/**
 * @brief Enable periph USARTs clock
 */
void Uart::enableUartClock() const
{
	if(_uart == USART1) {
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	} else if(_uart == USART2) {
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	} else if(_uart == USART3) {
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	} else {
	}
}

/**
  * @brief Disable periph USARTs clock
  */
void Uart::disableUartClock() const
{
	if(_uart == USART1) {
		RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
	} else if(_uart == USART2) {
		RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
	} else if(_uart == USART3) {
		RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
	} else {
	}
}

/**
 * @brief  Get type virtual port
 * @retval type UART
 */
VirtualPort::Type Uart::getType()
{
	return VirtualPort::Type::UART;
}

/**
 * @brief Get length data buffer
 * @retval length data
 */
size_t Uart::getLength()
{
	return _receiveBuffer->size();
}

/**
 * @brief Get is empty data buffer
 * @retval true - empty, false - no empty
 */
bool Uart::isEmpty()
{
	return _receiveBuffer->empty();
}

/**
 * @brief Wait complete transfer data
 */
void Uart::waitingCompleteTransfer()
{
	while(_transmitBuffer->size()) {
	}

	while(!isSetStatusFlag(USART_SR_TXE)) {
	}

	while(!isSetStatusFlag(USART_SR_TC)) {
	}
}

/**
 * @brief Transmit data
 * @param [in] data - pointer to buffer data
 * @param [in] len - length data
 */
void Uart::transmit(const void* data, size_t len)
{
	const uint8_t* val = static_cast<const uint8_t*>(data);
	while(len--) {
		pushBuffer(*val++);
	}
}

/**
 * @brief Receive data
 * @param [out] data - pointer to buffer data
 * @param [in] len - length buffer for data
 */
size_t Uart::receive(void* data, size_t len)
{
	uint32_t realLen = 0;
	uint8_t* val = static_cast<uint8_t*>(data);
	while(len--) {
		if(popBuffer(val++) == false) {
			realLen++;
			break;
		}
	}
	return realLen;
}

/**
 * @brief Clean transmiter
 */
void Uart::cleanTransmit()
{
	_transmitBuffer->clear();
}

/**
 * @brief Clean receive
 */
void Uart::cleanReceive()
{
	_receiveBuffer->clear();
	_isError = false;
}

/**
 * @brief  Returns the most recent received data by the _uart peripheral.
 * @param [out] ptr - pointer to data buffer.
 * @retval No data.
 */
bool Uart::popBuffer(uint8_t* const data)
{
	if(_receiveBuffer->empty()) {
		return false;
	}

	*data = _receiveBuffer->pop();
	return true;
}

/**
 * @brief Transmit data
 * @param [in] ptr - data
 */
void Uart::pushBuffer(uint8_t data)
{
	while(_transmitBuffer->overflow()) {
		// Buffer overflow. Waiting for Cleaning.
	}

	if((_transmitBuffer->size() > 0) || (!isSetStatusFlag(USART_SR_TXE))) {
		_transmitBuffer->push(data);

		// Enable the _uart Transmit interrupt
		enableTransmit();
	} else {
		write(data);
	}
}

/**
 * @brief Write data
 * @param [in] data - data
 */
void Uart::write(uint16_t data) const
{
	_uart->DR = (data & 0x01FF);
}

/**
 * @brief Read data
 * @retval data
 */
uint16_t Uart::read() const
{
	return (_uart->DR & 0x01FF);
}

/**
 * @brief Enable transmit interrupt so it sends back the data
 */
void Uart::enableTransmit()
{
	_uart->CR1 |= USART_CR1_TXEIE;
}

/**
 * @brief Disable the TXE interrupt as we don't need it anymore
 */
void Uart::disableTransmit()
{
	_uart->CR1 &= ~USART_CR1_TXEIE;
}

/**
 * @brief Enable USARTs Receive interrupt
 */
void Uart::enableReceive()
{
	_uart->CR1 |= USART_CR1_RXNEIE;
}

/**
 * @brief Disable USARTs Receive interrupt
 */
void Uart::disableReceive()
{
	_uart->CR1 &= ~USART_CR1_RXNEIE;
}

/**
 * @brief Is Set Flag
 */
bool Uart::isSetStatusFlag(uint16_t flag) const
{
	return (_uart->SR & flag);
}

/**
 * @brief Clear Flag
 */
void Uart::clearFlag(uint16_t flag) const
{
	_uart->SR = ~flag;
}

/**
 * @brief Is Set Interrupt Flag
 */
bool Uart::isSetInterruptFlag(uint16_t flag) const
{
	// Get the USART register index
	const uint8_t reg = static_cast<uint8_t>(flag) >> 5;

	// Get the interrupt position
	uint32_t itMask = flag & 0x001F;
	itMask = 1 << itMask;
	bool isCR;
	if(reg == 1) {
		// The IT  is in CR1 register
		isCR = (itMask & _uart->CR1);
	} else if(reg == 2) {
		// The IT  is in CR2 register
		isCR = (itMask & _uart->CR2);
	} else {
		// The IT  is in CR3 register
		isCR = (itMask & _uart->CR3);
	}

	uint32_t bitPos = flag >> 0x08;
	bitPos = 1 << bitPos;
	if((isCR) && (bitPos & _uart->SR)) {
		return true;
	}

	return false;
}

/**
 * @brief Get overflow rx buffer
 * @retval true - empty, false - no empty
 */
bool Uart::isOverflow() const
{
	return _receiveBuffer->overflow();
}

/**
 * @brief Get is error to interface
 * @retval true - error, false - so OK
 */
bool Uart::isError() const
{
	return _isError;
}

/**
 * @brief Clean error
 */
void Uart::cleanError()
{
	_isError = false;
}

/**
 * @brief Interrupt handler from all USARTs
 */
void Uart::handler()
{
	// Interruption for empty transmit register
	if(isSetInterruptFlag(USART_IT_TXE)) {
		if(!_transmitBuffer->empty()) {
			write(_transmitBuffer->pop());
		} else {
			disableTransmit();
		}
	}

	// The interrupt for data reception
	if(isSetInterruptFlag(USART_IT_RXNE)) {
		// Is Error
		bool isError = false;
		if(isSetStatusFlag(USART_SR_FE)) {
			clearFlag(USART_SR_FE);
			// Framing Error
			isError = true;
		}
		if(isSetStatusFlag(USART_SR_NE)) {
			clearFlag(USART_SR_NE);
			// Noise Error
			isError = true;
		}
		if(isSetStatusFlag(USART_SR_PE)) {
			clearFlag(USART_SR_PE);
			// Parity Error
			isError = true;
		}

		// Checking for data reception errors
		if(isError) {
			read();
		}

		if(!isError) {
			_receiveBuffer->push(read());
			_isError = false;
		} else {
			_isError = true;
		}
	}
}

} // namespace stm32f10x_driver_lib
