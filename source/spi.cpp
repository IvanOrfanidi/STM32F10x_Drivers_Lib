/**
 * @brief   This is file realise interface SPI.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <include/spi.h>
#include <type_traits>

namespace stm32f10x_driver_lib {

/**
 * @brief Get instance class
 * @param [in] spi - number object SPI
 * retval instance class
*/
Spi& Spi::getInstance(SPI_TypeDef* const spi)
{
	if(SPI1 == spi) {
		static Spi spi1(spi);
		return spi1;
	} else if(SPI2 == spi) {
		static Spi spi2;
		return spi2;
	}
#ifdef STM32F10X_HD
	else if(SPI3 == spi) {
		static Spi spi3;
		return spi3;
	}
#endif
	else {
		static_assert("Not correct I2C");
	}

	static Spi instance;
	return instance;
}

Spi::Spi()
  : _spi(nullptr)
  , _mode(Mode::MASTER)
  , _receiveBuffer(nullptr)
  , _isError(false)
  , _isCreated(false)
{
}

/**
 * @brief Private constructor class Spi
 * @param [in] spi - number object SPI   
 */
Spi::Spi(SPI_TypeDef* const spi)
  : Spi()
{
	_spi = spi;
	_isCreated = true;
	init(spi);
}

void Spi::init(SPI_TypeDef* const spi)
{
	Clock::reset(spi);
	Clock::enable(spi);
	enable();
}

/**
 * @brief Deinitialization class Spi
 */
void Spi::deInit()
{
	Clock::disable(_spi);
	deInitGpio();
	deInitBuffers();

	_isCreated = false;
}

/**
 * @brief  Initialization SPI
 * @param [in] config - config SPI
 */
void Spi::init(const Config& config, size_t rxSize)
{
	initBuffers(rxSize);
	initGpio();

	const uint16_t cr1 = _spi->CR1 & Mask::CR1_CLEAR;
	_spi->CR1 = cr1 | config.direction | config.mode | config.size |
	            config.polarity | config.phase | config.firstBit |
	            config.prescaler;

	if(Mode::MASTER == config.mode) {
		static constexpr uint16_t NSS_SOFT = 0x0200;
		_spi->CR1 |= NSS_SOFT;
	}

	// Activate the SPI mode
	static constexpr uint16_t SPI_MODE_SELECT = 0xF7FF;
	_spi->I2SCFGR &= SPI_MODE_SELECT;

	_spi->CRCPR = config.crcPolynomial;
}

/**
 * @brief  Initialization Tx buffer and Rx buffer
 * @param [in] rxSize - Rx size
 */
void Spi::initBuffers(size_t rxSize)
{
	if(nullptr == _receiveBuffer) {
		_receiveBuffer = ::new RingBuffer<uint16_t>(rxSize);
	}
}

/**
 * @brief  Deinitialization Tx buffer and Rx buffer
 */
void Spi::deInitBuffers()
{
	::delete _receiveBuffer;
}

/**
 * @brief  Get type virtual port
 * @retval type UART
 */
VirtualPort::Type Spi::getType()
{
	return VirtualPort::Type::SPI;
}

/**
 * @brief Get length data buffer
 * @retval length data
 */
size_t Spi::getLength()
{
	return _receiveBuffer->size();
}

/**
 * @brief Get is empty data buffer
 * @retval true - empty, false - no empty
 */
bool Spi::isEmpty()
{
	return _receiveBuffer->empty();
}

/**
 * @brief Wait complete transfer data
 */
void Spi::waitingCompleteTransfer()
{
	// Wait while spi will not busy
	while(isSetStatusFlag(SPI_SR_BSY)) {
	}

	while(!isSetStatusFlag(SPI_SR_TXE)) {
	}
}

/**
 * @brief Wait while receive
 */
void Spi::waitingReceive() const
{
	// Wait while spi will not busy
	while(isSetStatusFlag(SPI_SR_BSY)) {
	}

	while(!isSetStatusFlag(SPI_SR_RXNE)) {
	}
}

/**
 * @brief Transmit data
 * @param [in] data - pointer to buffer data
 * @param [in] len - length data
 */
void Spi::transmit(const void* data, size_t len)
{
	const uint16_t* val = static_cast<const uint16_t*>(data);
	while(len > 0) {
		waitingCompleteTransfer();

		// Send byte to SPI (TXE cleared)
		write(*val++);

		waitingReceive();

		_receiveBuffer->push(read());
		len--;
	}
}

/**
 * @brief Receive data
 * @param [out] data - pointer to buffer data
 * @param [in] len - length buffer for data
 */
size_t Spi::receive(void* data, size_t len)
{
	size_t realLen = 0;
	uint16_t* val = static_cast<uint16_t*>(data);
	while(len > 0) {
		if(_receiveBuffer->empty()) {
			break;
		}
		*val++ = _receiveBuffer->pop();

		realLen++;
		len--;
	}
	return realLen;
}

/**
 * @brief Enable transmit interrupt so it sends back the data
 */
void Spi::enableTransmit()
{
	_spi->CR2 |= SPI_CR2_TXEIE;
}

/**
 * @brief Disable the TXE interrupt as we don't need it anymore
 */
void Spi::disableTransmit()
{
	_spi->CR2 &= ~SPI_CR2_TXEIE;
}

/**
 * @brief Enable SPIx Receive interrupt
 */
void Spi::enableReceive()
{
	_spi->CR2 |= SPI_CR2_RXNEIE;
}

/**
 * @brief Disable SPIx Receive interrupt
 */
void Spi::disableReceive()
{
	_spi->CR2 &= ~SPI_CR2_RXNEIE;
}

/**
 * @brief Clean transmiter
 */
void Spi::cleanTransmit() {}

/**
 * @brief Clean receive
 */
void Spi::cleanReceive()
{
	_receiveBuffer->clear();
	_isError = false;
}

/**
 * @brief Initialization GPIO for SPI.
 * @info :
 * SCK - 
 *  master: Alternate function push-pull
 *  slave: Input floating
 * MOSI - 
 *  master: Alternate function push-pull, 
 *  slave: Input floating / Input pull-up
 * MISO - 
 *  master Input floating / Input pull-up
 *  slave Alternate function push-pull(point to point) Alternate function open drain (multi-slave)
 * NSS(if hardware) - 
 *  master Input floating/ Input pull-up / Input pull-down
 *  slave Input floating/ Input pull-up / Input pull-down
 */
void Spi::initGpio() const
{
	if(_spi == SPI1) {
		// Port clock
		Clock::enable(GPIOA);

		// SCK
		if(_mode == Mode::MASTER) {
			// Pin AF Push-Pull out and speed 50MHz
			GPIOA->CRL &= ~GPIO_CRL_CNF5;
			GPIOA->CRL |= (GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5);
		} else {
			// Pin Input floating
			GPIOA->CRL &= ~(GPIO_CRL_CNF5);
			GPIOA->CRL |= GPIO_CRL_CNF5_0;
		}

		// MOSI
		if(_mode == Mode::MASTER) {
			// Pin AF Push-Pull out and speed 50MHz
			GPIOA->CRL &= ~GPIO_CRL_CNF7;
			GPIOA->CRL |= (GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7);
		} else {
			// Pin Input floating
			GPIOA->CRL &= ~(GPIO_CRL_CNF7);
			GPIOA->CRL |= GPIO_CRL_CNF7_0;
		}

		// MISO
		if(_mode == Mode::MASTER) {
			// Pin Input floating
			GPIOA->CRL &= ~(GPIO_CRL_CNF6);
			GPIOA->CRL |= GPIO_CRL_CNF6_0;
		} else {
			// Pin AF Push-Pull out and speed 50MHz
			GPIOA->CRL &= ~GPIO_CRL_CNF6;
			GPIOA->CRL |= (GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6);
		}
	} else if(_spi == SPI2) {
		// Port clock
		Clock::enable(GPIOB);

		// SCK
		if(_mode == Mode::MASTER) {
			// Pin AF Push-Pull out and speed 50MHz
			GPIOB->CRH &= ~GPIO_CRH_CNF13;
			GPIOB->CRH |= (GPIO_CRH_CNF13_1 | GPIO_CRH_MODE13);
		} else {
			// Pin Input floating
			GPIOB->CRH &= ~(GPIO_CRH_CNF13);
			GPIOB->CRH |= GPIO_CRH_CNF13_0;
		}

		// MOSI
		if(_mode == Mode::MASTER) {
			// Pin AF Push-Pull out and speed 50MHz
			GPIOB->CRH &= ~GPIO_CRH_CNF15;
			GPIOB->CRH |= (GPIO_CRH_CNF15_1 | GPIO_CRH_MODE15);
		} else {
			// Pin Input floating
			GPIOB->CRH &= ~(GPIO_CRH_CNF15);
			GPIOB->CRH |= GPIO_CRH_CNF15_0;
		}

		// MISO
		if(_mode == Mode::MASTER) {
			// Pin Input floating
			GPIOB->CRH &= ~(GPIO_CRH_CNF14);
			GPIOB->CRH |= GPIO_CRH_CNF14_0;
		} else {
			// Pin AF Push-Pull out and speed 50MHz
			GPIOB->CRH &= ~GPIO_CRH_CNF14;
			GPIOB->CRH |= (GPIO_CRH_CNF14_1 | GPIO_CRH_MODE14);
		}
	}
#ifdef STM32F10X_HD
	else if(_spi == SPI3) {
		// Port clock
		Clock::enable(GPIOB);

		// SCK
		if(_mode == Mode::MASTER) {
			// Pin AF Push-Pull out and speed 50MHz
			GPIOB->CRL &= ~GPIO_CRL_CNF3;
			GPIOB->CRL |= (GPIO_CRL_CNF3_1 | GPIO_CRL_MODE3);
		} else {
			// Pin Input floating
			GPIOB->CRL &= ~(GPIO_CRL_CNF3);
			GPIOB->CRL |= GPIO_CRL_CNF3_0;
		}

		// MOSI
		if(_mode == Mode::MASTER) {
			// Pin AF Push-Pull out and speed 50MHz
			GPIOB->CRL &= ~GPIO_CRL_CNF5;
			GPIOB->CRL |= (GPIO_CRL_CNF5_1 | GPIO_CRL_MODE5);
		} else {
			// Pin Input floating
			GPIOB->CRL &= ~(GPIO_CRL_CNF5);
			GPIOB->CRL |= GPIO_CRL_CNF5_0;
		}

		// MISO
		if(_mode == Mode::MASTER) {
			// Pin Input floating
			GPIOB->CRL &= ~(GPIO_CRL_CNF4);
			GPIOB->CRL |= GPIO_CRL_CNF4_0;
		} else {
			// Pin AF Push-Pull out and speed 50MHz
			GPIOB->CRL &= ~GPIO_CRL_CNF4;
			GPIOB->CRL |= (GPIO_CRL_CNF4_1 | GPIO_CRL_MODE4);
		}
	}
#endif
	else {
	}

	Clock::enableAlternateFunction();
}

/**
 * @brief Deinitialization GPIO
 */
void Spi::deInitGpio() const
{
	if(_spi == SPI1) {
		GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
		GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);
		GPIOA->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
	} else if(_spi == SPI2) {
		GPIOB->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
		GPIOB->CRH &= ~(GPIO_CRH_CNF14 | GPIO_CRH_MODE14);
		GPIOB->CRH &= ~(GPIO_CRH_CNF15 | GPIO_CRH_MODE15);
	}
#ifdef STM32F10X_HD
	else if(_spi == SPI3) {
		GPIOB->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
		GPIOB->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4);
		GPIOB->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
	}
#endif
	else {
	}
}

/**
 * @brief Enable SPI
 */
void Spi::enable() const
{
	_spi->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief Disable SPI
 */
void Spi::disable() const
{
	_spi->CR1 &= ~SPI_CR1_SPE;
}

/**
 * @brief Enable CRC value calculation of the transferred bytes
 */
void Spi::enableCalculateCrc() const
{
	static constexpr uint16_t CRC_ENABLE = 0x2000;
	_spi->CR1 |= CRC_ENABLE;
}

/**
 * @brief Disable CRC value calculation of the transferred bytes
 */
void Spi::disableCalculateCrc() const
{
	static constexpr uint16_t CRC_DISABLE = 0xDFFF;
	_spi->CR1 &= ~CRC_DISABLE;
}

/**
 * @brief Configures the data size for the selected SPI
 * @param [in] size - specifies the SPI data size
 */
void Spi::setSize(Size size)
{
	_spi->CR1 &= Size::_16B;
	_spi->CR1 |= size;
}

/**
 * @brief Write data
 * @param [in] data - data
 */
void Spi::write(uint16_t data) const
{
	_spi->DR = data;
}

/**
 * @brief Read data
 * @retval data
 */
uint16_t Spi::read() const
{
	return _spi->DR;
}

/**
 * @brief Is Set Flag
 */
bool Spi::isSetStatusFlag(uint16_t flag) const
{
	return (_spi->SR & flag);
}

/**
 * @brief Clear Flag
 */
void Spi::clearFlag(uint16_t flag) const
{
	_spi->SR = ~flag;
}

} // namespace stm32f10x_driver_lib
