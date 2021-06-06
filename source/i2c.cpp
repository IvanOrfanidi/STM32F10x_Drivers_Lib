/**
 * @brief   This is file realise interface I2C
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <include/i2c.h>
#include <include/utils/lock_interrupt.h>
#include <type_traits>

namespace stm32f10x_driver_lib {

/**
 * @brief Get instance class
 * @param [in] i2c - number object I2C
 * retval instance class
*/
I2c& I2c::getInstance(I2C_TypeDef* const i2c)
{
	size_t interface; // I2C number
	if(i2c == I2C1) {
		static I2c i2c1(i2c);
		if(!i2c1._isCreated) {
			LockInterrupt lockInterrupt;
			if(!i2c1._isCreated) {
				i2c1.Clock::reset(i2c);
				i2c1.Clock::enable(i2c);

				i2c1.Clock::enableAlternateFunction();
				i2c1._scl.init(
				    GPIOB,
				    6,
				    {Gpio::Mode::OUTPUT_OPEN_DRAIN, Gpio::Speed::_50mhz});
				i2c1._sda.init(
				    GPIOB,
				    7,
				    {Gpio::Mode::OUTPUT_OPEN_DRAIN, Gpio::Speed::_50mhz});

				i2c1._isCreated = true;
			}
		}
		return i2c1;
	} else if(i2c == I2C2) {
		static I2c i2c2(i2c);
		if(!i2c2._isCreated) {
			LockInterrupt lockInterrupt;
			if(!i2c2._isCreated) {
				i2c2.Clock::reset(i2c);
				i2c2.Clock::enable(i2c);

				i2c2.Clock::enableAlternateFunction();
				i2c2._scl.init(
				    GPIOB,
				    10,
				    {Gpio::Mode::OUTPUT_OPEN_DRAIN, Gpio::Speed::_50mhz});
				i2c2._sda.init(
				    GPIOB,
				    11,
				    {Gpio::Mode::OUTPUT_OPEN_DRAIN, Gpio::Speed::_50mhz});

				i2c2._isCreated = true;
			}
		}
		return i2c2;
	} else {
		static_assert("Not correct I2C");
	}

	static I2c instance;
	return instance;
}

I2c::I2c()
  : _i2c(nullptr)
  , _sysClock(0)
  , _isError(false)
  , _isCreated(false)
{
}

I2c::I2c(I2C_TypeDef* const i2c)
  : I2c()
{
	_i2c = i2c;
}

/**
 * @brief Destructor
 */
I2c::~I2c()
{
	_scl.init({Gpio::Mode::INPUT_FLOATING, Gpio::Speed::_2mhz});
	_sda.init({Gpio::Mode::INPUT_FLOATING, Gpio::Speed::_2mhz});
	Clock::disable(_i2c);
}

/**
 * @brief  Initialization I2C and interrupt this
 * @param [in] config - config I2C
 */
void I2c::init(const Config& config)
{
	_sysClock = config.sysClock;
	_dutyCycle = config.dutyCycle;

	setFrequency(config.frequency);

	if(config.isAck) {
		enableAck();
	} else {
		disableAck();
	}

	setAcknowledgedAddress(config.acknowledgedAddress);
	setAddress(config.address);

	enable();
}

void I2c::setAcknowledgedAddress(AcknowledgedAddress acknowledgedAddress) const
{
	_i2c->OAR1 |= static_cast<uint16_t>(acknowledgedAddress);
}

void I2c::setAddress(uint16_t address) const
{
	_i2c->OAR1 |= address;
}

void I2c::setFrequency(uint32_t frequency)
{
	// Set frequency bits depending on frequency value
	static constexpr uint16_t CR2_FREQ_RESET = 0xFFC0;
	const uint16_t cr2 = _i2c->CR2 & CR2_FREQ_RESET;

	const uint32_t clock = getApbClock(_sysClock);
	const uint16_t freqRange = clock / 1'000'000;
	_i2c->CR2 = cr2 | freqRange;

	disable();

	// Configure speed in standard mode <100kHz
	uint16_t res;
	if(frequency <= 100'000) {
		res = clock / (frequency << 1);
		// Test if CCR value is under 0x4. Set minimum allowed value
		res = (res < 4) ? 4 : res;
		_i2c->CCR = res;
		_i2c->TRISE = freqRange;
	} else {
		// Configure speed in fast mode 100...400kHz
		if(_dutyCycle == DutyCycle::_2tt) {
			res = clock / (frequency * 3);
		} else {
			res = clock / (frequency * 25);
			res |= static_cast<uint16_t>(DutyCycle::_16_9tt);
		}

		static constexpr uint16_t CCR_CCR_SET = 0x0FFF;
		if(0 == (res & CCR_CCR_SET)) {
			res |= 0x0001;
		}

		static constexpr uint16_t CCR_FS_SET = 0x8000;
		res |= CCR_FS_SET;

		_i2c->TRISE = ((freqRange * 300) / 1'000) + 1;
	}

	_i2c->CCR = res;

	enable();
}

/**
 * @brief  Get APB1/APB2 clock frequency for I2C
 * @param [in] sysClock - system clock in Hz
 * @retval clock frequency in Hz
 */
uint32_t I2c::getApbClock(uint32_t sysClock) const
{
	// Get HCLK prescaler
	static constexpr uint32_t CFGR_HPRE_SET_MASK = 0x00F0;
	uint32_t temp = RCC->CFGR & CFGR_HPRE_SET_MASK;
	temp >>= 4;

	// Get HCLK clock frequency
	static constexpr uint8_t APB_AHB_PRESC_TABLE[16] = {
	    0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
	uint32_t presc = APB_AHB_PRESC_TABLE[temp];
	const uint32_t hclk = sysClock >> presc;

	// Get APB1 prescaler
	static constexpr uint32_t CFGR_PPRE1_SET_MASK = 0x0700;
	temp = RCC->CFGR & CFGR_PPRE1_SET_MASK;
	temp >>= 8;
	presc = APB_AHB_PRESC_TABLE[temp];

	// Get APB1/APB2 clock frequency
	const uint32_t clock = hclk >> presc;
	return clock;
}

/**
 * @brief Enable I2C
 */
void I2c::enable() const
{
	_i2c->CR1 |= I2C_CR1_PE;
}

/**
 * @brief Disable I2C
 */
void I2c::disable() const
{
	_i2c->CR1 &= ~I2C_CR1_PE;
}

/**
 * @brief  Get type virtual port
 * @retval type UART
 */
VirtualPort::Type I2c::getType()
{
	return VirtualPort::Type::I2C;
}

/**
 * @brief Get length data buffer
 * @retval length data
 */
size_t I2c::getLength()
{
	return 0;
}

/**
 * @brief Get is empty data buffer
 * @retval true - empty, false - no empty
 */
bool I2c::isEmpty()
{
	return true;
}

/**
 * @brief Wait complete transfer data
 */
void I2c::waitingCompleteTransfer() {}

/**
 * @brief Transmit data
 * @param [in] data - pointer to buffer data
 * @param [in] len - length data
 */
void I2c::transmit(const void* data, size_t len) {}

/**
 * @brief Receive data
 * @param [out] data - pointer to buffer data
 * @param [in] len - length buffer for data
 */
size_t I2c::receive(void* data, size_t len)
{
	return 0;
}

/**
 * @brief Clean transmiter
 */
void I2c::cleanTransmit() {}

/**
 * @brief Clean receive
 */
void I2c::cleanReceive()
{
	_isError = false;
}

/**
 * @brief Enable transmit interrupt so it sends back the data
 */
void I2c::enableTransmit() {}

/**
 * @brief Disable the TXE interrupt as we don't need it anymore
 */
void I2c::disableTransmit() {}

/**
 * @brief Enable SPIx Receive interrupt
 */
void I2c::enableReceive() {}

/**
 * @brief Disable SPIx Receive interrupt
 */
void I2c::disableReceive() {}

/**
 * @brief Write data
 * @param [in] data - data
 */
void I2c::write(uint8_t data) const
{
	_i2c->DR = data;
}

/**
 * @brief Read data
 * @retval data
 */
uint8_t I2c::read() const
{
	return _i2c->DR;
}

bool I2c::isSetStart() const
{
	constexpr uint16_t CR1_START_SET = 0x0100;
	_i2c->CR1 |= CR1_START_SET;

	auto timeout = CHECK_EVENT_TIMEOUT;
	while(!(_i2c->SR1 & I2C_SR1_SB)) {
		if(timeout-- <= 0) {
			return false;
		}
	}
	return true;
}

bool I2c::isSetAddress(uint8_t address) const
{
	write((address << 1) | 1);
	auto timeout = CHECK_EVENT_TIMEOUT;
	while(!(_i2c->SR1 & I2C_SR1_ADDR)) {
		if(timeout-- <= 0) {
			return false;
		}
	};
	return true;
}

void I2c::setStart() const
{
	_i2c->CR1 |= I2C_CR1_START;
}

void I2c::resetStart() const
{
	constexpr uint16_t CR1_START_RESET = 0xFEFF;
	_i2c->CR1 &= CR1_START_RESET;
}

void I2c::setStop() const
{
	constexpr uint16_t CR1_STOP_SET = 0x0200;
	_i2c->CR1 |= CR1_STOP_SET;
}

void I2c::resetStop() const
{
	constexpr uint16_t CR1_STOP_RESET = 0xFDFF;
	_i2c->CR1 &= CR1_STOP_RESET;
}

void I2c::enableAck() const
{
	_i2c->CR1 |= I2C_CR1_ACK;
}

void I2c::disableAck() const
{
	_i2c->CR1 &= ~I2C_CR1_ACK;
}

} // namespace stm32f10x_driver_lib