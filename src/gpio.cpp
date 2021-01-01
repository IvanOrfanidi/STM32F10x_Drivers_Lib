/**
 * @brief   This is file realise real time clock.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <inc/gpio.h>

namespace stm32f10x_driver_lib {
/**
 * @brief Constructor
 * @param [in] port - port identifier
 * @param [in] pin - pin number 
 */
Gpio::Gpio(const GPIO_TypeDef* port, uint8_t pin)
  : _port((GPIO_TypeDef*)port)
  , _pin(1 << pin)
{
	Clock::enable(port);
}

/**
 * @brief Constructor
 * @param [in] port - port identifier
 * @param [in] pin - pin number 
 * @param [in] config - configurate pin
 */
Gpio::Gpio(const GPIO_TypeDef* port, uint8_t pin, const Config& config)
  : Gpio(port, pin)
{
	_pin = (1 << pin);
	init(config);
}

/**
 * @brief Destruction
 */
Gpio::~Gpio()
{
	Config deInit;
	deInit.mode = Mode::INPUT_FLOATING;
	init(deInit);
}

/**
 * @brief Init pin
 * @param [in] config - configurate pin
 */
void Gpio::init(GPIO_TypeDef* port, uint8_t pin, const Config& config)
{
	_port = port;
	_pin = (1 << pin);
	Clock::enable(port);
	init(config);
}

/**
 * @brief Init pin
 * @param [in] config - configurate pin
 */
void Gpio::init(const Config& config) const
{
	uint32_t mode = static_cast<uint32_t>(config.mode) & 0x0F;
	if(static_cast<uint32_t>(config.mode) & 0x10) {
		mode |= static_cast<uint32_t>(config.speed);
	}

	static constexpr uint32_t NUM_PIN = 8;
	uint32_t pinPos;
	if(_pin & 0x00FF) {
		// GPIO CRL Configuration
		uint32_t crl = _port->CRL;
		for(pinPos = 0; pinPos < NUM_PIN; ++pinPos) {
			uint32_t pos = (1 << pinPos);
			if(_pin == pos) {
				pos = (pinPos << 2);
				// Clear the corresponding low control register bits
				const uint32_t mask = (0x0F << pos);
				crl &= ~mask;

				// Write the mode configuration in the corresponding bits
				crl |= (mode << pos);

				if(config.mode == Mode::INPUT_PULL_DOWN) {
					_port->BRR = (1 << pinPos);
				} else if(config.mode == Mode::INPUT_PULL_UP) {
					_port->BSRR = (1 << pinPos);
				}
			}
		}

		_port->CRL = crl;
	} else {
		// GPIO CRH Configuration
		uint32_t crh = _port->CRH;
		for(pinPos = 0; pinPos < NUM_PIN; ++pinPos) {
			uint32_t pos = (1 << (pinPos + NUM_PIN));
			if(_pin == pos) {
				pos = (pinPos << 2);
				// Clear the corresponding low control register bits
				const uint32_t mask = (0x0F << pos);
				crh &= ~mask;

				// Write the mode configuration in the corresponding bits
				crh |= (mode << pos);

				if(config.mode == Mode::INPUT_PULL_DOWN) {
					_port->BRR = (1 << (pinPos + 8));
				} else if(config.mode == Mode::INPUT_PULL_UP) {
					_port->BSRR = (1 << (pinPos + 8));
				}
			}
		}

		_port->CRH = crh;
	}

	// Config pin pull
	if(config.mode == Mode::INPUT_PULL_DOWN) {
		_port->BRR = (1 << pinPos);
	} else if(config.mode == Mode::INPUT_PULL_UP) {
		_port->BSRR = (1 << pinPos);
	}
}

/**
 * @brief Read pin
 * @retval true - flag is set, false - flag is reset
 */
bool Gpio::get() const
{
	return ((_port->IDR & _pin) != 0);
}

/**
 * @brief Set pin
 */
void Gpio::set() const
{
	_port->BSRR = _pin;
}

/**
 * @brief Reset pin
 */
void Gpio::reset() const
{
	_port->BSRR = (_pin << 16);
}

} // namespace stm32f10x_driver_lib
