/**
 * @brief   This file provides all the RCC(reset and clock control) firmware functions.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <inc/clock.h>

namespace stm32f10x_driver_lib {
/**
 * @brief Enable GPIO clock
 * @param [in] port - I/O port
 */
void Clock::enable(const GPIO_TypeDef* port) const
{
	if(port == GPIOA) {
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	} else if(port == GPIOB) {
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	} else if(port == GPIOC) {
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	} else if(port == GPIOD) {
		RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	} else if(port == GPIOE) {
		RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;
	} else {
	}
}

/**
 * @brief Disable GPIO clock
 * @param [in] port - I/O port
 */
void Clock::disable(const GPIO_TypeDef* port) const
{
	if(port == GPIOA) {
		RCC->APB2ENR &= ~RCC_APB2ENR_IOPAEN;
	} else if(port == GPIOB) {
		RCC->APB2ENR &= ~RCC_APB2ENR_IOPBEN;
	} else if(port == GPIOC) {
		RCC->APB2ENR &= ~RCC_APB2ENR_IOPCEN;
	} else if(port == GPIOD) {
		RCC->APB2ENR &= ~RCC_APB2ENR_IOPDEN;
	} else if(port == GPIOE) {
		RCC->APB2ENR &= ~RCC_APB2ENR_IOPEEN;
	} else {
	}
}

/**
 * @brief Enable SPI clock
 * @param [in] spi - num spi
 */
void Clock::enable(const SPI_TypeDef* spi) const
{
	if(SPI1 == spi) {
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	} else if(SPI2 == spi) {
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	}
#ifdef STM32F10X_HD
	else if(SPI3 == spi) {
		RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
	}
#endif
	else {
	}
}

/**
 * @brief Disable SPI clock
 * @param [in] spi - num spi
 */
void Clock::disable(const SPI_TypeDef* spi) const
{
	if(SPI1 == spi) {
		RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
	} else if(SPI2 == spi) {
		RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
	}
#ifdef STM32F10X_HD
	else if(SPI3 == spi) {
		RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
	}
#endif
	else {
	}
}

/**
 * @brief Enable Alternate Function for GPIO
 */
void Clock::enableAlternateFunction() const
{
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
}

/**
 * @brief Disable Alternate Function for GPIO
 */
void Clock::disableAlternateFunction() const
{
	RCC->APB2ENR &= ~RCC_APB2ENR_AFIOEN;
}

/**
  * @brief Enable HSE
  */
void Clock::enableHighSpeedExternalClock() const
{
	RCC->CR |= RCC_CR_HSEON;
}

/**
  * @brief Disable HSE
  */
void Clock::disableHighSpeedExternalClock() const
{
	RCC->CR &= ~RCC_CR_HSEON;
}

/**
  * @brief Enable LSE
  */
void Clock::enableLowSpeedExternalClock() const
{
	RCC->BDCR |= RCC_BDCR_LSEON;
}

/**
  * @brief Disable LSE
  */
void Clock::disableLowSpeedExternalClock() const
{
	RCC->BDCR &= ~RCC_BDCR_LSEON;
}

/**
  * @brief Enable HSI
  */
void Clock::enableHighSpeedInternalClock() const
{
	RCC->CR |= RCC_CR_HSION;
}

/**
  * @brief Disable HSI
  */
void Clock::disableHighSpeedInternalClock() const
{
	RCC->CR &= ~RCC_CR_HSION;
}

/**
  * @brief Enable LSI
  */
void Clock::enableLowSpeedInternalClock() const
{
	RCC->CSR |= RCC_CSR_LSION;
}

/**
  * @brief Disable LSI
  */
void Clock::disableLowSpeedInternalClock() const
{
	RCC->CSR &= ~RCC_CSR_LSION;
}

/**
  * @brief Enable PLL
  */
void Clock::enablePhaseLockedLoopsClock() const
{
	RCC->CR |= RCC_CR_PLLON;
}

/**
  * @brief Disable PLL
  */
void Clock::disablePhaseLockedLoopsClock() const
{
	RCC->CR &= ~RCC_CR_PLLON;
}

/*
 * @brief Set and cleared by hardware to indicate which Clock Source is used as system clock
 * @param [in] source - Clock Source(HSI, HSE or PLL)
 * 00: HSI oscillator used as system clock
 * 01: HSE oscillator used as system clock
 * 10: PLL used as system clock
 */
void Clock::setSystemSource(ClockSource source) const
{
	const uint32_t clock = static_cast<uint32_t>(source);
	if(clock < static_cast<uint32_t>(ClockSource::LSE)) {
		RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | clock;
	}
}

/**
  * @brief Configure Clock Source
  * @param [in] source - Clock Source(LSE, LSI or HSE)
  */
void Clock::setRtcSource(ClockSource source) const
{
	switch(source) {
		case ClockSource::LSE:
			RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;
			break;

		case ClockSource::HSE:
			RCC->BDCR |= RCC_BDCR_RTCSEL_HSE;
			break;

		case ClockSource::LSI:
			RCC->BDCR |= RCC_BDCR_RTCSEL_LSI;
			break;

		default:
			break;
	}
}

/**
  * @brief Return ready Clock Source
  * @param [in] source - Clock Source(HSI, HSE, PLL, LSE, LSI or HSE)
  * @retval true - success , false - fail
  */
bool Clock::isReadyClockSource(ClockSource source) const
{
	bool isReady = false;

	switch(source) {
		case ClockSource::LSE:
			isReady = RCC->BDCR & RCC_BDCR_LSERDY;
			break;

		case ClockSource::HSE:
			isReady = RCC->CR & RCC_CR_HSERDY;
			break;

		case ClockSource::HSI:
			isReady = RCC->CR & RCC_CR_HSIRDY;
			break;

		case ClockSource::PLL:
			isReady = RCC->CR & RCC_CR_PLLRDY;
			break;

		case ClockSource::LSI:
			isReady = RCC->CSR & RCC_CSR_LSIRDY;
			break;

		default:
			break;
	}

	return isReady;
}

/**
 * @brief Get the System Clock Source
 * @return ClockSource - (HSI, HSE or PLL)
 */
Clock::ClockSource Clock::getSystemClockSource() const
{
	ClockSource systemClockSource;
	const uint32_t cfgrSws = RCC->CFGR & RCC_CFGR_SWS;
	switch(cfgrSws) {
		case 0x00U: /* HSI used as system clock */
			systemClockSource = ClockSource::HSI;
			break;

		case 0x04U: /* HSE used as system clock */
			systemClockSource = ClockSource::HSE;
			break;

		case 0x08U: /* PLL used as system clock */
			systemClockSource = ClockSource::PLL;
			break;

		default:
			break;
	}

	return systemClockSource;
}

/**
  * @brief Set the AHB Prescale Factor
  */
void Clock::setAhbPrescaleFactor(uint32_t prescaler) const
{
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | prescaler;
}

/**
  * @brief Set the APB1 Prescale Factor
  */
void Clock::setApb1PrescaleFactor(uint32_t prescaler) const
{
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | prescaler;
}

//void Clock::setAdcPrescaleFactor

/**
  * @brief Set the APB2 Prescale Factor
  */
void Clock::setApb2PrescaleFactor(uint32_t prescaler) const
{
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | prescaler;
}

/**
 * @brief Set the PLL Multiplication Factor
 * @param [in] pllMultiplicationFactor - PLL Multiplication Factor
 */
void Clock::setPllMultiplicationFactor(uint32_t pllMultiplicationFactor) const
{
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PLLMULL) | pllMultiplicationFactor;
}

/**
 * @brief Set the PLL Source object
 * @param pllSource [in]
 */
void Clock::setPllSource(uint32_t pllSource) const
{
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PLLSRC) | pllSource;
}

/**
  * @brief Enable BKP
  */
void Clock::enableBackupRegister() const
{
	RCC->APB1ENR |= RCC_APB1RSTR_BKPRST | RCC_APB1RSTR_PWRRST;
}

/**
  * @brief Enable RTC
  */
void Clock::enableRrealTimeClock() const
{
	RCC->BDCR |= RCC_BDCR_RTCEN;
}

/**
  * @brief Disable RTC
  */
void Clock::disableRrealTimeClock() const
{
	RCC->BDCR &= ~RCC_BDCR_RTCEN;
}

/**
  * @brief Return ready RTC clock
  * @retval true - success , false - fail
  */
bool Clock::isReadyRrealTimeClock() const
{
	return (RCC->BDCR & RCC_BDCR_RTCEN);
}

/**
  * @brief Reset Backup Domain
  * @note  Deinitializes the BKP peripheral registers to their default reset values
  */
void Clock::backupReset() const
{
	// Set the backup domain software reset
	RCC->BDCR |= RCC_BDCR_BDRST;

	// Clear the backup domain software reset
	RCC->BDCR &= ~RCC_BDCR_BDRST;
}

/**
 * @brief Deinitialization SPI
 * @param [in] spi - num spi
 */
void Clock::spiReset(const SPI_TypeDef* spi) const
{
	if(SPI1 == spi) {
		RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
	} else if(SPI2 == spi) {
		RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;
	}
#ifdef STM32F10X_HD
	else if(SPI3 == spi) {
		RCC->APB1RSTR |= RCC_APB1RSTR_SPI3RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI3RST;
	}
#endif
	else {
	}
}

/**
 * @brief RCC Enable the Clock Security System
 */
void Clock::enableClockSecuritySystem() const
{
	RCC->CR |= RCC_CR_CSSON;
}

/**
 * @brief RCC Disable the Clock Security System
 */
void Clock::disableClockSecuritySystem() const
{
	RCC->CR &= ~RCC_CR_CSSON;
}

} // namespace stm32f10x_driver_lib
