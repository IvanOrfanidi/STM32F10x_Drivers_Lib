/**
 * @brief   This file provides all the RCC(reset and clock control) firmware functions.
 *
 */

#ifndef __CLOCK_H
#define __CLOCK_H

/* Includes ------------------------------------------------------------------*/
/* Standard lib */
#include <stdint.h>

/* Driver MCU */
#include <CMSIS/Include/stm32f10x.h>

/* Utils */
#include <inc/utils/non_copyable.h>
#include <inc/utils/non_movable.h>

#ifdef __cplusplus

namespace stm32f10x_driver_lib {

class Clock
  : public NonCopyable<Clock>
  , public NonMovable<Clock>
{
  public:
	enum class ClockSource {
		HSI = (1 << 0),
		HSE = (1 << 1),
		PLL = (1 << 2),
		LSE = (1 << 3),
		LSI = (1 << 4)
	};

	Clock() = default;
	virtual ~Clock() = default;

	// Enable GPIO clock
	void enable(const GPIO_TypeDef*) const;

	// Disable GPIO clock
	void disable(const GPIO_TypeDef*) const;

	// Enable SPI clock
	void enable(const SPI_TypeDef*) const;

	// Disable SPI clock
	void disable(const SPI_TypeDef*) const;

	// Enable Alternate Function for GPIO
	void enableAlternateFunction() const;

	// Disable Alternate Function for GPIO
	void disableAlternateFunction() const;

	// Enable HSE Clock
	void enableHighSpeedExternalClock() const;

	// Disable HSE
	void disableHighSpeedExternalClock() const;

	// Enable LSE Clock
	void enableLowSpeedExternalClock() const;

	// Disable LSE
	void disableLowSpeedExternalClock() const;

	// Enable LSI
	void enableLowSpeedInternalClock() const;

	// Disable LSI
	void disableLowSpeedInternalClock() const;

	// Enable HSI
	void enableHighSpeedInternalClock() const;

	// Disable HSI
	void disableHighSpeedInternalClock() const;

	// Set the AHB Prescale Factor
	void setAhbPrescaleFactor(uint32_t prescaler) const;

	// Set the APB1 Prescale Factor
	void setApb1PrescaleFactor(uint32_t prescaler) const;

	// Set the APB2 Prescale Factor
	void setApb2PrescaleFactor(uint32_t prescaler) const;

	// Enable BKP
	void enableBackupRegister() const;

	// Enable RTC
	void enableRrealTimeClock() const;

	// Disable RTC
	void disableRrealTimeClock() const;

	void setSystemSource(ClockSource) const;

	// Configure clock source
	void setRtcSource(ClockSource) const;

	// Return ready clock source
	bool isReadyClockSource(ClockSource) const;

	bool isReadyRrealTimeClock() const;

	// Reset Backup Domain
	void backupReset() const;

	// Reset SPI Domain
	void spiReset(const SPI_TypeDef*) const;

	// RCC Enable the Clock Security System
	void enableClockSecuritySystem() const;

	// RCC Disable the Clock Security System
	void disableClockSecuritySystem() const;
};

} // namespace stm32f10x_driver_lib

extern "C" {
}

#endif // __cplusplus

#endif // __CLOCK_H
