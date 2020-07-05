/**
 * @brief   This is file realise watchdog timer.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <inc/watchdog.h>

namespace stm32f10x_driver_lib {
/**
 * @brief Constructor
*/
Watchdog::Watchdog()
{
	// Enable LSI
	RCC->CSR |= RCC_CSR_LSION;
}

/**
 * @brief Enable(start) watchdog
 */
void Watchdog::start() const
{
	enable();
}

/**
 * @brief Enable(start) watchdog
 */
void Watchdog::enable() const
{
	constexpr uint16_t KeyEnable = 0xCCCC;
	IWDG->KR = KeyEnable;
}

/**
 * @brief Reloads(reset) watchdog counter
 */
void Watchdog::reset() const
{
	constexpr uint16_t KeyReload = 0xCCCC;
	IWDG->KR = KeyReload;
}

/**
 * @brief Reloads(reset) watchdog counter
 */
void Watchdog::reload() const
{
	reset();
}

/**
 * @brief Initialisation watchdog timer
 * @param [in] period in milliseconds (< 32760) from a watchdog reset until 
 *              a system reset is issued
 */
void Watchdog::init(uint32_t period) const
{
	constexpr uint32_t COUNT_LENGTH = 12;
	constexpr uint32_t COUNT_MASK = (1 << COUNT_LENGTH) - 1;
	constexpr uint32_t MAX_PERIOD_MS = 32760;

	period = (period > MAX_PERIOD_MS) ? MAX_PERIOD_MS : period;

	/* Set the count to represent ticks of 8kHz clock (the 32kHz LSI clock
	 * divided by 4 = lowest prescaler setting) */
	uint32_t reload = period << 3;

	// Prevent underflow
	reload = (0 == reload) ? 1 : reload;

	/* Shift count while increasing prescaler as many times as needed to
	 * fit into IWDG_RLR */
	uint8_t prescale = 0;
	while((reload - 1) >> COUNT_LENGTH) {
		reload >>= 1;
		prescale++;
	}

	// IWDG_RLR actually holds count - 1
	--reload;

	while(isPrescalerBusy()) {
	}
	enableWriteAccess();
	setPrescaler(static_cast<Prescaler>(prescale));

	while(isReloadBusy()) {
	}
	enableWriteAccess();
	setReload(reload & COUNT_MASK);
}

/**
 * @brief Get reload register status
 * @retval true - prescaler register is busy
 */
bool Watchdog::isReloadBusy() const
{
	return (IWDG->SR & Flag::RVU);
}

/**
 * @brief Get prescaler register status
 * @retval true - prescaler register is busy
 */
bool Watchdog::isPrescalerBusy() const
{
	return (IWDG->SR & Flag::PVU);
}

/**
 * @brief Sets watchdog reload value
 * @param [in] - reload value
 */
void Watchdog::setReload(uint16_t reload) const
{
	IWDG->RLR = reload;
}

/**
 * @brief Sets watchdog prescaler value
 * @param [in] - prescaler value (4...256)
 */
void Watchdog::setPrescaler(Prescaler prescaler) const
{
	IWDG->PR = static_cast<uint8_t>(prescaler);
}

/**
 * @brief Enable write access
 */
void Watchdog::enableWriteAccess() const
{
	constexpr uint16_t WriteAccessEnable = 0x5555;
	IWDG->KR = WriteAccessEnable;
}

/**
 * @brief Disable write access
 */
void Watchdog::disableWriteAccess() const
{
	constexpr uint16_t WriteAccessDisable = 0x0000;
	IWDG->KR = WriteAccessDisable;
}

} // namespace stm32f10x_driver_lib
