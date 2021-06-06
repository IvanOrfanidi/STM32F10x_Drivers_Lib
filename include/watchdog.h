/**
 * @brief   This is file realise watchdog timer.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WATCHDOG_H
#define __WATCHDOG_H

/* Standart lib */
#include <stdint.h>

/* Driver MCU */
#include <CMSIS/Include/stm32f10x.h>

/* Utils */
#include <include/utils/singleton_static.h>

#ifdef __cplusplus

namespace stm32f10x_driver_lib {

class Watchdog : public SingletonStatic<Watchdog>
{
  public:
	friend class SingletonStatic<Watchdog>;

	// Prescalers
	enum Prescaler : uint8_t {
		_4 = 0,
		_8 = 1,
		_16 = 2,
		_32 = 3,
		_64 = 4,
		_128 = 5,
		_256 = 6
	};

	// Initialisation watchdog timer
	void init(uint32_t) const;

	// Enable(start) watchdog
	void start() const;

	// Enable(start) watchdog
	void enable() const;

	// Reloads watchdog counter
	void reset() const;

	// Reloads watchdog counter
	void reload() const;

	// Sets watchdog reload value
	void setReload(uint16_t) const;

	// Sets watchdog prescaler value
	void setPrescaler(Prescaler) const;

  private:
	enum Flag : uint16_t { PVU = (1 << 0), RVU = (1 << 1) };

	Watchdog();
	~Watchdog() = default;

	// Get reload register status
	bool isReloadBusy() const;

	// Get prescaler register status
	bool isPrescalerBusy() const;

	// Enable write access
	void enableWriteAccess() const;

	// Disable write access
	void disableWriteAccess() const;
};

} // namespace stm32f10x_driver_lib

extern "C" {
}

#endif // __cplusplus

#endif // __WATCHDOG_H
