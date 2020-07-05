/**
 * @brief   This is file realise real time clock.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_H
#define __RTC_H

/* Includes ------------------------------------------------------------------*/
/* Standart lib */
#include <stdint.h>
#include <time.h>

/* Driver MCU */
#include <CMSIS/Include/stm32f10x.h>

/* Driver */
#include <inc/interrupt.h>
#include <inc/clock.h>

/* Utils */
#include <inc/utils/singleton_static.h>
#include <inc/utils/observer.h>

#ifdef __cplusplus

/* C++ lib*/
#include <functional>

/**
 * General struct Data Time
 */
struct RTC_t
{
	uint16_t year; // 1..4095
	uint8_t month; // 1..12
	uint8_t mday;  // 1.. 31
	uint8_t wday;  // 0..6(0 - SUNDAY)

	uint8_t hour; // 0..23
	uint8_t min;  // 0..59
	uint8_t sec;  // 0..59
};

namespace stm32f10x_driver_lib {

class Rtc
  : public SingletonStatic<Rtc>
  , public Clock
  , public Interrupt
  , public Observer
  , public InterruptObserver
{
  public:
	friend class SingletonStatic<Rtc>;

	// Priority interrupt
	enum Priority {
		PREEMPTION_PRIORITY = 0,
		SUB_PRIORITY = 0,
	};

	// Interrupt Handler
	virtual void handler() override;

	// Initialisation alarm handler
	virtual void createInterrupt(uint8_t preemption = PREEMPTION_PRIORITY,
	                             uint8_t sub = SUB_PRIORITY) override;

	void setTime(time_t) const; // Sets RTC_t current time and date

	void setTime(const RTC_t&) const; // Sets RTC_t current time and date

	void setTime(const RTC_t*) const; // Sets RTC_t current time and date

	void getTime(RTC_t*) const; // Geting current time

	void getTime(RTC_t&) const; // Geting current time

	time_t getTime() const; // Geting current time

	time_t dateToSec(const RTC_t*) const; // Converting date to sec

	time_t dateToSec(const RTC_t&) const; // Converting date to sec

	void secToDate(RTC_t*, time_t) const; // Convert sec to date

	// Determines the week number, the day number and the week day number
	uint8_t computeWeekDayNum(uint32_t, uint8_t, uint8_t) const;

	// Set alarm clock
	void setAlarm(const RTC_t* const) const;

	// Set alarm clock
	void setAlarm(time_t) const;

	// Get Alarm time
	time_t getAlarm() const;

	// Reset clock alarm
	void resetAlarm() const;

  private:
	enum Register : uint32_t {
		RTC_BKP_REGISTER = BKP_BASE + 0x0004,
	};

	// RTC Interrupts Flags
	enum InterruptsFlag : uint16_t {
		RTOFF = (1 << 5), //< RTC Operation OFF flag
		CNF = (1 << 4),   //< Configuration flag
		RSF = (1 << 3),   //< Registers Synchronized flag
		OWF = (1 << 2),   //< Overflow flag
		ALR = (1 << 1),   //< Alarm flag
		SEC = (1 << 0)    //< Second flag
	};

	// RTC Masks
	enum Mask : uint32_t {
		LSB = 0x0000FFFF,  //< RTC LSB Mask
		MSB = 0xFFFF0000,  //< RTC MSB Mask
		PRLH = 0x000F0000, //< RTC Prescaler MSB Mask
	};

	enum Prescaler : uint32_t { LSE_PRESCALER = 32767, LSI_PRESCALER = 40000 };

	// For date
	enum Date : uint32_t {
		MILLENNIUM = 2000,
		ONE_YEAR = 365,

		SEC_MILLENNIUM = 946684800,
		SEC_DAY = 86400,
		SEC_HOUR = 3600,
		SEC_MIN = 60
	};

	Rtc();
	~Rtc() = default;

	// Month length
	static constexpr uint8_t getMonLen(size_t);

	// Leap year
	uint16_t getLeapYear(uint16_t) const;

	// RTC return the Counter Value
	time_t getCounter() const; // RTC return the Counter Value

	// RTC set the Counter
	void setCounter(time_t) const; // RTC set the Counter

	// Waits until last write operation on RTC registers has finished
	void waitForLastTask() const;

	// Enters the RTC configuration mode
	void enterConfigMode() const;

	// Exits from the RTC configuration mode
	void exitConfigMode() const;

	// RTC Enable the Alarm
	void enableAlarm() const;

	// RTC Disable the Alarm
	void disableAlarm() const;

	// Enables access to the RTC and backup registers.
	void enableBackupWriteAccess() const;

	// Disable access to the RTC and backup registers
	void disableBackupWriteAccess() const;

	// Sets the RTC prescaler value.
	void setPrescaler(uint32_t prescaler) const;

	// Writes user data to the specified Data Backup Register
	void writeConfigBackupRegister(uint16_t) const;

	// Reads data from the specified Data Backup Register
	uint16_t readConfigBackupRegister() const;

	// Checks whether the specified RTC interrupt alarm has occurred or not
	bool getAlarmStatus();

	// Clears the RTC's interrupt alarm pending bits
	void clearAlarmStatus();
};

} // namespace stm32f10x_driver_lib

extern "C" {
}

#endif // __cplusplus

#endif // __RTC_H
