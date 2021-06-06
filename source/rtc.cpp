/**
 * @brief   This is file realise real time clock.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <include/rtc.h>

namespace stm32f10x_driver_lib {
/**
 * @brief Constructor
*/
Rtc::Rtc()
{
	Clock::enableLowSpeedInternalClock();
	Clock::enableBackupRegister();
	disableBackupWriteAccess();

	ClockSource status = static_cast<ClockSource>(readConfigBackupRegister());
	if(status != ClockSource::LSE && status != ClockSource::LSI) {
		Clock::backupReset();
		Clock::enableLowSpeedExternalClock();

		// Wait till LSE is ready
		auto timeout = 100000;
		while(!Clock::isReadyClockSource(ClockSource::LSE)) {
			if(0 == --timeout) {
				break;
			}
		}

		// LSE - OK
		if(timeout) {
			// Select LSE as RTC_t Clock Source
			Clock::setRtcSource(ClockSource::LSE); // LSE = 32.768 KHz
			waitForLastTask();

			Clock::enableRrealTimeClock();
			while(!Clock::isReadyRrealTimeClock()) {
				__NOP();
			}

			// Set RTC_t prescaler: set RTC_t period to 1sec
			// RTC_t period = RTCCLK/RTC_PR = (32.768KHz)/(32767+1)
			setPrescaler(Prescaler::LSE_PRESCALER);
			waitForLastTask();
			status = ClockSource::LSE;
		} else { // LSE - Fail

			// Wait till LSI is ready
			while(!Clock::isReadyClockSource(ClockSource::LSI)) {
				__NOP();
			}

			Clock::setRtcSource(ClockSource::LSE);
			waitForLastTask();

			// Enable RTC_t Clock
			Clock::enableRrealTimeClock();
			while(!Clock::isReadyRrealTimeClock()) {
				__NOP();
			}

			setPrescaler(Prescaler::LSI_PRESCALER);
			waitForLastTask();
			status = ClockSource::LSI;
		}
	}

	// Write RTC configure
	writeConfigBackupRegister(static_cast<uint16_t>(status));

	enableBackupWriteAccess();
}

/**
 * @bref Leap year
 * @retval true - leap year, false - no
 */
uint16_t Rtc::getLeapYear(uint16_t year) const
{
	return ((year % 4) ? 0 : 1);
}

/**
 * @bref Initialization interrupt
 * @param preemption - preemption priority
 * @param sub - sub priority
 */
void Rtc::createInterrupt(uint8_t preemption, uint8_t sub)
{
	create(RTC_IRQn, this, preemption, sub);
}

/**
 * @bref Reset clock alarm
 */
void Rtc::resetAlarm() const
{
	disableAlarm();
	disableBackupWriteAccess();
	waitForLastTask();
	enterConfigMode();

	// Reset alarm
	RTC->ALRH = 0;
	RTC->ALRL = 0;

	exitConfigMode();
	waitForLastTask();
	enableBackupWriteAccess();
}

/**
 * @bref Set alarm clock
 * @param [out] value - RTC alarm new value.
 */
void Rtc::setAlarm(time_t value) const
{
	disableBackupWriteAccess();
	enterConfigMode();

	// Set the ALARM MSB word
	RTC->ALRH = value >> 16;
	// Set the ALARM LSB word
	RTC->ALRL = (value & Mask::LSB);

	exitConfigMode();
	waitForLastTask();
	enableAlarm();
	waitForLastTask();
	enableBackupWriteAccess();
}

/** 
 * @brief Get Alarm time
 * @returns alarm time
*/
time_t Rtc::getAlarm() const
{
	time_t value = (RTC->ALRH << 16);
	value |= RTC->ALRL;
	return value;
}

/**
 * @bref Set alarm clock
 * @param rtc - time alarm
 */
void Rtc::setAlarm(const RTC_t* const rtc) const
{
	// converting date to sec
	const uint32_t value = dateToSec(rtc);

	disableBackupWriteAccess();
	enterConfigMode();

	// Set the ALARM MSB word
	RTC->ALRH = value >> 16;
	// Set the ALARM LSB word
	RTC->ALRL = (value & Mask::LSB);

	exitConfigMode();
	waitForLastTask();
	enableAlarm();
	waitForLastTask();
	enableBackupWriteAccess();
}

/**
 * @bref  Sets RTC_t current time and date
 * @param cnt - Time UNIX
 */
void Rtc::setTime(time_t cnt) const
{
	disableBackupWriteAccess();
	setCounter(cnt);
	enableBackupWriteAccess();
}

/**
 * @bref  Sets RTC_t current time and date
 * @param [in] rtc - Time and date in formate RTC_t
 * @retval None
 */
void Rtc::setTime(const RTC_t& rtc) const
{
	// converting date to sec
	const uint32_t cnt = dateToSec(&rtc);
	disableBackupWriteAccess();
	setCounter(cnt);
	enableBackupWriteAccess();
}

/**
 * @bref  Sets RTC_t current time and date
 * @param [in] rtc - Time and date in formate RTC_t
 * @retval None
 */
void Rtc::setTime(const RTC_t* const rtc) const
{
	// converting date to sec
	const uint32_t cnt = dateToSec(rtc);
	disableBackupWriteAccess();
	setCounter(cnt);
	enableBackupWriteAccess();
}

/**
 * @brief RTC return the Counter Value
 * @returns the 32 bit counter value
*/
time_t Rtc::getCounter() const
{
	time_t sec;
	do {
		const uint16_t cntl = RTC->CNTL;
		sec = (RTC->CNTH << 16) | cntl;
	} while(0 == sec);
	return sec;
}

/**
 * @brief RTC set the Counter
 * @param[in] counter 32 bit time setting for the counter
*/
void Rtc::setCounter(time_t counter) const
{
	enterConfigMode();
	waitForLastTask();

	RTC->CNTH = (counter & Mask::MSB) >> 16; // CNT[31:16]
	RTC->CNTL = (counter & Mask::LSB);       // CNT[15:0]

	exitConfigMode();
	waitForLastTask();
}

/**
 * @brief  Waits until last write operation on RTC registers has finished
 * @note   This function must be called before any write to RTC registers
 * @retval None
 */
void Rtc::waitForLastTask() const
{
	// Loop until RTOFF flag is set
	while((RTC->CRL & InterruptsFlag::RTOFF) == 0) {
	}
}

/**
 * @brief  Enters the RTC configuration mode
 * @retval None
 */
void Rtc::enterConfigMode() const
{
	// Set the CNF flag to enter in the Configuration Mode
	RTC->CRL |= InterruptsFlag::CNF;
}

/**
 * @brief  Exits from the RTC configuration mode
 * @retval None
 */
void Rtc::exitConfigMode() const
{
	// Reset the CNF flag to exit from the Configuration Mode
	RTC->CRL &= ~InterruptsFlag::CNF;
}

/**
 * @brief RTC Enable the Alarm
 * @retval None
 */
void Rtc::enableAlarm() const
{
	RTC->CRH |= InterruptsFlag::ALR;
}

/**
 * @brief RTC Disable the Alarm
 * @retval None
 */
void Rtc::disableAlarm() const
{
	RTC->CRH &= ~InterruptsFlag::ALR;
}

/**
 * @brief  Sets the RTC prescaler value
 * @param  PrescalerValue: RTC prescaler new value
 * @retval None
 */
void Rtc::setPrescaler(uint32_t prescaler) const
{
	enterConfigMode();

	// Set RTC PRESCALER LSB word */
	RTC->PRLL = (prescaler & Mask::LSB);
	// Set RTC PRESCALER MSB word
	RTC->PRLH = (prescaler & Mask::PRLH) >> 16;

	exitConfigMode();
}

/**
  * @brief  Checks whether the specified RTC interrupt alarm has occurred or not
  * @retval The new state of the RTC Alarm
  */
bool Rtc::getAlarmStatus()
{
	return (RTC->CRL & ALR);
}

/**
 * @brief  Clears the RTC's interrupt alarm pending bits
 * @retval None
 */
void Rtc::clearAlarmStatus()
{
	// Clear the corresponding RTC pending bit
	RTC->CRL &= ~InterruptsFlag::ALR;
}

/**
  * @brief  Enables access to the RTC and backup registers
  * @retval None
  */
void Rtc::enableBackupWriteAccess() const
{
	PWR->CR &= ~PWR_CR_DBP;
}

/**
  * @brief  Disable access to the RTC and backup registers
  * @retval None
  */
void Rtc::disableBackupWriteAccess() const
{
	PWR->CR |= PWR_CR_DBP;
}

/**
  * @brief Writes user data to the specified Data Backup Register
  * @retval None
  */
void Rtc::writeConfigBackupRegister(uint16_t data) const
{
	*(reinterpret_cast<uint32_t*>(RTC_BKP_REGISTER)) = data;
}

/**
  * @brief Reads data from the specified Data Backup Register
  * @retval value
  */
uint16_t Rtc::readConfigBackupRegister() const
{
	return *(reinterpret_cast<uint16_t*>(RTC_BKP_REGISTER));
}

/**
 * @bref Geting current time
 * @retval current time
 */
time_t Rtc::getTime() const
{
	return getCounter();
}

/**
 * @brief Get current time
 * @param [out] rtc - current time
 */
void Rtc::getTime(RTC_t* rtc) const
{
	const auto sec = getCounter();
	secToDate(rtc, sec);
}

/**
 * @brief Get current time
 * @param [out] rtc - Current time
 */
void Rtc::getTime(RTC_t& rtc) const
{
	const auto sec = getCounter();
	secToDate(&rtc, sec);
}

/**
 * @brief Converting date to sec
 * @param [in] rtc - date
 * @retval time
 */
time_t Rtc::dateToSec(const RTC_t& rtc) const
{
	return dateToSec(&rtc);
}

/**
 * @brief Converting date to sec
 * @param [in] rtc - date
 * @retval time
 */
time_t Rtc::dateToSec(const RTC_t* rtc) const
{
	const uint32_t year = rtc->year - Date::MILLENNIUM;

	uint32_t time = 0;
	if(year < 70) {
		time += SEC_MILLENNIUM; // Date later than 1 January 2000
		for(uint32_t tmp = 0; tmp < year; tmp++) {
			time += SEC_DAY * (ONE_YEAR + getLeapYear(tmp));
		}
	} else {
		for(uint32_t tmp = 70; tmp < year; tmp++) {
			time += SEC_DAY * (ONE_YEAR + getLeapYear(tmp));
		};
	}

	for(uint32_t tmp = 0; tmp < rtc->month; tmp++) {
		time +=
		    SEC_DAY * (getMonLen(tmp) + ((tmp == 2) ? getLeapYear(year) : 0));
	};

	time += SEC_DAY * (rtc->mday - 1);
	time += SEC_HOUR * rtc->hour;
	time += SEC_MIN * rtc->min;
	time += rtc->sec;

	return time;
}

/**
 * @brief Convert sec to date
 * @param [out] rtc - date
 * @param [in] sec - time
 */
void Rtc::secToDate(RTC_t* rtc, time_t sec) const
{
	if(sec >= SEC_MILLENNIUM) {
		sec -= SEC_MILLENNIUM; // // Date later than 1 January 2000
		rtc->year = 0;
	} else {
		rtc->year = 70; // Date from 1970 to 1999
	}

	uint32_t temp = 0;
	for(temp = ONE_YEAR;
	    sec >= (temp = SEC_DAY * (ONE_YEAR + getLeapYear(rtc->year)));
	    sec -= temp, rtc->year++) {
	};

	for(rtc->month = 1;
	    sec >=
	    (temp = SEC_DAY * (getMonLen(rtc->month) +
	                       ((2 == rtc->month) ? getLeapYear(rtc->year) : 0)));
	    rtc->month++) {
		sec -= temp;
	};

	rtc->year += MILLENNIUM;
	rtc->mday = sec / (SEC_DAY) + 1;
	sec %= SEC_DAY;
	rtc->hour = sec / SEC_HOUR;
	sec %= SEC_HOUR;
	rtc->min = sec / SEC_MIN;
	sec %= SEC_MIN;
	rtc->sec = sec;
	rtc->wday = computeWeekDayNum(rtc->year, rtc->month, rtc->mday);
}

/**
 * @brief Determines the week number, the day number and the week day number
 * @param [in] year - year to check
 * @param [in] month - month to check
 * @param [in] day - day to check
 * @note   Day is calculated with hypothesis that year > 2000
 * @retval Week day (0 - SUNDAY)
 */
uint8_t Rtc::computeWeekDayNum(uint32_t year, uint8_t month, uint8_t day) const
{
	if(month < 3) {
		return (((23 * month) / 9) + day + 4 + year + ((year - 1) / 4) -
		        ((year - 1) / 100) + ((year - 1) / 400)) %
		       7;
	}

	return (((23 * month) / 9) + day + 4 + year + (year / 4) - (year / 100) +
	        (year / 400) - 2) %
	       7;
}

/**
 * @brief Month length
 * @retval number of days per month
 */
constexpr uint8_t Rtc::getMonLen(size_t index)
{
	constexpr uint8_t monLen[] = {
	    0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	return monLen[index];
}

/**
 * @bref Interrupt handler
 */
void Rtc::handler()
{
	if(getAlarmStatus()) {
		// Call callbacks function
		notify();
		// Clear the RTC_t Second interrupt
		clearAlarmStatus();
	}
}

} // namespace stm32f10x_driver_lib
