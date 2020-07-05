/**
 * @brief   This file Software Timer.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SOFT_TIMER_H
#define __SOFT_TIMER_H

/* Includes ------------------------------------------------------------------*/
/* Standart lib */
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus

/* C++ lib */
#include <functional>

class SoftTimer
{
  public:
	explicit SoftTimer()
	  : _getClock(nullptr)
	  , _count(0)
	{
	}

	/**
     * @brief Constructor
     * @param [in] callback - function for get clock
     */
	explicit SoftTimer(std::function<time_t()> callback)
	  : _getClock(callback)
	{
		stop();
	}

	~SoftTimer() = default;

	/**
     * @brief Constructor
     * @param [in] value - time timer 
     * @param [in] callback - function for get clock
     */
	explicit SoftTimer(time_t value, std::function<time_t()> callback)
	  : _getClock(callback)
	{
		_count = _getClock() + value;
	};

	/**
    * @brief Stopped timer
    * @retval true - timer is stop
    */
	bool isStopped() const
	{
		return (0 == _count);
	};

	/**
    * @brief Started timer
    * @retval true - timer is starting
    */
	bool isStarted() const
	{
		return (!isStopped());
	};

	/**
     * @brief Start timer
     * @param [in] value - time timer 
     */
	void start(time_t value)
	{
		_count = _getClock() + value;
	};

	// Stop timer
	void stop()
	{
		_count = 0;
	};

	/**
    * @brief Match timer
    * @retval true - match timer 
    */
	bool isMatch() const
	{
		if(isStopped()) {
			return false;
		}
		return (_count <= _getClock());
	};

	/**
    * @brief Get over time timer
    * @retval over time
    */
	time_t overTime() const
	{
		return (_getClock() - _count);
	};

	/**
    * @brief Remaining time timer
    * @retval remaining time
    */
	time_t remainingTime() const
	{
		if(isStopped()) {
			return 0;
		}
		return (_count - _getClock());
	};

  private:
	std::function<time_t()> _getClock; //< Callback handler function

	volatile time_t _count; //< Counter
};

extern "C" {
}
#endif // __cplusplus

#endif // __SOFT_TIMER_H
