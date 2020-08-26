/**
 * @brief   This is file realise Observer.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OBSERVER_H
#define __OBSERVER_H

/* Includes ------------------------------------------------------------------*/
/* Standart lib */
#include <stdint.h>

/* Utils */
#include <inc/utils/lock_interrupt.h>

#ifdef __cplusplus

/* C++ lib*/
#include <functional>
#include <set>

// Subject Observer
class SubjectObserver
{
  public:
	SubjectObserver() = default;
	virtual ~SubjectObserver() = default;

	virtual void update() = 0;
};

// Observer
class Observer
{
  public:
	Observer() = default;
	virtual ~Observer() = default;

	// Notify subscribers and subjects
	void notify()
	{
		for(const auto& subject : _subjects) {
			subject->update();
		}
	}

	/**
     * @brief Adding subscriber in observer
     * @param [in] subject - callback subject
    */
	void attach(SubjectObserver* subject)
	{
		// Check interrupt status enable
		LockInterrupt lockInterrupt;
		_subjects.emplace(subject);
	}

	/**
     * @brief Detach subscriber in observer
     * @param [in] subject - callback subject
    */
	void detach(SubjectObserver* subject)
	{
		// Check interrupt status enable
		LockInterrupt lockInterrupt;
		_subjects.erase(subject);
	}

  private:
	std::set<class SubjectObserver*> _subjects; //< Subjects class
};

extern "C" {
}

#endif // __cplusplus

#endif // __OBSERVER_H
