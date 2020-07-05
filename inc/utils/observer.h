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

#ifdef __cplusplus

/* C++ lib*/
#include <functional>
#include <list>

// SubjectObserver
class SubjectObserver
{
  public:
	SubjectObserver() = default;
	~SubjectObserver() = default;

	virtual void update() = 0;
};

// ObServer
class Observer
{
  public:
	// Priority subscriber
	enum class Priority { LOW, HIGH };

	Observer() = default;
	~Observer() = default;

	// Notify subscribers and subjects
	void notify()
	{
		for(auto& subject : _subjects) {
			subject->update();
		}
	}

	/**
     * @brief Adding subscriber in observer
     * @param [in] subject - callback subject
     * @param [in] priority - priority(LOW or HIGH)
    */
	void attach(SubjectObserver* subject,
	            const Priority priority = Priority::LOW)
	{
		if(Priority::HIGH == priority) {
			_subjects.push_front(subject);
		} else {
			_subjects.push_back(subject);
		}
	}

  private:
	std::list<class SubjectObserver*> _subjects; //< Subjects class
};

extern "C" {
}

#endif //__cplusplus

#endif // __OBSERVER_H
