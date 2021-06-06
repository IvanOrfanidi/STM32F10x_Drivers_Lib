/**
 * @brief   This file provides all the Interrupt firmware functions.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INTERRUPT_H
#define __INTERRUPT_H

/* Includes ------------------------------------------------------------------*/
/* Standart lib */
#include <stdint.h>

/* Driver MCU */
#include <CMSIS/Include/stm32f10x.h>

/* Utils */
#include <include/utils/non_copyable.h>
#include <include/utils/non_movable.h>

#ifdef __cplusplus

/* C++ lib */
#include <functional>

namespace stm32f10x_driver_lib {

// Interrupt Observer
class InterruptObserver
{
  public:
	InterruptObserver() = default;
	virtual ~InterruptObserver() = default;

	// Initialization interrupt
	virtual void createInterrupt(uint8_t, uint8_t) = 0;
	// Interrupt handler
	virtual void handler() = 0;
};

class Interrupt
  : public NonCopyable<Interrupt>
  , public NonMovable<Interrupt>
{
  public:
	enum Default {
		PREEMPTION_PRIORITY = 0,
		SUB_PRIORITY = 0,
	};

	enum NumberOfInterrupts : size_t {
#ifdef STM32F10X_LD
		NVIC_MAX_COUNT = USBWakeUp_IRQn
#endif
#ifdef STM32F10X_LD_VL
		    NVIC_MAX_COUNT = TIM7_IRQn
#endif
#ifdef STM32F10X_MD
		        NVIC_MAX_COUNT = USBWakeUp_IRQn
#endif
#ifdef STM32F10X_MD_VL
		            NVIC_MAX_COUNT = TIM7_IRQn
#endif
#ifdef STM32F10X_HD
		                NVIC_MAX_COUNT = DMA2_Channel4_5_IRQn
#endif
#ifdef STM32F10X_HD_VL
		                    NVIC_MAX_COUNT = DMA2_Channel5_IRQn
#endif
#ifdef STM32F10X_XL
		                        NVIC_MAX_COUNT = DMA2_Channel4_5_IRQn
#endif
#ifdef STM32F10X_CL
		                            NVIC_MAX_COUNT = OTG_FS_IRQn
#endif
	};

	/**
    @code  
     The table below gives the allowed values of the pre-emption priority and subpriority according
     to the Priority Grouping configuration performed by NVIC_PriorityGroupConfig function
      ==========================================================================
      Priority Group   | Preemption Priority | Sub Priority  | Description
      ==========================================================================
       _0              |         0           |      0-15     |   0 bits for pre-emption priority
                       |                     |               |   4 bits for subpriority
      --------------------------------------------------------------------------
       _1              |         0-1         |      0-7      |   1 bits for pre-emption priority
                       |                     |               |   3 bits for subpriority
      --------------------------------------------------------------------------   
       _2              |        0-3          |       0-3     |   2 bits for pre-emption priority
                       |                     |               |   2 bits for subpriority
      --------------------------------------------------------------------------    
       _3              |        0-7          |       0-1     |   3 bits for pre-emption priority
                       |                     |               |   1 bits for subpriority
      --------------------------------------------------------------------------    
       _4              |       0-15          |       0       |   4 bits for pre-emption priority
                       |                     |               |   0 bits for subpriority                       
      ==========================================================================
    @endcode
    */

	// Preemption Priority Group
	enum class PriorityGroup : uint32_t {
		_0 = 0x0700,
		_1 = 0x0600,
		_2 = 0x0500,
		_3 = 0x0400,
		_4 = 0x0300
	};

	Interrupt() = default; //Constructor

	// Constructor
	explicit Interrupt(size_t,
	                   InterruptObserver* const,
	                   uint32_t preemption = PREEMPTION_PRIORITY,
	                   uint32_t sub = SUB_PRIORITY);

	// Create interrupt
	void create(size_t,
	            InterruptObserver* const,
	            uint32_t preemption = PREEMPTION_PRIORITY,
	            uint32_t sub = SUB_PRIORITY);

	// Distructor (disable interrupt)
	~Interrupt();

	// Interrupt handler
	static void handler(size_t);

	// Sets the vector table location and Offset.
	static void setVectorTable(uint32_t, uint32_t);

	// Configures the priority grouping
	static void setPriorityGroup(PriorityGroup);

	// Check interrupt status enable
	static bool isEnabledGlobally();

	// Enable interrupts
	static void enableGlobally();

	// Disable interrupts
	static void disableGlobally();

	// Enable interrupt
	void enable() const;

	// Disable interrupt
	void disable() const;

  private:
	size_t _channel; //< Interrupt channel

	// Interrupt handler callback
	static class InterruptObserver* _callback[NVIC_MAX_COUNT];
};

} // namespace stm32f10x_driver_lib

extern "C" {
/* ALL INTERRUPT */
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void SPI1_IRQHandler(void);
void SPI2_IRQHandler(void);
void RTC_IRQHandler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

#ifdef STM32F10X_HD
void SPI3_IRQHandler(void);
#endif
};

#endif // __cplusplus

#endif // __INTERRUPT_H
