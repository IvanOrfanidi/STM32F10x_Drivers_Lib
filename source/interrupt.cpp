/**
 * @brief   This is file realize interface Interrupt.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <include/interrupt.h>

namespace stm32f10x_driver_lib {
/**
 * Main array pointers of classes Interrupt
 * NVIC_MAX_COUNT max number interrupts
 */
InterruptObserver* Interrupt::_callback[NVIC_MAX_COUNT];

/**
 * @bref Constructor interrupt
 * @param [in] channel - interrupt channel
 * @param [in] callback - callback interrupt handler
 * @param [in] preemption - preemption priority
 * @param [in] sub - sub priority
 */
Interrupt::Interrupt(size_t channel,
                     InterruptObserver* const callback,
                     uint32_t preemption,
                     uint32_t sub)
{
	create(channel, callback, preemption, sub);
}

/**
 * @bref Initialization interrupt
 * @param channel - channel number
 * @param callback - callback class function
 * @param preemption - preemption priority
 * @param sub - sub priority
 */
void Interrupt::create(size_t channel,
                       InterruptObserver* const callback,
                       uint32_t preemption,
                       uint32_t sub)
{
	_channel = channel;

	// Compute the Corresponding IRQ Priority
	uint32_t priority = (0x0700 - (SCB->AIRCR & 0x0700)) >> 0x08;
	const uint32_t pre_priority = 0x04 - priority;
	uint32_t sub_priority = 0x0F;
	sub_priority >>= priority;

	priority = preemption << pre_priority;
	priority |= sub & sub_priority;
	priority <<= 0x04;

	NVIC->IP[_channel] = priority;

	// Enable the Selected IRQ Channels
	NVIC->ISER[_channel >> 0x05] = 0x01 << (_channel & 0x1F);

	_callback[_channel] = callback;
}

/**
 * @bref Distructor interrupt
 */
Interrupt::~Interrupt()
{
	_callback[_channel] = nullptr;
	NVIC->ICER[_channel >> 0x05] = 0x01 << (_channel & 0x1F);
}

/**
  * @brief  Sets the vector table location and Offset.
  * @param  [in] vectTab: specifies if the vector table is in RAM or FLASH memory.
  * @param  [in] offset: Vector Table base offset field.
  *         This value must be a multiple of 0x200.
 */
void Interrupt::setVectorTable(uint32_t vectTab, uint32_t offset)
{
	SCB->VTOR = vectTab | (offset & 0x1FFFFF80);
}

/**
 * @bref Check interrupt status enable
 * retval true - if they are enabled, false - if they are disabled
 */
bool Interrupt::isEnabledGlobally()
{
	/* Read PRIMASK register, check interrupt status before you disable them
	   Returns 0 if they are enabled, or non-zero if disabled */
	const uint32_t primask = __get_PRIMASK();
	return (0 == primask);
}

/**
 * @bref Enable interrupts
 */
void Interrupt::enableGlobally()
{
	__enable_irq();
}

/**
 * @bref Disable interrupts
 */
void Interrupt::disableGlobally()
{
	__disable_irq();
	__DSB();
	__ISB();
}

/**
  * @brief  Configures the priority grouping: pre-emption priority and subpriority.
  * @param [in] priorityGroup: specifies the priority grouping bits length. 
  *   This parameter can be one of the following values:
  *     @arg _0: 0 bits for pre-emption priority 4 bits for subpriority
  *     @arg _1: 1 bits for pre-emption priority 3 bits for subpriority
  *     @arg _2: 2 bits for pre-emption priority 2 bits for subpriority
  *     @arg _3: 3 bits for pre-emption priority 1 bits for subpriority
  *     @arg _4: 4 bits for pre-emption priority 0 bits for subpriority
 */
void Interrupt::setPriorityGroup(PriorityGroup priorityGroup)
{
	static constexpr uint32_t AIRCR_VECTKEY_MASK = 0x05FA0000;
	SCB->AIRCR = AIRCR_VECTKEY_MASK | static_cast<uint32_t>(priorityGroup);
}

/**
 * @bref Enable interrupt
 */
void Interrupt::enable() const
{
	NVIC_EnableIRQ(static_cast<IRQn_Type>(_channel));
}

/**
 * @bref Disable interrupt
 */
void Interrupt::disable() const
{
	NVIC_DisableIRQ(static_cast<IRQn_Type>(_channel));
}

/**
 * @bref Interrupt handler
 * @param [in] channel - interrupt channel
 */
void Interrupt::handler(size_t channel)
{
	_callback[channel]->handler();
}

} // namespace stm32f10x_driver_lib

/**
 * @brief  This function handles USART1 global interrupt request.
 */
void USART1_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(USART1_IRQn);
}

/**
 * @brief  This function handles USART2 global interrupt request.
 */
void USART2_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(USART2_IRQn);
}

/**
 * @brief  This function handles USART3 global interrupt request.
 */
void USART3_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(USART3_IRQn);
}

/**
 * @brief  This function handles SPI1 global interrupt request.
 */
void SPI1_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(SPI1_IRQn);
}

/**
 * @brief  This function handles SPI2 global interrupt request.
 */
void SPI2_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(SPI2_IRQn);
}

/**
 * @brief  This function handles SPI3 global interrupt request.
 */
#ifdef STM32F10X_HD
void SPI3_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(SPI3_IRQn);
}
#endif
/**
  * @brief  This function handles RTC_t global interrupt request.
  */
void RTC_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(RTC_IRQn);
}

/* This C-function handles EXTIs global interrupt request */
void EXTI0_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(EXTI0_IRQn);
}

void EXTI1_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(EXTI1_IRQn);
}

void EXTI2_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(EXTI2_IRQn);
}

void EXTI3_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(EXTI3_IRQn);
}

void EXTI4_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(EXTI4_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(EXTI9_5_IRQn);
}

void EXTI15_10_IRQHandler(void)
{
	stm32f10x_driver_lib::Interrupt::handler(EXTI15_10_IRQn);
}
