/**
 * @brief   This is file realise virtual port.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VIRTUAL_PORT_H
#define __VIRTUAL_PORT_H

/* Includes ------------------------------------------------------------------*/
/* Standart lib */
#include <stdio.h>

#ifdef __cplusplus

class VirtualPort
{
  public:
	enum class Type { UART, SPI, I2C };

	VirtualPort() = default;
	virtual ~VirtualPort() = default;

	// Get type virtual port
	virtual Type getType() = 0;

	// Transmits single data
	virtual void transmit(const void*, size_t) = 0;

	// Returns the most recent received data
	virtual size_t receive(void*, size_t) = 0;

	virtual void cleanTransmit() = 0;

	virtual void cleanReceive() = 0;

	virtual size_t getLength() = 0;

	// Waiting complete transfer
	virtual void waitingCompleteTransfer() = 0;

	virtual bool isEmpty() = 0;

	virtual void enableTransmit() = 0;

	virtual void disableTransmit() = 0;

	virtual void enableReceive() = 0;

	virtual void disableReceive() = 0;
};

extern "C" {
}
#endif // __cplusplus

#endif // __VIRTUAL_PORT_H
