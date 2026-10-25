#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "stm32l4xx_hal.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MB_OK = 0, MB_INVALID_ARGUMENT, MB_TIMEOUT, MB_IO_ERROR,
    MB_CRC_ERROR, MB_PROTOCOL_ERROR, MB_EXCEPTION, MB_NOT_READY, MB_RANGE_ERROR
} MB_Status;

/* This points to a function that sets the DE and /RE pin*. That function should
   set a gpio to either '1' to enable transmission to or '0' to enable receive from
   the RS485 transceiver
 * can also be NULL if hardware DE is enabled. */
typedef void (*MB_SetTransmit)(void *context, uint8_t transmitting);

typedef struct {
    UART_HandleTypeDef *uart;
    MB_SetTransmit set_transmit;
    void *context;
    uint32_t timeout_ms;
    uint32_t gap_ms;
    uint8_t exception_code;
} MB_Bus;

/* UART must already be configured. One bus per physical UART. Task context
 * only; serialize the ENTIRE transaction with a common mutex if using an RTOS.
 * Do not use interrupt/DMA reception on this UART at the same time. */
MB_Status MB_Init(MB_Bus *bus, UART_HandleTypeDef *uart,
                  MB_SetTransmit direction, void *context, uint32_t timeout_ms);
uint16_t MB_CRC16(const uint8_t *data, size_t length);
/* Reads 1..16 standard holding registers; output changes only on success.
 * exception_code is reset on every call and set for a valid exception frame. */
MB_Status MB_ReadHoldingRegisters(MB_Bus *bus, uint8_t address,
                                 uint16_t first, uint16_t count, uint16_t *values);

#ifdef __cplusplus
}
#endif
#endif
