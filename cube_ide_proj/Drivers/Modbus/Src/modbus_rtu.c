#include "modbus_rtu.h"

/* modbus protocol error checking */
uint16_t MB_CRC16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xffffU;
    size_t i;
    unsigned bit;
    for (i = 0; i < length; ++i) {
        crc ^= data[i];
        for (bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc >> 1) ^ ((crc & 1U) ? 0xa001U : 0U));
    }
    return crc;
}

/* this is the important one. Initializes a bus tied to USART1 or USART2.
   the wind sensor and solar irradiance sensor needs to use the same baud rate.
   whichever USART we pass here needs to be set to that baud rate. */
MB_Status MB_Init(MB_Bus *bus, UART_HandleTypeDef *uart,
                  MB_SetTransmit direction, void *context, uint32_t timeout_ms)
{
    if (!bus || !uart || !uart->Init.BaudRate || !timeout_ms ||
        timeout_ms == HAL_MAX_DELAY) return MB_INVALID_ARGUMENT;
    bus->uart = uart;
    bus->set_transmit = direction;
    bus->context = context;
    bus->timeout_ms = timeout_ms;
    /* 3.5-character silence, rounded up, including tick phase.
     * 12 bits/character covers parity and two stop bits.
     * Basically modbus protocol magic */
    bus->gap_ms = uart->Init.BaudRate > 19200U ? 3U :
                  (42000U + uart->Init.BaudRate - 1U) / uart->Init.BaudRate + 1U;
    bus->exception_code = 0;
    if (direction) direction(context, 0);
    return MB_OK;
}

static MB_Status receive(MB_Bus *bus, uint8_t *data, uint16_t size, uint32_t start)
{
    uint32_t elapsed = HAL_GetTick() - start;
    HAL_StatusTypeDef status;
    if (elapsed >= bus->timeout_ms) return MB_TIMEOUT;
    status = HAL_UART_Receive(bus->uart, data, size, bus->timeout_ms - elapsed);
    return status == HAL_OK ? MB_OK : status == HAL_TIMEOUT ? MB_TIMEOUT : MB_IO_ERROR;
}

/* This reads the holding registers. Pass in a pointer to the ModBus, device address
   the starting register address to read, no. of 16-bit registers to read, and a pointer
   to an array that will store the values */
MB_Status MB_ReadHoldingRegisters(MB_Bus *bus, uint8_t address,
                                 uint16_t first, uint16_t count, uint16_t *values)
{
    uint8_t tx[8], rx[37], discard;
    uint16_t crc, length, i;
    uint32_t start;
    MB_Status result;
    HAL_StatusTypeDef hal;
    if (bus) bus->exception_code = 0;
    if (!bus || !bus->uart || !values || address < 1U || address > 247U ||
        count < 1U || count > 16U || (uint32_t)first + count > 65536U ||
        !bus->timeout_ms || bus->timeout_ms == HAL_MAX_DELAY || !bus->gap_ms)
        return MB_INVALID_ARGUMENT;

    /* This clears the UART recieve buffer in case there is old data. If a broken device
       continuously sends data, then the read function just times out. rip */
    start = HAL_GetTick();
    do {
        hal = HAL_UART_Receive(bus->uart, &discard, 1, bus->gap_ms);
        if (hal == HAL_TIMEOUT) break;
        if (hal != HAL_OK) return MB_IO_ERROR;
        if (HAL_GetTick() - start >= bus->timeout_ms) return MB_TIMEOUT;
    } while (1);

    tx[0] = address; tx[1] = 3;
    tx[2] = (uint8_t)(first >> 8); tx[3] = (uint8_t)first;
    tx[4] = (uint8_t)(count >> 8); tx[5] = (uint8_t)count;
    crc = MB_CRC16(tx, 6);
    tx[6] = (uint8_t)crc; tx[7] = (uint8_t)(crc >> 8);

    /* if using hardware_DE, the set_transmit should be set to null in the MB_Init() call
       since uart just handles it automatically */
    if (bus->set_transmit) bus->set_transmit(bus->context, 1);
    hal = HAL_UART_Transmit(bus->uart, tx, sizeof(tx), bus->timeout_ms);
    if (bus->set_transmit) bus->set_transmit(bus->context, 0);
    if (hal != HAL_OK) return hal == HAL_TIMEOUT ? MB_TIMEOUT : MB_IO_ERROR;

    start = HAL_GetTick();
    result = receive(bus, rx, 3, start);
    if (result != MB_OK) return result;
    if (rx[0] != address) return MB_PROTOCOL_ERROR;
    if (rx[1] == (3U | 0x80U)) length = 5;
    else if (rx[1] == 3U && rx[2] == count * 2U) length = (uint16_t)(5U + count * 2U);
    else return MB_PROTOCOL_ERROR;
    result = receive(bus, rx + 3, (uint16_t)(length - 3U), start);
    if (result != MB_OK) return result;
    crc = MB_CRC16(rx, length - 2U);
    if (rx[length - 2U] != (uint8_t)crc || rx[length - 1U] != (uint8_t)(crc >> 8))
        return MB_CRC_ERROR;
    if (rx[1] & 0x80U) {
        bus->exception_code = rx[2];
        return MB_EXCEPTION;
    }
    for (i = 0; i < count; ++i)
        values[i] = (uint16_t)(((uint16_t)rx[3U + 2U*i] << 8) | rx[4U + 2U*i]);
    return MB_OK;
}
