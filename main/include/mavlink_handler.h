#ifndef MAVLINK_HANDLER_H
#define MAVLINK_HANDLER_H

#include <stdint.h>

/**
 * @brief Initialize the Mavlink handler.
 */
void mavlink_handler_init(void);

/**
 * @brief Parse a single byte from the incoming stream.
 * 
 * @param byte The byte received from UART.
 */
void mavlink_handler_parse_byte(uint8_t byte);

#endif // MAVLINK_HANDLER_H
