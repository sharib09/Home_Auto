#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void uart_rx_init(void);
void uart_rx_start(void);
void uart2_smoketest_init(void);
void uart2_smoketest_start(void);

int uart_tx_write(const uint8_t *data, int len);

/* Global UART state flags (shared across tasks/ISRs) */
extern volatile uint8_t ReceivedByte;
extern volatile bool    uart_recieve_flag;

#ifdef __cplusplus
}
#endif
