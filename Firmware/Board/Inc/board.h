#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BOARD_WATCHDOG_TIMEOUT_MS 500U

/* Pin assignments for the Dexter STM32H562RGT6 controller PCB:
 * PA0 / PA1: UART4 TX / RX to MAX3485 DI / RO, AF8, 115200 baud, 8N1.
 * PA2: MAX3485 DE and /RE tied together (high = transmit).
 * PB5 / PB6: UART5 RX / TX, AF14, 115200 baud, 3.3 V serial console.
 * PC13: external normally closed emergency-stop loop to GND; open/high = stop.
 * All external logic must be 3.3 V compatible.
 */

/* Call after HAL_Init() and SystemClock_Config(), with interrupts enabled. */
bool board_init(void);
uint32_t board_millis(void);

/* One request and one exact-length reply. No retry: a failed command may have
 * reached the motor already. The application decides how to handle failures.
 * rx_len == 0 permits a broadcast with no reply. Maximum frame size is 64.
 * The entire operation is bounded by timeout_ms, limited to 10 ms here.
 * Configure ZDT Response=Receive so arrival notifications cannot mix with replies.
 */
bool board_motor_exchange(const uint8_t *tx, size_t tx_len,
                          uint8_t *rx, size_t rx_len, uint32_t timeout_ms);

/* Console RX/TX use interrupt rings so motor polling does not lose commands.
 * RX corruption/overflow returns ASCII CAN (0x18): discard the current command
 * through its next newline. TX is best effort; a full queue drops a whole write.
 */
bool board_console_read(char *ch);
void board_console_write(const char *text);

bool board_estop_active(void);
/* Start immediately after HAL_Init(), before configuring clocks/peripherals.
 * Timeout is nominal: the independent LSI oscillator has frequency tolerance.
 */
bool board_watchdog_init(void);
bool board_watchdog_was_reset(void);
/* Call only after a complete main-loop update. Never feed from an ISR/wait. */
void board_watchdog_refresh(void);

#endif
