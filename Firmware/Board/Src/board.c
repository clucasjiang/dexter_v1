#include "board.h"
#include "stm32h5xx_hal.h"

#define SERIAL_BAUD 115200U
#define MOTOR_FRAME_MAX 64U
#define MOTOR_TIMEOUT_MAX_MS 10U
#define BUS_QUIET_US 300U
#define CONSOLE_RX_SIZE 256U
#define CONSOLE_TX_SIZE 2048U
#define UART_ERRORS (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)
#define UART_CLEAR_ERRORS (USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF)

static volatile char console_rx[CONSOLE_RX_SIZE];
static volatile uint16_t console_rx_head;
static volatile uint16_t console_rx_tail;
static volatile bool console_rx_fault;
static volatile bool console_rx_discard;
static volatile char console_tx[CONSOLE_TX_SIZE];
static volatile uint16_t console_tx_head;
static volatile uint16_t console_tx_tail;
static uint32_t cycles_per_us;
static bool watchdog_reset;
static uint32_t watchdog_refresh_ms;

static bool expired(uint32_t start, uint32_t timeout_ms)
{
    return (uint32_t)(HAL_GetTick() - start) >= timeout_ms;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * cycles_per_us;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}

static bool uart_wait(USART_TypeDef *uart, uint32_t flag,
                      uint32_t start, uint32_t timeout_ms)
{
    while ((uart->ISR & flag) == 0U) {
        if ((uart->ISR & UART_ERRORS) != 0U || expired(start, timeout_ms)) {
            return false;
        }
    }
    return !expired(start, timeout_ms);
}

static bool uart_init(USART_TypeDef *uart, bool fifo)
{
    uint32_t start = HAL_GetTick();
    uart->CR1 = 0U;
    uart->CR2 = 0U;
    uart->CR3 = 0U;
    uart->PRESC = 0U;
    uart->BRR = (HAL_RCC_GetPCLK1Freq() + SERIAL_BAUD / 2U) / SERIAL_BAUD;
    uart->ICR = UART_CLEAR_ERRORS | USART_ICR_TCCF;
    uart->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE |
                (fifo ? USART_CR1_FIFOEN : 0U);
    return uart_wait(uart, USART_ISR_TEACK, start, 10U) &&
           uart_wait(uart, USART_ISR_REACK, start, 10U);
}

bool board_watchdog_init(void)
{
    uint32_t start = HAL_GetTick();
    watchdog_reset = (RCC->RSR & RCC_RSR_IWDGRSTF) != 0U;
    RCC->RSR |= RCC_RSR_RMVF;

    /* Starting IWDG also starts LSI. 500 counts at 32 kHz / 32 is about 500 ms.
     * I refresh it only from the main loop, never inside a wait or interrupt.
     * LSI tolerance means this is a recovery timer, not a precise motion timer.
     */
    IWDG->KR = 0xCCCCU;
    IWDG->KR = 0x5555U;
    IWDG->PR = 3U;
    IWDG->RLR = BOARD_WATCHDOG_TIMEOUT_MS * (LSI_VALUE / 1000U) / 32U - 1U;
    IWDG->EWCR = IWDG_EWCR_EWIC; /* Reset recovery, no early-wakeup interrupt. */
    while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU | IWDG_SR_EWU)) != 0U) {
        if (expired(start, 50U)) {
            return false;
        }
    }
    /* Changing WINR reloads the counter, as in ST's HAL initialization. */
    if (IWDG->WINR != IWDG_WINR_WIN) IWDG->WINR = IWDG_WINR_WIN;
    else IWDG->KR = 0xAAAAU;
    watchdog_refresh_ms = HAL_GetTick();
    return true;
}

bool board_watchdog_was_reset(void)
{
    return watchdog_reset;
}

bool board_init(void)
{
    GPIO_InitTypeDef pin = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();

    /* Both serial peripherals use PCLK1. This tracks the clock chosen in main. */
    RCC->CCIPR1 &= ~(RCC_CCIPR1_UART4SEL | RCC_CCIPR1_UART5SEL);

    /* Set the output latches before changing pin mode to avoid a DE glitch. */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    pin.Pin = GPIO_PIN_2;
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Pull = GPIO_NOPULL;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &pin);

    pin.Pin = GPIO_PIN_13;
    pin.Mode = GPIO_MODE_INPUT;
    pin.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &pin);

    pin.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    pin.Mode = GPIO_MODE_AF_PP;
    pin.Pull = GPIO_PULLUP;
    pin.Speed = GPIO_SPEED_FREQ_MEDIUM;
    pin.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &pin);
    pin.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    pin.Alternate = GPIO_AF14_UART5;
    HAL_GPIO_Init(GPIOB, &pin);

    /* DWT times RS485 direction turnaround and inter-frame quiet intervals. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cycles_per_us = SystemCoreClock / 1000000U;
    uint32_t cycle_check = DWT->CYCCNT;
    for (uint32_t i = 0U; i < 16U; ++i) {
        __NOP();
    }
    if (cycles_per_us == 0U || DWT->CYCCNT == cycle_check) {
        return false;
    }

    if (!uart_init(UART4, false) || !uart_init(UART5, true)) {
        return false;
    }
    console_rx_head = 0U;
    console_rx_tail = 0U;
    console_tx_head = 0U;
    console_tx_tail = 0U;
    console_rx_fault = false;
    console_rx_discard = false;
    UART5->CR3 |= USART_CR3_EIE;
    UART5->CR1 |= USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE;
    NVIC_SetPriority(UART5_IRQn, 5U);
    NVIC_EnableIRQ(UART5_IRQn);
    return true;
}

uint32_t board_millis(void)
{
    return HAL_GetTick();
}

bool board_motor_exchange(const uint8_t *tx, size_t tx_len,
                          uint8_t *rx, size_t rx_len, uint32_t timeout_ms)
{
    if (tx == NULL || tx_len == 0U || tx_len > MOTOR_FRAME_MAX ||
        rx_len > MOTOR_FRAME_MAX || (rx_len != 0U && rx == NULL) || timeout_ms == 0U) {
        return false;
    }
    if (timeout_ms > MOTOR_TIMEOUT_MAX_MS) {
        timeout_ms = MOTOR_TIMEOUT_MAX_MS;
    }
    uint32_t start = HAL_GetTick();
    uint32_t quiet_start = DWT->CYCCNT;
    uint32_t quiet_cycles = BUS_QUIET_US * cycles_per_us;

    /* Clear a previous partial/late frame and require a short quiet interval.
     * Even a noisy or continuously active bus cannot trap the main loop here.
     */
    UART4->ICR = UART_CLEAR_ERRORS;
    while ((uint32_t)(DWT->CYCCNT - quiet_start) < quiet_cycles) {
        if ((UART4->ISR & USART_ISR_RXNE_RXFNE) != 0U) {
            (void)UART4->RDR;
            quiet_start = DWT->CYCCNT;
        }
        if ((UART4->ISR & UART_ERRORS) != 0U) {
            UART4->ICR = UART_CLEAR_ERRORS;
            quiet_start = DWT->CYCCNT;
        }
        if (expired(start, timeout_ms)) {
            return false;
        }
    }

    GPIOA->BSRR = GPIO_PIN_2;
    delay_us(2U); /* Allow the MAX3485 driver to enable. */
    UART4->ICR = USART_ICR_TCCF;
    for (size_t i = 0U; i < tx_len; ++i) {
        if (!uart_wait(UART4, USART_ISR_TXE_TXFNF, start, timeout_ms)) {
            goto failed;
        }
        UART4->TDR = tx[i];
    }
    /* TXE means the data register is empty. TC means the final stop bit is out. */
    if (!uart_wait(UART4, USART_ISR_TC, start, timeout_ms)) {
        goto failed;
    }
    GPIOA->BSRR = (uint32_t)GPIO_PIN_2 << 16U;

    for (size_t i = 0U; i < rx_len; ++i) {
        if (!uart_wait(UART4, USART_ISR_RXNE_RXFNE, start, timeout_ms) ||
            (UART4->ISR & UART_ERRORS) != 0U) {
            goto failed;
        }
        rx[i] = (uint8_t)UART4->RDR;
    }

    /* A frame longer than expected is a protocol error, never a valid prefix.
     * The protocol layer checks address, function, length and check byte.
     */
    quiet_start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - quiet_start) < quiet_cycles) {
        if ((UART4->ISR & (USART_ISR_RXNE_RXFNE | UART_ERRORS)) != 0U ||
            expired(start, timeout_ms)) {
            goto failed;
        }
    }
    return true;

failed:
    GPIOA->BSRR = (uint32_t)GPIO_PIN_2 << 16U;
    UART4->ICR = UART_CLEAR_ERRORS;
    UART4->RQR = USART_RQR_RXFRQ;
    return false;
}

/* This overrides the weak handler in the startup file. Each ring has one writer
 * and one reader; only the short RX fault/reset operation needs a critical section.
 */
void UART5_IRQHandler(void)
{
    if ((UART5->ISR & UART_ERRORS) != 0U) {
        UART5->ICR = UART_CLEAR_ERRORS;
        UART5->RQR = USART_RQR_RXFRQ;
        console_rx_fault = true;
        console_rx_discard = true;
    }
    while ((UART5->ISR & USART_ISR_RXNE_RXFNE) != 0U) {
        char ch = (char)UART5->RDR;
        uint16_t next = (uint16_t)((console_rx_head + 1U) % CONSOLE_RX_SIZE);
        if (console_rx_discard) {
            if (ch != '\n' && ch != '\r') {
                continue;
            }
            console_rx_discard = false;
        }
        if (next == console_rx_tail) {
            console_rx_fault = true;
            console_rx_discard = true;
        } else {
            console_rx[console_rx_head] = ch;
            console_rx_head = next;
        }
    }
    while ((UART5->CR1 & USART_CR1_TXEIE_TXFNFIE) != 0U &&
           (UART5->ISR & USART_ISR_TXE_TXFNF) != 0U) {
        if (console_tx_tail == console_tx_head) {
            UART5->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
            break;
        }
        UART5->TDR = (uint8_t)console_tx[console_tx_tail];
        console_tx_tail = (uint16_t)((console_tx_tail + 1U) % CONSOLE_TX_SIZE);
    }
}

bool board_console_read(char *ch)
{
    if (ch == NULL) {
        return false;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (console_rx_fault) {
        console_rx_tail = console_rx_head;
        console_rx_fault = false;
        *ch = 0x18; /* CAN: do not execute a damaged command. */
        __set_PRIMASK(primask);
        return true;
    }
    if (console_rx_tail == console_rx_head) {
        __set_PRIMASK(primask);
        return false;
    }
    *ch = console_rx[console_rx_tail];
    console_rx_tail = (uint16_t)((console_rx_tail + 1U) % CONSOLE_RX_SIZE);
    __set_PRIMASK(primask);
    return true;
}

void board_console_write(const char *text)
{
    if (text == NULL) {
        return;
    }
    size_t length = 0U;
    while (text[length] != '\0' && length < CONSOLE_TX_SIZE) {
        ++length;
    }
    uint16_t head = console_tx_head;
    size_t free_space = (console_tx_tail + CONSOLE_TX_SIZE - head - 1U) % CONSOLE_TX_SIZE;
    if (length > free_space) {
        return; /* Logging must never delay current feedback or motor monitoring. */
    }
    for (size_t i = 0U; i < length; ++i) {
        console_tx[head] = text[i];
        head = (uint16_t)((head + 1U) % CONSOLE_TX_SIZE);
    }
    console_tx_head = head;
    UART5->CR1 |= USART_CR1_TXEIE_TXFNFIE;
}

bool board_estop_active(void)
{
    /* No debounce delays on the stop path. A noisy/broken loop should stop us. */
    return (GPIOC->IDR & GPIO_PIN_13) != 0U;
}

void board_watchdog_refresh(void)
{
    uint32_t now = HAL_GetTick();
    if (now == watchdog_refresh_ms) return;
    watchdog_refresh_ms = now;
    IWDG->KR = 0xAAAAU;
}
