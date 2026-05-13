#ifndef UART_JETSON_H
#define UART_JETSON_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern UART_HandleTypeDef huart_jetson;
extern DMA_HandleTypeDef  hdma_usart1_rx;
extern DMA_HandleTypeDef  hdma_usart1_tx;

/*
 * USART1 Jetson 통신 — SPI1 대체
 *
 * 핀:  PA9  (TX, AF7)  →  Jetson RX
 *      PA10 (RX, AF7)  ←  Jetson TX
 * 설정: 921600 8N1, no flow control
 * DMA:  RX = DMA2 Stream2 Ch4 (circular)
 *       TX = DMA2 Stream7 Ch4 (normal)
 *
 * RX 수신 전략:
 *   DMA circular 모드로 rx_buf 에 연속 수신.
 *   UART IDLE 인터럽트로 "Jetson 이 보내기 끝남" 감지.
 *   IDLE 콜백에서 DMA NDTR 읽어 head 위치 갱신 → 프로토콜 계층이 파싱.
 */

#define JETSON_UART_BAUDRATE     921600
#define JETSON_UART_RX_BUF_SIZE  512

void MX_USART1_Jetson_Init(void);

void uart_jetson_start_rx(void);

bool uart_jetson_transmit_dma(const uint8_t *data, uint16_t size);
bool uart_jetson_tx_ready(void);

/* IDLE ISR 에서 호출 — rx_idle_flag set + head 위치 snapshot */
void uart_jetson_idle_callback(void);

/* RX 버퍼 접근 (프로토콜 계층용) */
const uint8_t *uart_jetson_rx_buffer(void);
uint16_t       uart_jetson_rx_head(void);
void           uart_jetson_rx_consume(uint16_t new_tail);
uint16_t       uart_jetson_rx_tail(void);
uint16_t       uart_jetson_rx_available(void);

/* IDLE 플래그 — ISR 에서 set, 메인 루프에서 clear */
volatile bool *uart_jetson_idle_flag_ptr(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_JETSON_H */
