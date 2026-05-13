#include "uart_jetson.h"
#include "system_hal.h"   /* Error_Handler() */

/* === Peripheral handles === */
UART_HandleTypeDef huart_jetson;
DMA_HandleTypeDef  hdma_usart1_rx;
DMA_HandleTypeDef  hdma_usart1_tx;

/* === RX circular buffer + tracking === */
static uint8_t         rx_buf[JETSON_UART_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;   /* DMA 가 현재 쓰고 있는 위치 */
static uint16_t          rx_tail = 0;   /* 프로토콜 계층이 읽은 위치 */
static volatile bool     rx_idle = false;

/* === USART1 초기화: 921600 8N1, full-duplex, DMA === */
void MX_USART1_Jetson_Init(void)
{
    /* ---- GPIO: PB6 (TX), PB7 (RX) — USART1 alternate pins
     * PA9/PA10 는 Nucleo-F446RE 의 USB OTG VBUS/ID 회로와 충돌 (PA10 풀다운)
     * 으로 사용 불가. PB6/PB7 로 우회. (원래 I2C1 자리는 PB8/PB9 로 swap 완료) */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    GPIO_InitTypeDef gp = {0};
    gp.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gp.Mode      = GPIO_MODE_AF_PP;
    gp.Pull      = GPIO_NOPULL;
    gp.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gp.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &gp);

    /* ---- USART1 ---- */
    huart_jetson.Instance          = USART1;
    huart_jetson.Init.BaudRate     = JETSON_UART_BAUDRATE;
    huart_jetson.Init.WordLength   = UART_WORDLENGTH_8B;
    huart_jetson.Init.StopBits     = UART_STOPBITS_1;
    huart_jetson.Init.Parity       = UART_PARITY_NONE;
    huart_jetson.Init.Mode         = UART_MODE_TX_RX;
    huart_jetson.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart_jetson.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart_jetson) != HAL_OK) {
        Error_Handler();
    }

    /* ---- DMA2 Stream2 Ch4: USART1_RX (circular) ---- */
    hdma_usart1_rx.Instance                 = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_usart1_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(&huart_jetson, hdmarx, hdma_usart1_rx);

    /* ---- DMA2 Stream7 Ch4: USART1_TX (normal) ---- */
    hdma_usart1_tx.Instance                 = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_usart1_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(&huart_jetson, hdmatx, hdma_usart1_tx);

    /* ---- NVIC ---- */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/* === RX DMA circular 시작 + IDLE 인터럽트 활성화 === */
void uart_jetson_start_rx(void)
{
    rx_head = 0;
    rx_tail = 0;
    rx_idle = false;

    HAL_UART_Receive_DMA(&huart_jetson, rx_buf, JETSON_UART_RX_BUF_SIZE);

    /* IDLE 인터럽트 활성화 (HAL 이 자동으로 안 켜줌) */
    __HAL_UART_ENABLE_IT(&huart_jetson, UART_IT_IDLE);
}

/* === TX: feedback DMA 전송 ===
 * 주의: HAL_UART_GetState() 는 gState(TX) | RxState 의 OR 값을 반환.
 *       RX DMA circular 모드가 활성이면 RxState=BUSY_RX 라서
 *       HAL_UART_STATE_BUSY_TX 비트와 우연히 겹쳐 항상 busy 로 오판됨.
 *       반드시 gState 만 직접 확인해야 함.
 */
bool uart_jetson_transmit_dma(const uint8_t *data, uint16_t size)
{
    if (huart_jetson.gState != HAL_UART_STATE_READY) {
        return false;
    }
    return HAL_UART_Transmit_DMA(&huart_jetson, (uint8_t *)data, size) == HAL_OK;
}

bool uart_jetson_tx_ready(void)
{
    return huart_jetson.gState == HAL_UART_STATE_READY;
}

/* === IDLE 콜백 — USART1_IRQHandler 에서 호출 === */
void uart_jetson_idle_callback(void)
{
    uint16_t ndtr = __HAL_DMA_GET_COUNTER(huart_jetson.hdmarx);
    rx_head = JETSON_UART_RX_BUF_SIZE - ndtr;
    if (rx_head >= JETSON_UART_RX_BUF_SIZE) {
        rx_head = 0;
    }
    rx_idle = true;
}

/* === RX 버퍼 접근 함수들 === */
const uint8_t *uart_jetson_rx_buffer(void)   { return rx_buf; }
uint16_t       uart_jetson_rx_tail(void)     { return rx_tail; }

/* rx_head 는 IDLE ISR 에서도 갱신되지만, 신뢰성을 위해 항상 DMA NDTR 에서
 * 직접 계산. IDLE 인터럽트가 어떤 이유로 안 와도 데이터 위치 즉시 반영. */
uint16_t uart_jetson_rx_head(void)
{
    uint16_t ndtr = (uint16_t)__HAL_DMA_GET_COUNTER(huart_jetson.hdmarx);
    uint16_t h = JETSON_UART_RX_BUF_SIZE - ndtr;
    if (h >= JETSON_UART_RX_BUF_SIZE) h = 0;
    return h;
}

void uart_jetson_rx_consume(uint16_t new_tail)
{
    rx_tail = new_tail % JETSON_UART_RX_BUF_SIZE;
}

uint16_t uart_jetson_rx_available(void)
{
    uint16_t h = rx_head;
    uint16_t t = rx_tail;
    if (h >= t) return h - t;
    return JETSON_UART_RX_BUF_SIZE - t + h;
}

volatile bool *uart_jetson_idle_flag_ptr(void) { return &rx_idle; }
