#include "stm32f401_conf.h"

static SemaphoreHandle_t _semaphore;
static SemaphoreHandle_t _mutex;
static volatile uint16_t _rx_length;
static volatile uint8_t _err;
static volatile uint8_t _address;
QueueHandle_t USART_Queue;

static void USART_Acquire(void) {
    xSemaphoreTake(_mutex, portMAX_DELAY);
}

static void USART_Release(void) {
    xSemaphoreGive(_mutex);
}

void USART_InitOnce(void) {
    _semaphore = xSemaphoreCreateBinary();
    _mutex = xSemaphoreCreateMutex();
    USART_Queue = xQueueCreate(QUEUE_LENGTH, sizeof(uint8_t));
}

static void USART_PinReinit(void) {
    /* USART2 */
    GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_7);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
}

static void USART_Init(void) {
    USART_PinReinit();

    taskENTER_CRITICAL();
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    taskEXIT_CRITICAL();

    /* CONFIG FOR ASYNC MODE */
    LL_USART_ConfigAsyncMode(USART2);
    /* FULL DUPLEX REGIMES */
    LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);
    /* START + 8B + STOP, NO PARITY */
    LL_USART_ConfigCharacter(USART2, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
    /* USART BAUDRATE */
    LL_USART_SetBaudRate(USART2, 42000000, LL_USART_OVERSAMPLING_16, 115200);

    LL_USART_Enable(USART2);

    LL_USART_EnableIT_IDLE(USART2);
    LL_USART_EnableIT_RXNE(USART2);
    LL_USART_EnableIT_TXE(USART2);
    LL_USART_EnableIT_TC(USART2);

    NVIC_SetPriority(USART2_IRQn, 12);
    NVIC_EnableIRQ(USART2_IRQn);
}

static void USART_Deinit(void) {
    USART2->CR1 = 0;
    NVIC_DisableIRQ(USART2_IRQn);
    taskENTER_CRITICAL();
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART2);
    taskEXIT_CRITICAL();
}

uint8_t USART_Transaction(USART_TypeDef *USART, ) {
    static uint8_t _buf;
    USART_Acquire();
    USART_Init();

    xSemaphoreGive(_semaphore);
    _err = pdFALSE;

    if(xSemaphoreTake(_semaphore, pdMS_TO_TICKS(TRANSACTION_TIMEOUT_ms))) {
        USART_Deinit();
        USART_Release();
        if(_err) {
            return pdFALSE;
        }
    } else {
        USART_Deinit();
        USART_Release();
        return pdFALSE;
    }

    I2C_Release();
    return pdTRUE;

}

void USART2_IRQHandler(void) {



}

