#include "stm32f401_conf.h"

static SemaphoreHandle_t _semaphore;
static SemaphoreHandle_t _mutex;
static volatile uint16_t _rx_length;
static volatile uint8_t _err;
static _Bool _init = pdFALSE;
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
    USART_Queue = xQueueCreate(USART_QUEUE_LENGTH, sizeof(uint8_t));
}

static void USART_NVIC_Enable(void) {
    NVIC_SetPriority(USART2_IRQn, 12);
    //NVIC_EnableIRQ(USART2_IRQn);
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

static void USART_Init(USART_TypeDef *USART) {
    USART_PinReinit();

    taskENTER_CRITICAL();
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    taskEXIT_CRITICAL();

    /* CONFIG FOR ASYNC MODE */
    LL_USART_ConfigAsyncMode(USART);
    /* FULL DUPLEX REGIMES */
    LL_USART_SetTransferDirection(USART, LL_USART_DIRECTION_TX);
    /* START + 8B + STOP, NO PARITY */
    LL_USART_ConfigCharacter(USART, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
    /* USART BAUDRATE */
    LL_USART_SetBaudRate(USART, 42000000, LL_USART_OVERSAMPLING_16, 115200);

    USART_NVIC_Enable();
    LL_USART_Enable(USART);

    LL_USART_EnableIT_IDLE(USART);
    LL_USART_EnableIT_RXNE(USART);
}

static void USART_Deinit(void) {
    USART2->CR1 = 0;
    taskENTER_CRITICAL();
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART2);
    taskEXIT_CRITICAL();

    _init = pdFALSE;
}

uint8_t USART_Transaction(USART_TypeDef *USART, QueueHandle_t queue) {
    USART_Acquire();
    _err = pdFALSE;

    if(_init == pdFALSE) {
        USART_Init(USART);
        _init = pdTRUE;
    }

    LL_USART_EnableIT_TXE(USART);
    LL_USART_EnableIT_TC(USART);
    NVIC_EnableIRQ(USART2_IRQn);
    LL_USART_Enable(USART2);

    if(xSemaphoreTake(_semaphore, pdMS_TO_TICKS(USART_TRANSACTION_TIMEOUT_ms))) {
        //USART_Deinit();
        USART_Release();
        if(_err) {
            return pdFALSE;
        }
    } else {
        //USART_Deinit();
        USART_Release();
        return pdFALSE;
    }

    return pdTRUE;
}

void USART2_IRQHandler(void) {
    static uint8_t _buf;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(LL_USART_IsActiveFlag_TXE(USART2)) {
        if(xQueueReceiveFromISR(USART_Queue, &_buf, &xHigherPriorityTaskWoken) != errQUEUE_EMPTY) {
            LL_USART_TransmitData8(USART2, _buf);
            return;
        } else if(LL_USART_IsActiveFlag_TC(USART2)) {
            LL_USART_DisableIT_TC(USART2);
            LL_USART_DisableIT_TXE(USART2);
            LL_USART_Disable(USART2);
            xSemaphoreGiveFromISR(_semaphore, &xHigherPriorityTaskWoken);
            if(xHigherPriorityTaskWoken == pdTRUE) {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}
