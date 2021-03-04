#include "stm32f401_conf.h"

static SemaphoreHandle_t _semaphore;
static SemaphoreHandle_t _mutex;
static volatile uint16_t _rx_length;
static volatile uint8_t _err;
static volatile uint8_t _address;
QueueHandle_t I2C_Queue;

static void I2C_Acquire(void) {
    xSemaphoreTake(_mutex, portMAX_DELAY);
}

static void I2C_Release(void) {
    xSemaphoreGive(_mutex);
}

void I2C_InitOnce() {
    _semaphore = xSemaphoreCreateBinary();
    _mutex = xSemaphoreCreateMutex();
    I2C_Queue = xQueueCreate(QUEUE_LENGTH, sizeof(uint8_t));
}


static void I2C_Init(void) {
    GPIOB->MODER    &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);

    taskENTER_CRITICAL();
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
    taskEXIT_CRITICAL();

    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    LL_I2C_SetPeriphClock(I2C1, 42000000);
    LL_I2C_SetClockSpeedMode(I2C1, LL_I2C_CLOCK_SPEED_STANDARD_MODE);
    LL_I2C_SetClockPeriod(I2C1, 420);
    LL_I2C_SetRiseTime(I2C1, 43);
    LL_I2C_DisableBitPOS(I2C1);

    LL_I2C_EnableIT_EVT(I2C1);
    LL_I2C_EnableIT_ERR(I2C1);
    LL_I2C_EnableIT_RX(I2C1);
    LL_I2C_Enable(I2C1);

    NVIC_SetPriorityGrouping(0);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_SetPriority(I2C1_EV_IRQn, 11);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
    NVIC_SetPriority(I2C1_ER_IRQn, 11);
}

static void I2C_Deinit(void) {
    I2C1->CR1 = 0;
    NVIC_DisableIRQ(I2C1_EV_IRQn);
    NVIC_DisableIRQ(I2C1_ER_IRQn);
    taskENTER_CRITICAL();
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C1);
    taskEXIT_CRITICAL();
}

uint8_t I2C_Transaction(I2C_TypeDef *I2C, uint8_t address, uint16_t rx_length, QueueHandle_t queue) {
    I2C_Acquire();
    I2C_Init();
    _rx_length = rx_length;
    #ifdef I2C_ADDRESS_NOT_SHIFT
        _address = address;
    #else
        _address = address << 1;
    #endif

    xSemaphoreGive(_semaphore);
    _err = 0;

    if(_rx_length) {
        if(xSemaphoreTake(_semaphore, pdMS_TO_TICKS(TRANSACTION_TIMEOUT_ms)) == pdFALSE) {
            __BKPT(1);
            I2C_Release();
            return pdFALSE;
        }
        LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
        LL_I2C_GenerateStartCondition(I2C);
    } else {
        if(xSemaphoreTake(_semaphore, pdMS_TO_TICKS(TRANSACTION_TIMEOUT_ms)) == pdFALSE) {
            __BKPT(1);
            I2C_Release();
            return pdFALSE;
        }
        LL_I2C_EnableIT_TX(I2C);
        LL_I2C_GenerateStartCondition(I2C);
    }

    if(xSemaphoreTake(_semaphore, pdMS_TO_TICKS(TRANSACTION_TIMEOUT_ms))) {
        I2C_Deinit();
        I2C_Release();
        if(_err) {
            return pdFALSE;
        }

    } else {
        I2C_Deinit();
        I2C_Release();
        return pdFALSE;
    }

    I2C_Release();
    return pdTRUE;
}

void I2C1_EV_IRQHandler(void) {
    static uint8_t _buf;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(LL_I2C_IsActiveFlag_ADDR(I2C1)){
        LL_I2C_ClearFlag_ADDR(I2C1);
    }
    if(LL_I2C_IsActiveFlag_SB(I2C1)) {
        LL_I2C_ReadReg(I2C1, SR1);
        LL_I2C_ReadReg(I2C1, SR2);
        LL_I2C_TransmitData8(I2C1, _address);
        return;

    } else if(LL_I2C_IsActiveFlag_TXE(I2C1) && !(_rx_length)) {
        if(xQueueReceiveFromISR(I2C_Queue, &_buf, &xHigherPriorityTaskWoken) != errQUEUE_EMPTY) {
            LL_I2C_TransmitData8(I2C1, _buf);
            return;
        }
        LL_I2C_GenerateStopCondition(I2C1);

    } else if(LL_I2C_IsActiveFlag_RXNE(I2C1) && (_rx_length)) {
        _buf = LL_I2C_ReceiveData8(I2C1);
        xQueueSendFromISR(I2C_Queue, &_buf, &xHigherPriorityTaskWoken);
        if(_rx_length == 1) {
            LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
            LL_I2C_GenerateStopCondition(I2C1);
        }
        _rx_length--;
        return;

    }

    LL_I2C_ReadReg(I2C1, SR1);
    LL_I2C_ReadReg(I2C1, SR2);
    LL_I2C_ReadReg(I2C1, DR);

    if(!(_rx_length)){

        xSemaphoreGiveFromISR(_semaphore, &xHigherPriorityTaskWoken);
        if(xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void I2C1_ER_IRQHandler(void) {
    _err = pdTRUE;

    BaseType_t xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(_semaphore, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    I2C1->CR1 |= I2C_CR1_SWRST;

    //__BKPT(1);
}
