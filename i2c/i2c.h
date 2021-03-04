#define TRANSACTION_TIMEOUT_ms 1000
#define QUEUE_LENGTH 8192
#define I2C_ADDRESS_NOT_SHIFT

extern QueueHandle_t I2C_Queue;

uint8_t I2C_Transaction(I2C_TypeDef *I2C, uint8_t address, uint16_t rx_length, QueueHandle_t queue);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void I2C_Init_Once(void);
