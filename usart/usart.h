#define TRANSACTION_TIMEOUT_ms 1000
#define USART_QUEUE_LENGTH 1024

extern QueueHandle_t USART_Queue;

void USART2_IRQHandler(void);
void USART_InitOnce(void);
uint8_t USART_Transaction(USART_TypeDef *USART, QueueHandle_t queue);
