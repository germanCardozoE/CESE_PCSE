#include "main.h"
#include "data_types.h"

typedef struct
{
	uint8_t 	uartLength;
	bool_t  	uartFlag;
	uint8_t		uartMsg[21];
} uartType_t;


//void sendMsg(UART_HandleTypeDef * ,uartType_t );
//void sendMsg(void);

//void sendMsg(UART_HandleTypeDef * );
void sendMsg(UART_HandleTypeDef *,uint8_t * );

void UART_Rx_Interrupt(UART_HandleTypeDef *,uartType_t * );

uint8_t	validateMsj(uint8_t *);

bool_t getUartFlag(void);

void clrUartFlag(void);

