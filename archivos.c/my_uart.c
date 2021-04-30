#include "my_uart.h" 
#include "string.h" 

static bool_t interruptUartFlag= FALSE;


// function to transmit the data
void sendMsg(UART_HandleTypeDef *huart, uint8_t * bufferTx)
{
	HAL_UART_Transmit_IT(huart,bufferTx,strlen((char*)bufferTx));
}

// function to read the uart interrupt flag form outside the my_uart.c
bool_t getUartFlag(void)
{
	bool_t uartFlag;
	
	uartFlag=interruptUartFlag;
	
	return uartFlag;
}

// function to erase the uart interrupt flag form outside the my_uart.c
void clrUartFlag(void)
{	
	interruptUartFlag=FALSE;	
}
 

// reception of the interrupt callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		interruptUartFlag= TRUE; // Si hubo interrupcion de uart activo flag
}	

// no  blocking function to attend the interrupt
void UART_Rx_Interrupt(UART_HandleTypeDef *huart, uartType_t * buffer)
{
		static uint8_t cont; //character counter

		if(buffer->uartMsg[cont]!='\n')
		{
			cont++;
			buffer->uartFlag=INCOMPLETE;
		}
		else
		{
			buffer->uartFlag=COMPLETE;
			buffer->uartLength=cont;
			cont=0;
		}
		if(cont>5) cont=0;
	  HAL_UART_Receive_IT(huart,&buffer->uartMsg[cont],1);
}

// function to validate the header BTHEADER
uint8_t	validateMsj(uint8_t  *msj)
{
	char msjHeader[]=BTHEADER;
	uint8_t len=0;
	uint8_t auxVar=0;

	
	len=strlen(msjHeader);
	while(auxVar<=(len-1))
	{
		if((char)msj[auxVar]!=msjHeader[auxVar])
			return 0;
		else
			auxVar++;
	}
	
	return msj[auxVar];
	
}







