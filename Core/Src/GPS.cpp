//File Name: gps-driver.cpp
//Description: This implements the definition of the gps driver

//Include Files
#include <string.h>
#include "../Inc/GPS.hpp"

//Private Constants
#define RX_MESSAGE_MAX_SIZE 100
//Private Variables
static USART_TypeDef* uartInstance;
static char ping[RX_MESSAGE_MAX_SIZE];
static char pong[RX_MESSAGE_MAX_SIZE];
static uint8_t messageStart = 0;
static uint16_t rxMessageIndex = 0;
//Public Constants

//Public Variables

//Private Function Prototypes
//Public Function Prototypes

//Private Function Definitions

//Public Function Definitions
void GPS_init(USART_TypeDef* uart_instance)
{
	uartInstance = uart_instance;
	//GPRMC only
	const char options[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
	for(uint16_t i = 0; i < sizeof(options); i++)
	{
		while(!(uartInstance->ISR & USART_ISR_TXE));
		uartInstance->TDR = options[i];
	}
	memset(ping,0, RX_MESSAGE_MAX_SIZE);
}

void GPS_startReception(void)
{
	//Enable interrupts
	uartInstance->CR1 |= USART_CR1_RXNEIE;
}

char* GPS_RxCpltCallback(bool* success)
{
	*success = false;

	char rxChar = (char)uartInstance->RDR;
	if(rxChar == '$')
	{
		messageStart = 1;
		rxMessageIndex = 0;
	}
	if(messageStart)
	{
		ping[rxMessageIndex++] = rxChar;
	}
	if(rxChar == '\n')
	{
		ping[rxMessageIndex] = '\0';
		messageStart = 0;
		*success = true;
	}
	return ping;
}
