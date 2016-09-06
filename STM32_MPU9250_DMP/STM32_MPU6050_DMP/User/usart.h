#ifndef _USART_H_
#define _USART_H_




void USART2_Config(void);
void UsartSend(uint8_t ch);


void Print(uint8_t num);
void PrintInt(uint16_t num);
void PrintChar(char *s);
void PrintHexInt16(int16_t num);
void usart_INIT(void);



#endif

