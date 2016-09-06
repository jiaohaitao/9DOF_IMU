#include "stm32f10x.h"
#include "usart.h"

 
//串口输出配置
void USART2_Config(void)
{
	
	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//  

//	/*" 第1步：打开GPIO和USART部件的时钟 "*/
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

//	/*" 第2步：将USART Tx的GPIO配置为推挽复用模式 "*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	/*" 第3步：将USART Rx的GPIO配置为浮空输入模式							 "
//	"	由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的	  "
//		但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
//	*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_Init(GPIOA, &GPIO_InitStructure);


// 
//	USART_InitStructure.USART_BaudRate = 115200;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_Parity = USART_Parity_No;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	USART_Init(USART2, &USART_InitStructure);
// 
//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
// 
//	USART_Cmd(USART2, ENABLE);

//	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
//		如下语句解决第1个字节无法正确发送出去的问题 */
//	USART_ClearFlag(USART2, USART_FLAG_TC);     /* 清发送外城标志，Transmission Complete flag */

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(USART1, ENABLE);
}

 

void UsartSend(uint8_t ch)
{
	USART_SendData(USART1, (uint8_t) ch);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

 
//以字符的格式输出
void Print(uint8_t num)
{
	uint8_t  bai,shi,ge;
	bai=num/100;
	shi=num%100/10;
	ge=num%10;
	UsartSend('0'+bai);
	UsartSend('0'+shi);
	UsartSend('0'+ge);
}


//以字符的形式输出INT型数据
void PrintInt(uint16_t num)
{
	 uint8_t w5,w4,w3,w2,w1;
	 w5=num/10000;
	 w4=num%10000/1000;
	 w3=num%1000/100;
	 w2=num%100/10;
	 w1=num%10;
	 UsartSend('0'+w5);
	 UsartSend('0'+w4);
	 UsartSend('0'+w3);
	 UsartSend('0'+w2);
	 UsartSend('0'+w1);
}


//输出字符串
void PrintChar(char *s)
{
	char *p;
	p=s;
	while(*p != '\0')
	{
		UsartSend(*p);
		p++;
	}
}


void PrintHexInt16(int16_t num)
{
	UsartSend((num & 0xff00) >> 8);//先发送高８位，再发送低８位
	UsartSend((uint8_t)(num & 0x00ff));
}


void usart_INIT(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* NVIC_PriorityGroup 1 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//串口
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}	


