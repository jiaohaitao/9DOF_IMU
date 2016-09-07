/***************************************************

*文件描述:
*         根据官方DMP库用MPU9250解姿态
          输出为Yaw,pitch,Roll值（角度值）
					参考网上部分代码
*Author:
*         王馨 @ UESTC
*Time:
*					2015.3.30
*version:
*         v1.0
***************************************************/
 
#include "stm32f10x.h"
#include "delay.h"
#include "math.h"
#include "usart.h"
#include "my_inv_mpu.h"


#define  led_on    GPIO_ResetBits(GPIOC, GPIO_Pin_13)
#define  led_off   GPIO_SetBits(GPIOC, GPIO_Pin_13)
    
//指示灯配置
void LED_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);    
    
    
    led_off;
    
}
void Data_Send_Status(float rol,float pit,float yaw);
void Send_Data(int16_t ad1,int16_t ad2,int16_t ad3,int16_t ad4,int16_t ad5,int16_t ad6,int16_t ad7,int16_t ad8,int16_t ad9);
int main(void)
{  
//声明相关变量	
   
    u16 count=0;    	
	  int MPU_INIT_FLAG=0;
	  short acc[3],gyo[3];
		float roll,pitch,yaw;
    SystemInit();
    
    USART2_Config();  //串口初始化
  
    LED_Config();
    led_off;
		MPU_INIT_FLAG=my_mpu_init();
    while(1)
    {
			//----------------------------------------------------LED---------------------------------------
        count ++;          //灯亮灭代表正在运行状态
        if(count<1000)
        {
            led_on;  
            
        }
        else if(count<2000)
        {
            led_off;  
            
        }
        else if(count==2000)
        {
            count = 0; 
            
        }
			//-----------------------------------------------------MPU-----------------------------------------
			if(MPU_INIT_FLAG==0){
				//void my_read_imu(short *accel,short *gyro,float *Roll,float *Pitch,float *Yaw)
				if(my_read_imu(acc,gyo,&roll,&pitch,&yaw)==0){
					Data_Send_Status(roll,pitch,yaw);
					Send_Data(acc[0],acc[1],acc[2],gyo[0],gyo[1],gyo[2],0,0,0);
				}
			}        
    }//end while(1)

}
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
void Data_Send_Status(float rol,float pit,float yaw)
{
	u8 _cnt=0,data_to_send[100]={0};
	vs16 _temp;vs32 _temp2 = 0;	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
		
	
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	for(i=0;i<_cnt;i++)
	UsartSend(data_to_send[i]);
}


void Send_Data(int16_t ad1,int16_t ad2,int16_t ad3,int16_t ad4,int16_t ad5,int16_t ad6,int16_t ad7,int16_t ad8,int16_t ad9)
{
	unsigned char i=0;
	unsigned char _cnt=0,sum = 0;
//	unsigned int _temp;
	u8 data_to_send[50];

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	

	data_to_send[_cnt++]=BYTE1(ad1);
	data_to_send[_cnt++]=BYTE0(ad1);
	data_to_send[_cnt++]=BYTE1(ad2);
	data_to_send[_cnt++]=BYTE0(ad2);
	data_to_send[_cnt++]=BYTE1(ad3);
	data_to_send[_cnt++]=BYTE0(ad3);
	
	data_to_send[_cnt++]=BYTE1(ad4);
	data_to_send[_cnt++]=BYTE0(ad4);
	data_to_send[_cnt++]=BYTE1(ad5);
	data_to_send[_cnt++]=BYTE0(ad5);
	data_to_send[_cnt++]=BYTE1(ad6);
	data_to_send[_cnt++]=BYTE0(ad6);
	data_to_send[_cnt++]=BYTE1(ad7);
	data_to_send[_cnt++]=BYTE0(ad7);
	data_to_send[_cnt++]=BYTE1(ad8);
	data_to_send[_cnt++]=BYTE0(ad8);
	data_to_send[_cnt++]=BYTE1(ad9);
	data_to_send[_cnt++]=BYTE0(ad9);
	
	data_to_send[3] = _cnt-4;
	//oíD￡?é
	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	for(i=0;i<_cnt;i++){
		USART_SendData(USART1,data_to_send[i]);
		while( USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET );
	}
}
