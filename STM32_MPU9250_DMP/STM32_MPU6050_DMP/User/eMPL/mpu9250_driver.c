#include "mpu9250_driver.h"
#include "stm32f10x.h"
#include "stm32_mpu_iic.h"

short IMU_MAG_OFFSET[3]={0xffec,0xffef,0x0031};
//short IMU_MAG_OFFSET[3]={0xffe4,0x001d,0x008d};
void MPU9250_GetMag_Offset(short *magoffset)
{
	magoffset[0]=IMU_MAG_OFFSET[0];
	magoffset[1]=IMU_MAG_OFFSET[1];
	magoffset[2]=IMU_MAG_OFFSET[2];
}


//---------------------IIC--------------------------
//uint32_t t=1000000;

#define I2C_SCL_0() GPIOB->ODR &= ~(1 << 6)
#define I2C_SCL_1() GPIOB->ODR |= 1 << 6
#define I2C_SDA_0() GPIOB->ODR &= ~(1 << 7)
#define I2C_SDA_1() GPIOB->ODR |= 1 << 7
#define I2C_READ_SDA() GPIOB->IDR & (1 << 7)

#define HMC5883L_ADDRESS 0x3c



void Soft_Delay(uint32_t t)
{
	while(t --);
}

void Soft_I2C_Delay(void)
{
	uint32_t t = 0;			//72MHz,IAR????:low, 
								//t = 10:200 ~ 270KHz
								//t = 3:340 ~ 440KHz;
								//t = 2:350 ~ 480KHz;
								//t = 1:400 ~ 530KHz;
								//t = 0:440 ~ 610KHz;
	Soft_Delay(t);
}

void Soft_I2C_Start(void)
{
	I2C_SDA_1();
	I2C_SCL_1();
	Soft_I2C_Delay();		//1.5us,???????
	I2C_SDA_0();
	Soft_I2C_Delay();
	I2C_SDA_0();
	Soft_I2C_Delay();
}

void Soft_I2C_Stop(void)
{

	I2C_SCL_0();			
	Soft_I2C_Delay();
	I2C_SDA_0();
	Soft_I2C_Delay();
	I2C_SCL_1();
	Soft_I2C_Delay();
	I2C_SDA_1();
	Soft_I2C_Delay();
}
void Soft_I2C_Initial(void)
{
	RCC->APB2ENR |= 1 << 3;		//PB
	GPIOB->CRL &= 0x00ffffff;	//PB7 SDA,PB6 SCL
	GPIOB->CRL |= 0x73000000;	//PB7 OD,PB6 PP
	I2C_SDA_1();
	I2C_SCL_1();
}
void Soft_I2C_Write_Byte(uint8_t data)
{
	uint8_t i = 0;
	
	for(i = 0;i < 8;i ++)
	{
		I2C_SCL_0();
		Soft_I2C_Delay();
		
		if(data & 0x80)
			I2C_SDA_1();
		else
			I2C_SDA_0();
		
		data <<= 1;
		Soft_I2C_Delay();
		I2C_SCL_1();
		Soft_I2C_Delay();		
	}
	I2C_SCL_0();
}

uint8_t Soft_I2C_Read_Byte(void)
{
	uint8_t i = 0;
	uint8_t data = 0;
	
	I2C_SDA_1();
	
	for(i = 0;i < 8;i ++)
	{
		data <<= 1;
		
		I2C_SCL_0();
		Soft_I2C_Delay();
		I2C_SCL_1();
		Soft_I2C_Delay();
		
		if(I2C_READ_SDA())
			data |= 0x01;
		else
			;	
	}
	I2C_SCL_0();
	
	return data;
}

void Soft_I2C_Wait_ACK(void)
{

	I2C_SCL_0();
	Soft_I2C_Delay();
	I2C_SDA_1();				//??SDA
	Soft_I2C_Delay();
	I2C_SCL_1();
	Soft_I2C_Delay();
	if(I2C_READ_SDA() == 0)
	{
		I2C_SCL_0();
		Soft_I2C_Delay();
		return ;
	}	
	else
		;
		
	I2C_SCL_0();
	Soft_I2C_Delay();
}



void Soft_I2C_Send_ACK(uint8_t type)	//0:ack;1:nack
{
	I2C_SCL_0();
	Soft_I2C_Delay();
	if(type == 0)
		I2C_SDA_0();
	else if(type == 1)
		I2C_SDA_1();
	Soft_I2C_Delay();
	
	I2C_SCL_1();
	Soft_I2C_Delay();
	I2C_SCL_0();
	Soft_I2C_Delay();
}

void Soft_I2C_Write_Device_Register(uint8_t device_address,
									uint8_t register_address,
									uint8_t data)
{
	Soft_I2C_Start();
	Soft_I2C_Write_Byte(device_address);
	Soft_I2C_Wait_ACK();
	Soft_I2C_Write_Byte(register_address);
	Soft_I2C_Wait_ACK();
	Soft_I2C_Write_Byte(data);
	Soft_I2C_Wait_ACK();
	Soft_I2C_Stop();
}

void Soft_I2C_Read_Data(uint8_t device_address,
						uint8_t register_address,
						uint8_t *pbuffer,
						uint8_t count)
{
	Soft_I2C_Start();
	Soft_I2C_Write_Byte(device_address);
	Soft_I2C_Wait_ACK();
	Soft_I2C_Write_Byte(register_address);
	Soft_I2C_Wait_ACK();
	
	Soft_I2C_Start();
	Soft_I2C_Write_Byte(device_address + 1);
	Soft_I2C_Wait_ACK();
	
	for(;count >= 2;count --,pbuffer ++)
	{
		*pbuffer = Soft_I2C_Read_Byte();
		Soft_I2C_Send_ACK(0);
	}
	
	*pbuffer = Soft_I2C_Read_Byte();
	Soft_I2C_Send_ACK(1);
	Soft_I2C_Stop();
}
//单字节读取*****************************************
unsigned char I2C_ByteRead(unsigned char SlaveAddress,unsigned char REG_Address)
{   unsigned char REG_data;     	
	Soft_I2C_Read_Data(SlaveAddress,REG_Address,&REG_data,1);
	return REG_data;

}		

//-------------------------------mpu9250-------------------------------------
// 定义MPU9250内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

		
#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08


#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		  0x75	//IIC地址寄存器(默认数值0x68，只读)


//****************************

#define	GYRO_ADDRESS   0xD0	  //陀螺地址
#define MAG_ADDRESS    0x18   //磁场地址
#define ACCEL_ADDRESS  0xD0 
uint8_t mpu_read_data[14] = {0,};
short ax = 0,ay = 0,az = 0,gx = 0,gy = 0,gz = 0;	
#define MPU9250_SLAVE_ADDRESS  		0xd0
#define AK8963_SLAVE_ADDRESS (0x0c << 1)
uint8_t ak8963_read_data[7] = {0,};
int16_t mx = 0,my = 0,mz = 0;

void AK8963_Initial(void)
{
	/*
	Soft_I2C_Write_Device_Register(AK8963_SLAVE_ADDRESS,0x09, 0x18);		
	Soft_I2C_Write_Device_Register(AK8963_SLAVE_ADDRESS,0x0a, 0x11);		//????2,100Hz,16bit
	*/
		//MAG

 Soft_I2C_Write_Device_Register(MAG_ADDRESS,0x09,0x18);
	Soft_Delay(500000);
   Soft_I2C_Write_Device_Register(MAG_ADDRESS,0x0A,0x11);
	 Soft_I2C_Write_Device_Register(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
 //  Delayms(5);	
	Soft_Delay(500000);	
}

void MPU9250_Initial(void)
{
	/*
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x6B, 0x00);	    
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x19 , 0x07);	    
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x1A , 0x06);	        
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x1C , 0x08);	 
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x1B, 0x18); 
	Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x37, 0x02);		//pass through
	*/
	Soft_I2C_Initial();
	
	
	Soft_I2C_Write_Device_Register(GYRO_ADDRESS,PWR_MGMT_1, 0x00);	//解除休眠状态
	Soft_I2C_Write_Device_Register(GYRO_ADDRESS,SMPLRT_DIV, 0x07);
	Soft_I2C_Write_Device_Register(GYRO_ADDRESS,CONFIG, 0x06);
	Soft_I2C_Write_Device_Register(GYRO_ADDRESS,GYRO_CONFIG, 0x18);//+-2000dps
	Soft_I2C_Write_Device_Register(GYRO_ADDRESS,ACCEL_CONFIG, 0x00);//+-16g
	
	AK8963_Initial();
		Soft_Delay(500000);				
}




void READ_MPU9250(short *acc,short *gyo,short *mag)
{
	unsigned char BUF[6]={0};
	 //读取加速度
	 BUF[0]=I2C_ByteRead(ACCEL_ADDRESS,ACCEL_XOUT_L); 
   BUF[1]=I2C_ByteRead(ACCEL_ADDRESS,ACCEL_XOUT_H);
   //A_Y=	(BUF[1]<<8)|BUF[0];
		acc[1]=	(BUF[1]<<8)|BUF[0];
   //A_X/=164; 						   //读取计算X轴数据

   BUF[2]=I2C_ByteRead(ACCEL_ADDRESS,ACCEL_YOUT_L);
   BUF[3]=I2C_ByteRead(ACCEL_ADDRESS,ACCEL_YOUT_H);
   //A_X=	(BUF[3]<<8)|BUF[2];
		acc[0]=	(BUF[3]<<8)|BUF[2];
 //  A_Y/=164; 						   //读取计算Y轴数据
   BUF[4]=I2C_ByteRead(ACCEL_ADDRESS,ACCEL_ZOUT_L);
   BUF[5]=I2C_ByteRead(ACCEL_ADDRESS,ACCEL_ZOUT_H);
   //A_Z=	(BUF[5]<<8)|BUF[4];
		acc[2]=(BUF[5]<<8)|BUF[4];
 //  A_Z/=164; 			
	
	//读取 陀螺仪
	 BUF[0]=I2C_ByteRead(GYRO_ADDRESS,GYRO_XOUT_L); 
   BUF[1]=I2C_ByteRead(GYRO_ADDRESS,GYRO_XOUT_H);
   //G_Y=	-1*((BUF[1]<<8)|BUF[0]);
	 gyo[1]=-1*((BUF[1]<<8)|BUF[0]);
  // G_X/=16.4; 						   //读取计算X轴数据

   BUF[2]=I2C_ByteRead(GYRO_ADDRESS,GYRO_YOUT_L);
   BUF[3]=I2C_ByteRead(GYRO_ADDRESS,GYRO_YOUT_H);
   //G_X=	-1*((BUF[3]<<8)|BUF[2]);
	 gyo[0]=-1*((BUF[3]<<8)|BUF[2]);
 //  G_Y/=16.4; 						   //读取计算Y轴数据
   BUF[4]=I2C_ByteRead(GYRO_ADDRESS,GYRO_ZOUT_L);
   BUF[5]=I2C_ByteRead(GYRO_ADDRESS,GYRO_ZOUT_H);
  // G_Z=	-1*((BUF[5]<<8)|BUF[4]);
	 gyo[2]=-1*((BUF[5]<<8)|BUF[4]);
 //  G_Z/=16.4; 	//读取计算Z轴数据
	 
	 //读取磁力计
	 //Soft_Delay(50000);	
	 //Soft_I2C_Write_Device_Register(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
   //Soft_Delay(50000);	
	// Soft_Delay(100000);	
   Soft_I2C_Write_Device_Register(MAG_ADDRESS,0x0A,0x01);
   Soft_Delay(100000);	
	 //Soft_Delay(1000000);
   BUF[0]=I2C_ByteRead (MAG_ADDRESS,MAG_XOUT_L);
   BUF[1]=I2C_ByteRead (MAG_ADDRESS,MAG_XOUT_H);
   //M_Y=-1*((BUF[1]<<8)|BUF[0]);
	 mag[1]=-1*((BUF[1]<<8)|BUF[0]);

   BUF[2]=I2C_ByteRead(MAG_ADDRESS,MAG_YOUT_L);
   BUF[3]=I2C_ByteRead(MAG_ADDRESS,MAG_YOUT_H);
   //M_X=	-1*((BUF[3]<<8)|BUF[2]);
	 mag[0]=-1*((BUF[3]<<8)|BUF[2]);
   						   //读取计算Y轴数据
	 
   BUF[4]=I2C_ByteRead(MAG_ADDRESS,MAG_ZOUT_L);
   BUF[5]=I2C_ByteRead(MAG_ADDRESS,MAG_ZOUT_H);
   //M_Z=-1*((BUF[5]<<8)|BUF[4]);
	 mag[2]=-1*((BUF[5]<<8)|BUF[4]);
 					       //读取计算Z轴数据
				 
}
