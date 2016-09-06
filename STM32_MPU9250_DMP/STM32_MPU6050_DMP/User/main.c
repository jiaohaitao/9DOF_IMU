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
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "math.h"
#include "usart.h"
#include "stm32_iic.h"



#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
    
#define q30  1073741824.0f


float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;


u8 show_tab[]={"0123456789"};

float Pitch,Roll,Yaw;
int temp;

float Init_Pitch=0,Init_Roll=0,Init_Yaw=0;
 
 
struct rx_s 
{
    unsigned char header[3];
    unsigned char cmd;
};


struct hal_s 
{
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};


static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

                                           
enum packet_type_e
{
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC
};

 

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}



static  unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

 /*自检函数*/
static void run_self_test(void)
{
    int result;

    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) 
    {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		PrintChar("setting bias succesfully ......\n");
    }
	else
	{
		PrintChar("bias has not been modified ......\n");
	}
    
}
 


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




//误差纠正
#define  Pitch_error  1.0
#define  Roll_error   -2.0
#define  Yaw_error    0.0


void Data_Send_Status(float rol,float pit,float yaw);
void reset_sensor_fusion(void);
void delay()
	{
	long i,j;
	for(i=0;i<1000;i++)
		for(j=0;j<10000;j++);
}
void Send_Data(int16_t ad1,int16_t ad2,int16_t ad3,int16_t ad4,int16_t ad5,int16_t ad6,int16_t ad7,int16_t ad8,int16_t ad9);
int main(void)
{  
//声明相关变量
unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];	
	
    int result=0;
    u16 count=0;    
		
	
	
    SystemInit();
    
    USART2_Config();  //串口初始化
  
    LED_Config();
    
    i2cInit();      //IIC总线的初始化
   
    result = mpu_init();
    // PrintChar("123...\n ");
    led_on;
    if(!result)   //返回0代表初始化成功
    {   
        PrintChar("mpu initialization complete......\n ");
        
        //mpu_set_sensor
        if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
        {
            PrintChar("mpu_set_sensor complete ......\n");
        }
        else
        {
            PrintChar("mpu_set_sensor come across error ......\n");
        }
        
        //mpu_configure_fifo
        if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
        {
            PrintChar("mpu_configure_fifo complete ......\n");
        }
        else
        {
            PrintChar("mpu_configure_fifo come across error ......\n");
        }
        
        //mpu_set_sample_rate
        if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
        {
            PrintChar("mpu_set_sample_rate complete ......\n");
        }
        else
        {
            PrintChar("mpu_set_sample_rate error ......\n");
        }
        
        //dmp_load_motion_driver_firmvare
        if(!dmp_load_motion_driver_firmware())
        {
            PrintChar("dmp_load_motion_driver_firmware complete ......\n");
        }
        else
        {
            PrintChar("dmp_load_motion_driver_firmware come across error ......\n");
        }
        
        //dmp_set_orientation
        if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
        {
            PrintChar("dmp_set_orientation complete ......\n");
        }
        else
        {
            PrintChar("dmp_set_orientation come across error ......\n");
        }
        
        //dmp_enable_feature
        if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL))
        {
            PrintChar("dmp_enable_feature complete ......\n");
        }
        else
        {
            PrintChar("dmp_enable_feature come across error ......\n");
        }
        
        //dmp_set_fifo_rate
        if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
        {
            PrintChar("dmp_set_fifo_rate complete ......\n");
        }
        else
        {
            PrintChar("dmp_set_fifo_rate come across error ......\n");
        }
        
        run_self_test();
        
        if(!mpu_set_dmp_state(1))
        {
            PrintChar("mpu_set_dmp_state complete ......\n");
        }
        else
        {
            PrintChar("mpu_set_dmp_state come across error ......\n");
        }
        
    }
    led_off;
    delay();
		//reset_sensor_fusion();
    while(1)
    {
//        count ++;          //灯亮灭代表正在运行状态
//        if(count<1000)
//        {
//            led_on;  
//            
//        }
//        else if(count<2000)
//        {
//            led_off;  
//            
//        }
//        else if(count==2000)
//        {
//            count = 0; 
//            
//        }
        
        
        //float Yaw,Roll,Pitch;
				led_on;
        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	 
				led_off;
        /* Gyro and accel data are written to the FIFO by the DMP in chip
        * frame and hardware units. This behavior is convenient because it
        * keeps the gyro and accel outputs of dmp_read_fifo and
        * mpu_read_fifo consistent.
        */
        /*     if (sensors & INV_XYZ_GYRO )
        send_packet(PACKET_TYPE_GYRO, gyro);
        if (sensors & INV_XYZ_ACCEL)
        send_packet(PACKET_TYPE_ACCEL, accel); */
        /* Unlike gyro and accel, quaternions are written to the FIFO in
        * the body frame, q30. The orientation is set by the scalar passed
        * to dmp_set_orientation during initialization.
        */
				/*四元数解姿态*/
        if (sensors & INV_WXYZ_QUAT )
        {
            q0 = quat[0] / q30;
            q1 = quat[1] / q30;
            q2 = quat[2] / q30;
            q3 = quat[3] / q30;
            
            Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 + Pitch_error; // pitch
            Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 + Roll_error; // roll
            Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3 + Yaw_error;
						Yaw*=-1.0;
						Pitch*=-1.0;
					
						Roll+=Init_Roll;
					  Yaw+=Init_Yaw;
					  Pitch+=Init_Pitch;
					   
									
           /*
            //显示Pitch
            PrintChar("Pitch:");
            temp = (int)(Pitch*100.0);
            if(temp>=0)
            {
                UsartSend('+');
                UsartSend(show_tab[temp/1000]);
                UsartSend(show_tab[temp%1000/100]);
                UsartSend('.');
                UsartSend(show_tab[temp%100/10]);
                UsartSend(show_tab[temp%10]);

            }
            else
            {
                temp = - temp;
                UsartSend('-');
                UsartSend(show_tab[temp/1000]);
                UsartSend(show_tab[temp%1000/100]);
                UsartSend('.');
                UsartSend(show_tab[temp%100/10]);
                UsartSend(show_tab[temp%10]);

            }
            PrintChar("度 ");
            
            
            //显示Roll
            PrintChar("Roll:");
            temp = (int)(Roll*100.0);
            if(temp>=0)
            {
                UsartSend('+');
                UsartSend(show_tab[temp/1000]);
                UsartSend(show_tab[temp%1000/100]);
                UsartSend('.');
                UsartSend(show_tab[temp%100/10]);
                UsartSend(show_tab[temp%10]);

            }
            else
            {
                temp = - temp;
                UsartSend('-');
                UsartSend(show_tab[temp/1000]);
                UsartSend(show_tab[temp%1000/100]);
                UsartSend('.');
                UsartSend(show_tab[temp%100/10]);
                UsartSend(show_tab[temp%10]);

            }      
            PrintChar("度 ");
            
                         
            //显示Yaw
            PrintChar("Yaw:");
            temp = (int)(Yaw*100.0);
            if(temp>=0)
            {
                UsartSend('+');
                UsartSend(show_tab[temp/1000]);
                UsartSend(show_tab[temp%1000/100]);
                UsartSend('.');
                UsartSend(show_tab[temp%100/10]);
                UsartSend(show_tab[temp%10]);

            }
            else
            {
                temp = - temp;
                UsartSend('-');
                UsartSend(show_tab[temp/1000]);
                UsartSend(show_tab[temp%1000/100]);
                UsartSend('.');
                UsartSend(show_tab[temp%100/10]);
                UsartSend(show_tab[temp%10]);

            }
            PrintChar("度 ");
            
            PrintChar("\r\n");
						*/
            Data_Send_Status(Roll,Pitch,Yaw);
						Send_Data(accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2],0,0,0);
        }

        if(sensors & INV_XYZ_GYRO)
        {}
        if(sensors & INV_XYZ_ACCEL)
        {}

    }

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

 // Computes the cross product of two vectors
// out has to different from v1 and v2 (no in-place)!
void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3])
{
  out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}
		float temp_acc[3];
		float temp_gyo[3];
		float temp_mag[3];
float MAG_Heading;

void Compass_Heading()
{
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(Init_Roll);
  sin_roll = sin(Init_Roll);
  cos_pitch = cos(Init_Pitch);
  sin_pitch = sin(Init_Pitch);
  
  // Tilt compensated magnetic field X
  mag_x = temp_mag[0] * cos_pitch + temp_mag[1] * sin_roll * sin_pitch + temp_mag[2] * cos_roll * sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = temp_mag[1] * cos_roll - temp_mag[2] * sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-mag_y, mag_x);
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion() {
		unsigned long temp_time;	
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};
	short temp_gyo1[3]={0};
	short temp_acc1[3]={0};
	short temp_mag1[3]={0};
	int i=0;
	mpu_get_gyro_reg(temp_gyo1,&temp_time);
	mpu_get_accel_reg(temp_acc1,&temp_time);
	mpu_get_compass_reg(temp_mag1,&temp_time);
//	temp_gyo1[0]=temp_gyo1[1]=temp_gyo1[2]=0;
//	temp_acc1[0]=temp_acc1[1]=temp_acc1[2]=1000;
//	temp_mag1[0]=temp_mag1[1]=temp_mag1[2]=0;
	for(i=0;i<3;i++){
	temp_gyo[i]=temp_gyo1[i];
		temp_acc[i]=temp_acc1[i];
		temp_mag[i]=temp_mag1[i];
	}
//  read_sensors();
//  timestamp = millis();
  
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  Init_Pitch = -atan2(temp_acc[0], sqrt(temp_acc[1] * temp_acc[1] + temp_acc[2] * temp_acc[2]));
	
  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, temp_acc, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  Init_Roll = atan2(temp2[1], temp2[2]);
  
  // GET YAW
  Compass_Heading();
  Init_Yaw = MAG_Heading;
  
  // Init rotation matrix
//  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}
//void Send_Data(void)
//{
//	unsigned char i=0;
//	unsigned char _cnt=0,sum = 0;
////	unsigned int _temp;
//	u8 data_to_send[50];

//	short temp_gyo1[3]={0};
//	short temp_acc1[3]={0};
//	short temp_mag1[3]={0};
//	unsigned long temp_time;	
//	mpu_get_gyro_reg(temp_gyo1,&temp_time);
//	mpu_get_accel_reg(temp_acc1,&temp_time);
//	mpu_get_compass_reg(temp_mag1,&temp_time);	
//	
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x02;
//	data_to_send[_cnt++]=0;
//	

//	data_to_send[_cnt++]=BYTE1(temp_acc1[0]);
//	data_to_send[_cnt++]=BYTE0(temp_acc1[0]);
//	data_to_send[_cnt++]=BYTE1(temp_acc1[1]);
//	data_to_send[_cnt++]=BYTE0(temp_acc1[1]);
//	data_to_send[_cnt++]=BYTE1(temp_acc1[2]);
//	data_to_send[_cnt++]=BYTE0(temp_acc1[2]);
//	
//	data_to_send[_cnt++]=BYTE1(temp_gyo1[0]);
//	data_to_send[_cnt++]=BYTE0(temp_gyo1[0]);
//	data_to_send[_cnt++]=BYTE1(temp_gyo1[1]);
//	data_to_send[_cnt++]=BYTE0(temp_gyo1[1]);
//	data_to_send[_cnt++]=BYTE1(temp_gyo1[2]);
//	data_to_send[_cnt++]=BYTE0(temp_gyo1[2]);
//	data_to_send[_cnt++]=BYTE1(temp_mag1[0]);
//	data_to_send[_cnt++]=BYTE0(temp_mag1[0]);
//	data_to_send[_cnt++]=BYTE1(temp_mag1[1]);
//	data_to_send[_cnt++]=BYTE0(temp_mag1[1]);
//	data_to_send[_cnt++]=BYTE1(temp_mag1[2]);
//	data_to_send[_cnt++]=BYTE0(temp_mag1[2]);
//	
//	data_to_send[3] = _cnt-4;
//	//oíD￡?é
//	for(i=0;i<_cnt;i++)
//		sum+= data_to_send[i];
//	data_to_send[_cnt++]=sum;
//	
//	for(i=0;i<_cnt;i++){
//		USART_SendData(USART1,data_to_send[i]);
//		while( USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET );
//	}
//}
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
