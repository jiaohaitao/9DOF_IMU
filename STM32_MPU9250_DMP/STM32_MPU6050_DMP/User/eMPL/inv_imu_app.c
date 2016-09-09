
#include "mpu9250_driver.h"
#include "imu_init_position.h"
#include "inv_imu_app.h"

#define 	FILTER_LENGTH  50
void Get_Imu_Init_Position(float *Imu_Init_Position)
{
		short Acc[3]={0};
		short Gyo[3]={0};
		short Mag[3]={0};
		short Mag_Offset[3]={0};
		int Acc_Filter_Sum[3]={0};
		int Gyo_Filter_Sum[3]={0};
		int Mag_Filter_Sum[3]={0};
		int i=0,j=0;
		MPU9250_Initial();
		MPU9250_GetMag_Offset(Mag_Offset);
		
		for(i=0;i<FILTER_LENGTH;i++){
			READ_MPU9250(Acc,Gyo,Mag);	
			for(j=0;j<3;j++){
				Acc_Filter_Sum[j]+=Acc[j];
				Gyo_Filter_Sum[j]+=Gyo[j];
				Mag[j]-=Mag_Offset[j];
				Mag_Filter_Sum[j]+=Mag[j];
			}
		}
			
		for(i=0;i<3;i++){
			Acc[i]=Acc_Filter_Sum[i]/FILTER_LENGTH;
			Gyo[i]=Gyo_Filter_Sum[i]/FILTER_LENGTH;
			Mag[i]=Mag_Filter_Sum[i]/FILTER_LENGTH;
		}
		reset_sensor_fusion(Acc,Gyo,Mag,Imu_Init_Position);
}	
