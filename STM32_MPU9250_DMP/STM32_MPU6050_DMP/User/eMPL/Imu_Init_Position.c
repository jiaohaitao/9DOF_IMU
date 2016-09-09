#include "imu_init_position.h"
#include "math.h"

static float Init_Pitch=0,Init_Roll=0,Init_Yaw=0;

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
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

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
// Init rotation matrix using euler angles
void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll)
{
  float c1 = cos(roll);
  float s1 = sin(roll);
  float c2 = cos(pitch);
  float s2 = sin(pitch);
  float c3 = cos(yaw);
  float s3 = sin(yaw);

  // Euler angles, right-handed, intrinsic, XYZ convention
  // (which means: rotate around body axes Z, Y', X'') 
  m[0][0] = c2 * c3;
  m[0][1] = c3 * s1 * s2 - c1 * s3;
  m[0][2] = s1 * s3 + c1 * c3 * s2;

  m[1][0] = c2 * s3;
  m[1][1] = c1 * c3 + s1 * s2 * s3;
  m[1][2] = c1 * s2 * s3 - c3 * s1;

  m[2][0] = -s2;
  m[2][1] = c2 * s1;
  m[2][2] = c1 * c2;
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion(short *acc,short *gyo,short *mag,float *imu_position) {
		unsigned long temp_time;	
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};
	int i=0;
/*
		Soft_I2C_Read_Data(MPU9250_SLAVE_ADDRESS,0x3B,mpu_read_data,14);

		ax = ((unsigned short)mpu_read_data[0] << 8) | mpu_read_data[1];
		ay = ((unsigned short) mpu_read_data[2] << 8) | mpu_read_data[3];
		az = ((unsigned short)mpu_read_data[4] << 8) | mpu_read_data[5];
		gx = ((unsigned short)mpu_read_data[8] << 8) | mpu_read_data[9];
		gy = ((unsigned short)mpu_read_data[10] << 8) | mpu_read_data[11];
		gz = ((unsigned short)mpu_read_data[12] << 8) | mpu_read_data[13];
	
		Soft_I2C_Write_Device_Register(MPU9250_SLAVE_ADDRESS,0x37,0x02);//turn on Bypass Mode 
		Soft_Delay(1000);	
		Soft_I2C_Write_Device_Register(AK8963_SLAVE_ADDRESS,0x0A,0x01);
		Soft_Delay(50000);	
		
		Soft_I2C_Read_Data(AK8963_SLAVE_ADDRESS,0x03,ak8963_read_data,7);			//????2:100Hz,???0x09 ST2!!!??????ST2????!!!				
		
		mx = ((uint16_t)ak8963_read_data[1] << 8) | ak8963_read_data[0];
		my = ((uint16_t)ak8963_read_data[3] << 8) | ak8963_read_data[2];
		mz = ((uint16_t)ak8963_read_data[5] << 8) | ak8963_read_data[4];
*/			
		for(i=0;i<3;i++){
			temp_acc[i]=acc[i];
			temp_gyo[i]=gyo[i];
			temp_mag[i]=mag[i];
		}
//	mpu_get_gyro_reg(temp_gyo1,&temp_time);
//	mpu_get_accel_reg(temp_acc1,&temp_time);
//	mpu_get_compass_reg(temp_mag1,&temp_time);


//	temp_gyo1[0]=temp_gyo1[1]=temp_gyo1[2]=0;
//	temp_acc1[0]=temp_acc1[1]=temp_acc1[2]=1000;
//	temp_mag1[0]=temp_mag1[1]=temp_mag1[2]=0;

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
  init_rotation_matrix(DCM_Matrix, Init_Yaw, Init_Pitch, Init_Roll);
	
//void Euler_angles(void)
{
  Init_Pitch = -asin(DCM_Matrix[2][0]);
  Init_Roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  Init_Yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}	
	Init_Pitch*=180.0/3.14;
	Init_Roll*=180.0/3.14;
	Init_Yaw*=180.0/3.14;

	imu_position[0]=Init_Roll;
	imu_position[1]=Init_Pitch;
  imu_position[2]=Init_Yaw;
}