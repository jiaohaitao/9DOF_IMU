#ifndef _MY_INV_MPU
#define _MY_INV_MPU
int my_mpu_init(void);
int my_read_imu(short *accel,short *gyro,float *Roll,float *Pitch,float *Yaw);
#endif