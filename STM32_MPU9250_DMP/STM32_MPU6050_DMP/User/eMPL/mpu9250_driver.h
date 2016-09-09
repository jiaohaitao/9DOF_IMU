#ifndef _MPU_9250_H
#define _MPU_9250_H

void MPU9250_Initial(void);
void READ_MPU9250(short *acc,short *gyo,short *mag);
void MPU9250_GetMag_Offset(short *magoffset);

#endif