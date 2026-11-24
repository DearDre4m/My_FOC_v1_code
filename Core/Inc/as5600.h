#ifndef _AS5600_H
#define _AS5600_H

#include "main.h"

#define AS5600_ADDR 0X36U
#define AS5600_OUTPUT_HIGH_REG 0x0CU
#define AS5600_OUTPUT_LOW_REG 0x0DU
#define pi  3.14159265358979323846f

extern float Motor_Angle;
extern float Motor_Angle_Last;
extern float Motor_Angle_Now;

void AS5600_AngleInit_Rad(void);
void AS5600_AngelUpdate_Rad(void);
HAL_StatusTypeDef AS5600_ReadRawData(I2C_HandleTypeDef *hi2c, uint16_t *raw_angle_data);
float AS5600_AngleDataChange_Degree(uint16_t raw_data);
float AS5600_AngleDataChange_Rad(uint16_t raw_data);
void AS5600_UsartTransmitAngle(float angle_data);
float AS5600_Diff_Degree(float data_now, float data_last);
float AS5600_Diff_Rad(float data_now, float data_last);


#endif
