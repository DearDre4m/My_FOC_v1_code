#ifndef _MOTOR_H
#define _MOTOR_H
#include "tim.h"
#include "i2c.h"
#include "as5600.h"
#include <math.h>
#include <stdint.h>


/* ================= PMSM电机物理参数 ================= */

/* 以下为暂定参考值，尚未针对2804D3实测确认 */
#define MOTOR_RS 2.3f /* 定子等效相电阻，Ω */
#define MOTOR_LD 0.00086f /* d轴电感，H */
#define MOTOR_LQ 0.00086f /* q轴电感，H */
#define MOTOR_PSI_F 0.0035f /* 永磁磁链，Wb */
#define MOTOR_POLE_PAIRS 7.0f /* 极对数 */
/* ==================== 1. 电角度校准相关变量 ==================== */

/* 电角度零点偏移，单位rad */
extern float Motor_Electric_Offset;

/* AS5600传感器方向，+1或-1 */
extern int8_t Motor_Sensor_Direction;

/* 电角度是否校准成功 */
extern uint8_t Motor_Electric_Calibrated;

/* 用于检查方向校准的机械角度变化，单位rad */
extern float Motor_Calib_DeltaRad;

/* 电角度为0时，AS5600测得的机械角度，单位rad */
extern float Motor_ElectricZero_MechanicalAngle;

extern volatile uint8_t Motor_Calib_Result;
/* ==================== 2. 电机初始化 ==================== */
void Motor_Init(void);

/* ==================== 3. 电机硬件驱动 ==================== */

/* 驱动板使能与关闭 */
void Motor_Enable(void);
void Motor_Disable(void);

/* 三相PWM启动与比较值设置 */
void Motor_Pwm_Start(void);
void Motor_Pwm_Set(uint16_t pwm_ccr1, uint16_t pwm_ccr2, uint16_t pwm_ccr3);


/* ==================== 4. 坐标变换 ==================== */

/* Clarke变换：abc -> alpha-beta */
void Motor_Clarke(float ua, float ub, float uc, float *Ualpha, float *Ubeta);

void Motor_InvClarke(float Ualpha, float Ubeta, float *ua, float *ub, float *uc);
/* Park变换：alpha-beta -> dq */
void Motor_Park(float Ualpha, float Ubeta, float theta, float *Ud, float *Uq);

void Motor_InvPark(float Ud, float Uq, float theta, float *Ualpha, float *Ubeta);

/* ==================== 5. PMSM物理模型 ==================== */

/* PMSM dq轴电流->dq轴电压物理模型 */
void Motor_DQModel(float Id, float Iq, float dId, float dIq, float omega_e, float *Ud, float *Uq);


/* ==================== 6. SPWM调制 ==================== */

/* 基于三相参考电压的SPWM */
void Motor_SPWM(float ua, float ub, float uc, float vdc);

/* dq轴参考电压 -> 逆Park -> 逆Clarke -> SPWM/SVPWM */
void Motor_SetPhaseVoltage_dq_SPWM(float ud, float uq, float theta_e, float vdc);

void Motor_SetPhaseVoltage_dq_SVPWM(float ud, float uq, float theta_e, float vdc);
/* ==================== 7. 电角度校准 ==================== */

/* 电角度方向及零点校准 */
uint8_t Motor_CalibrationElectricAngle(float U_align, float Vdc);


/* ==================== 8. 机械角度与电角度获取更新 ==================== */

/* 获取校准后的累计机械角度并更新，逆时针为正，单位rad */
float Motor_GetMechanicalAngle_Rad(void);

/* 获取校准后的累计电角度并更新，单位rad */
float Motor_GetElectricAngle_Rad(void);

/* 将弧度限制到[0, 2PI] */
float Motor_WrapAngle_Rad(float angle);
/* ==================== 9. FOC控制 ==================== */

/* 基于PMSM dq物理模型的开环电流控制 */
void Motor_VoltageFOC(float id, float iq,float dId, float dIq,float theta_e, float omega_e,float Vdc);


#endif