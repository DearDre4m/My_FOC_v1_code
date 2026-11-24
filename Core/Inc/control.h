#ifndef _CONTROL_H
#define _CONTROL_H

#include "main.h"

typedef struct
{
    float KP; //比例系数 Kpω
    float KI; //积分系数 Kiω
    float Ba; //阻尼系数
    float integral;  // 积分状态
    float out_max;   //限幅上限
    float out_min;   //限幅下限
    float iq_output; //输出：q轴电流期望值
    float id_output; //输出：d轴电流期望值
}Speed_PI;

typedef struct
{
    float KP_q; //比例系数 Kpi
    float KI_q; //积分系数 Kii
    float KP_d; //比例系数 Kpi
    float KI_d; //积分系数 Kii
    float integral_q;  // 积分状态
    float integral_d;  // 积分状态
    float out_max;   //限幅上限
    float out_min;   //限幅下限
    float uq_output; //输出：q轴电流期望值
    float ud_output; //输出：d轴电流期望值
}Current_PI;


/* 两个独立的转速PI控制器 */
extern Speed_PI PI_Speed_normal;
extern Speed_PI PI_Speed_decay;

extern Current_PI PI_Current_normal;

#define SPEED_TS 0.01f//依靠软件计数器触发
#define MOTOR_J 3.0e-6f
#define CONTROL_SPEED_OUTMAX 3.0f
#define CONTROL_SPEED_OUTMIN -3.0f
#define CONTROL_SPEED_WB 50.0f
#define CONTROL_SPEED_B 0.0f

#define CURRENT_TS 0.0001f
#define CONTROL_CURRENT_OUTMAX 7.0f
#define CONTROL_CURRENT_OUTMIN -7.0f
#define CONTROL_CURRENT_WB 6000.0f


/* 误差死区：1rpm，换算为rad/s */
#define CONTROL_SPEED_ERR_DEADZONE (1.0f * 2.0f * 3.1415926f / 60.0f)
/* 每10ms执行一次积分衰减 */
#define SPEED_INTEGRAL_DECAY  0.9995f

/* 初始化 */
void Control_Speed_PI_init(Speed_PI *PI);
void Control_Current_PI_init(Current_PI *PI);
/* 普通转速PI */
void Control_Speed_PI_normal_run(float wm_expect, float wm_real, Speed_PI *PI);
/* 带积分衰减的转速PI */
void Control_Speed_PI_decay_run(float wm_expect, float wm_real, Speed_PI *PI);
/* 普通电流PI */
void Control_Current_PI_normal_run(float iq_expect, float iq_real, float id_expect, float id_real, Current_PI *PI);



#endif