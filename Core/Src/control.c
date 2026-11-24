#include "control.h"
#include "as5600.h"
#include "motor.h"
#include <math.h>

Speed_PI PI_Speed_normal;
Speed_PI PI_Speed_decay;

Current_PI PI_Current_normal;


void Control_Speed_PI_init(Speed_PI *PI)
{
    if (PI == NULL)
        return;
    float K, BA, kp, ki;
    K =  (1.50f * MOTOR_POLE_PAIRS * MOTOR_PSI_F)/MOTOR_J;
    BA = (CONTROL_SPEED_WB - CONTROL_SPEED_B / MOTOR_J) / K;
    kp = CONTROL_SPEED_WB / K;
    ki = CONTROL_SPEED_WB * kp;
    PI->KP = kp;
    PI->KI = ki;
    PI->Ba = BA;
    PI->out_max = CONTROL_SPEED_OUTMAX;
    PI->out_min = CONTROL_SPEED_OUTMIN;
    PI->integral  = 0.0f;
    PI->iq_output = 0.0f;
    PI->id_output = 0.0f;
}

void Control_Current_PI_init(Current_PI *PI)
{
    if (PI == NULL)
        return;
    float kp_q, ki_q, kp_d, ki_d;
    kp_q = CONTROL_CURRENT_WB * MOTOR_LQ;
    ki_q = CONTROL_CURRENT_WB * MOTOR_RS;
    kp_d = CONTROL_CURRENT_WB * MOTOR_LD;
    ki_d = CONTROL_CURRENT_WB * MOTOR_RS;
    PI->KP_q = kp_q;
    PI->KI_q = ki_q;
    PI->KP_d = kp_d;
    PI->KI_d = ki_d;
    PI->out_max = CONTROL_CURRENT_OUTMAX;
    PI->out_min = CONTROL_CURRENT_OUTMIN;
    PI->integral_q  = 0.0f;
    PI->integral_d  = 0.0f;
    PI->uq_output = 0.0f;
    PI->ud_output = 0.0f;
}

void Control_Speed_PI_normal_run(float wm_expect, float wm_real, Speed_PI *PI)
{
    if (PI == NULL)
        return;

    /* 1. 计算转速误差 */
    float wm_err = wm_expect - wm_real;

    /* 2. 计算积分候选值 */
    float integral_new = PI->integral + wm_err * SPEED_TS;

    /* 3. 计算未限幅的期望电流 */
    float iq_output_test = PI->KP * wm_err + PI->KI * integral_new - PI->Ba * wm_real;

        /* 4. 抗积分饱和 */
    if (!(((iq_output_test > PI->out_max) && (wm_err > 0.0f)) || ((iq_output_test < PI->out_min) && (wm_err < 0.0f))))
    {
        PI->integral = integral_new;
    }
    

    /* 5. 计算最终iq输出 */
    PI->iq_output = PI->KP * wm_err + PI->KI * PI->integral - PI->Ba * wm_real;

    /* 6. iq输出限幅 */
    if (PI->iq_output > PI->out_max)
    {
        PI->iq_output = PI->out_max;
    }
    else if (PI->iq_output < PI->out_min)
    {
        PI->iq_output = PI->out_min;
    }

    /* 7. 计算最终id输出 */
    PI->id_output = 0.0f;
}

void Control_Speed_PI_decay_run(float wm_expect, float wm_real, Speed_PI *PI)
{
    if (PI == NULL)
    return;

    /* 计算转速误差 */
    float wm_err = wm_expect - wm_real;
    float wm_abs = fabsf(wm_err);

    /* 误差小于误差死区进行积分衰减 */
    if(wm_abs <= CONTROL_SPEED_ERR_DEADZONE)
    {
        PI->integral = SPEED_INTEGRAL_DECAY * PI->integral;
        wm_err = 0.0f;
    }
    /* 误差大于误差死区计算积分 */
    else
    {
        /* 计算积分候选值 */
        float integral_new =  PI->integral + wm_err * SPEED_TS;

        /* 计算未限幅的期望电流 */
        float iq_output_test = PI->KP * wm_err + PI->KI * integral_new - PI->Ba * wm_real;

        /* 抗积分饱和 */
        if (!(((iq_output_test > PI->out_max) && (wm_err > 0.0f)) || ((iq_output_test < PI->out_min) && (wm_err < 0.0f))))
        {
            PI->integral = integral_new; 
        }
    }
    
    /* 计算最终iq输出 */
    PI->iq_output = PI->KP * wm_err + PI->KI * PI->integral - PI->Ba * wm_real;

    /* iq输出限幅 */
    if (PI->iq_output > PI->out_max)
    {
        PI->iq_output = PI->out_max;
    }
    else if (PI->iq_output < PI->out_min)
    {
        PI->iq_output = PI->out_min;
    }

    /* 计算最终id输出 */
    PI->id_output = 0.0f;


}

void Control_Current_PI_normal_run(float iq_expect, float iq_real, float id_expect, float id_real, Current_PI *PI)
{
    if (PI == NULL)
        return;

    /* 1. 计算转速误差 */
    float iq_err = iq_expect - iq_real;
    float id_err = id_expect - id_real;
    /* 2. 计算积分候选值 */
    float integral_q_new = PI->integral_q + iq_err * CURRENT_TS;
    float integral_d_new = PI->integral_d + id_err * CURRENT_TS;

    /* 3. 计算未限幅的电压输出 */
    float uq_output_test = PI->KP_q * iq_err + PI->KI_q * integral_q_new ;
    float ud_output_test = PI->KP_d * id_err + PI->KI_d * integral_d_new ;

    /* 4.抗积分饱和 */
    if (!(((uq_output_test > PI->out_max) && (iq_err > 0.0f)) || ((uq_output_test < PI->out_min) && (iq_err < 0.0f))))
    {
        PI->integral_q = integral_q_new;
    }
    
    if (!(((ud_output_test > PI->out_max) && (id_err > 0.0f)) || ((ud_output_test < PI->out_min) && (id_err < 0.0f))))
    {
        PI->integral_d = integral_d_new;
    }
    /* 5. 计算最终uq、ud输出 */
    PI->uq_output = PI->KP_q * iq_err + PI->KI_q * PI->integral_q;
    PI->ud_output = PI->KP_d * id_err + PI->KI_d * PI->integral_d;

    /* 6. uq输出限幅 */
    if (PI->uq_output > PI->out_max)
    {
        PI->uq_output = PI->out_max;
    }
    else if (PI->uq_output < PI->out_min)
    {
        PI->uq_output = PI->out_min;
    }

    /* 7. ud输出限幅 */
    if (PI->ud_output > PI->out_max)
    {
        PI->ud_output = PI->out_max;
    }
    else if (PI->ud_output < PI->out_min)
    {
        PI->ud_output = PI->out_min;
    }

}

