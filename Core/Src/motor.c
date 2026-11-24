#include "motor.h"

#define MOTOR_PI         3.14159265359f
#define MOTOR_TWO_PI     (2.0f * MOTOR_PI)



volatile uint8_t Motor_Calib_Result = 0xFFU;
/* 电角度零点偏移，单位rad */
float Motor_Electric_Offset = 0.0f;//需要根据实际的传感器读数进行修改
/* 传感器方向，+1或-1 */
int8_t Motor_Sensor_Direction = -1;//需要根据逆时针传感器读数减小还是增加来修改减小为-1反之为1本程序会自动识别
/* 电角度是否校准成功 */
uint8_t Motor_Electric_Calibrated = 0U;

/* 用于检查方向校准的机械角度变化 */
float Motor_Calib_DeltaRad = 0.0f;

/* 电角度为0时，AS5600测得的机械角度，单位rad */
float Motor_ElectricZero_MechanicalAngle = 0.0f;

/* 驱动板FLT故障信号：0表示故障，1表示未报告故障 */
volatile uint8_t Motor_FLT_State = 0xFF;
void Motor_Enable(void)
{
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
}

void Motor_Disable(void)
{
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
}

void Motor_Pwm_Start(void)
{

    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

}

void Motor_Pwm_Set(uint16_t pwm_ccr1, uint16_t pwm_ccr2, uint16_t pwm_ccr3)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 , pwm_ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2 , pwm_ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3 , pwm_ccr3);

}

void Motor_Clarke(float ua, float ub, float uc, float *Ualpha, float *Ubeta)
{
    *Ualpha = (2.0f / 3.0f)* (ua - 0.5f * ub - 0.5f * uc);

    *Ubeta = (1.0f / 1.7320508f) * (ub - uc);
}

void Motor_InvClarke(float Ualpha, float Ubeta, float *ua, float *ub, float *uc)
{
    *ua = Ualpha;
    *ub = Ualpha * -0.5f + Ubeta * ((float)sqrt(3)/2.0f); 
    *uc = Ualpha * -0.5f - Ubeta * ((float)sqrt(3)/2.0f);
}


void Motor_Park(float Ualpha, float Ubeta, float theta, float *Ud, float *Uq)
{
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    *Ud = Ualpha * cos_theta + Ubeta * sin_theta;

    *Uq = -Ualpha * sin_theta + Ubeta * cos_theta;
}

void Motor_InvPark(float Ud, float Uq, float theta, float *Ualpha, float *Ubeta)
{
    *Ualpha = cosf(theta) * Ud - sinf(theta) * Uq;
    *Ubeta = sinf(theta) * Ud + cosf(theta) * Uq;
}

/*
 * PMSM dq轴物理模型
 *
 * 输入：
 * Id      ：目标d轴电流，A
 * Iq      ：目标q轴电流，A
 * omega_e ：转子电角速度，rad/s
 *
 * 输出：
 * Ud      ：d轴前馈电压，V
 * Uq      ：q轴前馈电压，V
 *
 */

void Motor_DQModel(float Id, float Iq, float dId, float dIq,float omega_e, float *Ud, float *Uq)
{
        /* 1. 计算dq轴磁链 */
    float Psi_d = MOTOR_LD * Id + MOTOR_PSI_F;
    float Psi_q = MOTOR_LQ * Iq;

    /* 2. 计算dq轴磁链变化率 */
    float dPsi_d = MOTOR_LD * dId;
    float dPsi_q = MOTOR_LQ * dIq;

    /* 3. 根据PMSM电压方程计算dq轴电压 */
    *Ud = MOTOR_RS * Id + dPsi_d - omega_e * Psi_q;

    *Uq = MOTOR_RS * Iq + dPsi_q + omega_e * Psi_d;

}

/*
    Motor_SPWM()：根据三相参考电压计算PWM占空比

    Ua、Ub、Uc：三相参考电压，单位V
    Vdc：直流母线电压，单位V
*/
void Motor_SPWM(float ua, float ub, float uc, float vdc)
{
    if (vdc <= 0)
    {
        Motor_Disable();
        return;
    }

    /* 根据普通SPWM公式计算占空比 */
    float Da = 0.5f + ua/vdc;
    float Db = 0.5f + ub/vdc;
    float Dc = 0.5f + uc/vdc;

    /* 防止占空比越界 */
    if (Da < 0.0f || Da > 1.0f || Db < 0.0f || Db > 1.0f || Dc < 0.0f || Dc > 1.0f)
    {
        Motor_Disable();
        return;
    }

    /* 获取TIM1自动重装载值 */
    uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(&htim1);
    
    /* 占空比转换成CCR比较值 */
    float pwm_ccr1 = (uint16_t)(ARR * Da + 0.5f);//四舍五入
    float pwm_ccr2 = (uint16_t)(ARR * Db + 0.5f);
    float pwm_ccr3 = (uint16_t)(ARR * Dc + 0.5f);

    /* 写入三路PWM */
    Motor_Pwm_Set(pwm_ccr1, pwm_ccr2, pwm_ccr3);

}


/*
    Motor_SetPhaseVoltage_dq_SPWM()：dq轴电压转换为三相PWM

    Ud：d轴参考电压，单位V
    Uq：q轴参考电压，单位V
    theta_e：电角度，单位rad
    Vdc：直流母线电压，单位V
*/
void Motor_SetPhaseVoltage_dq_SPWM(float ud, float uq, float theta_e, float vdc)
{
    /* 第一步：逆Park变换 */    
    float ualpha;
    float ubeta;
    Motor_InvPark(ud, uq, theta_e, &ualpha, &ubeta);

    /* 第二步：逆Clarke变换 */
    float ua;
    float ub; 
    float uc;
    Motor_InvClarke(ualpha, ubeta, &ua, &ub, &uc);

    /* 第三步：SPWM调制 */
    Motor_SPWM(ua, ub, uc, vdc);

}

/* 提前声明静态函数 */
static uint8_t Motor_Get_Sector(float Ualpha, float Ubeta);

void Motor_SetPhaseVoltage_dq_SVPWM(float ud, float uq, float theta_e, float vdc)
{
    const float SQRT3 = 1.7320508f;
    /* 第一步：逆Park变换 */    
    float ualpha;
    float ubeta;
    Motor_InvPark(ud, uq, theta_e, &ualpha, &ubeta);
    int N;
    N = Motor_Get_Sector(ualpha, ubeta);

    float K_norm = SQRT3 / vdc;//K_norm = K/ts
    float X_norm = K_norm * ubeta;
    float Y_norm = K_norm * (0.8660254f * ualpha + 0.5f * ubeta);
    float Z_norm = K_norm * (-0.8660254f * ualpha + 0.5f * ubeta); 

    float T1 = 0.0f;
    float T2 = 0.0f;
    float T0 = 0.0f;
    switch (N)
    {
    case 5:
        T1 = -Z_norm;
        T2 = X_norm;
        break;
    case 7:
        T1 = Z_norm;
        T2 = Y_norm;
        break;
    case 3:
        T1 = X_norm;
        T2 = -Y_norm;
        break;
    case 2:
        T1 = -X_norm;
        T2 = Z_norm;
        break;
    case 0:
        T1 = -Y_norm;
        T2 = -Z_norm;
        break;
    case 4:
        T1 = Y_norm;
        T2 = -X_norm;
        break;
    default:
    Motor_Disable();
        break;
    }

    /*过调制处理*/
    float T_sum = T1 + T2;

    if (T_sum > 1.0f)
    {
        T1 = T1 / T_sum;
        T2 = T2 / T_sum;
    }
    
    T0 = 1 - T1 - T2;
    uint16_t ARR = __HAL_TIM_GET_AUTORELOAD(&htim1);

    uint16_t CCR_1, CCR_2, CCR_3, CCR_A, CCR_B, CCR_C;

    CCR_3 = (0.5f * T0 + T1 + T2) * ARR;
    CCR_2 = (0.5f * T0 + T2) * ARR;
    CCR_1 = (0.5f * T0) * ARR;
    switch (N)
    {
    case 5:
        CCR_A = CCR_3; 
        CCR_B = CCR_2; 
        CCR_C = CCR_1;
        break;
    case 7:
        CCR_A = CCR_2; 
        CCR_B = CCR_3; 
        CCR_C = CCR_1;
        break;
    case 3:
        CCR_A = CCR_1; 
        CCR_B = CCR_3; 
        CCR_C = CCR_2;
        break;
    case 2:
        CCR_A = CCR_1; 
        CCR_B = CCR_2; 
        CCR_C = CCR_3;
        break;
    case 0:
        CCR_A = CCR_2; 
        CCR_B = CCR_1; 
        CCR_C = CCR_3;
        break;
    case 4:
        CCR_A = CCR_3; 
        CCR_B = CCR_1; 
        CCR_C = CCR_2;
        break;
    default:
    Motor_Disable();
        break;
    }
    Motor_Pwm_Set(CCR_A, CCR_B, CCR_C); 
}

/* 将弧度限制到[0, 2PI] */
float Motor_WrapAngle_Rad(float angle)
{
    while(angle >= MOTOR_TWO_PI)
    {
        angle = angle - MOTOR_TWO_PI;
    }

    while(angle < 0)
    {
        angle = angle + MOTOR_TWO_PI;
    }
    
    return angle;
}


/* 读取一次AS5600机械角度，单位rad */
static HAL_StatusTypeDef Motor_ReadMechanicalAngle(float *Angle)//float *Angle指针变量实际使用时要&angle
{
    uint16_t raw_data = 0;
    HAL_StatusTypeDef status = AS5600_ReadRawData(&hi2c2, &raw_data);
    if(status == HAL_OK)
    {
        *Angle = AS5600_AngleDataChange_Rad(raw_data);//*Angle解引用传递数值保存
    }

    return status;
}


/*
    停止校准，关闭驱动并清除最后的非零电压指令
*/
static void Motor_CalibrationStop(void)
{
    Motor_Disable();
    uint16_t mid = (uint16_t)(__HAL_TIM_GET_AUTORELOAD(&htim1)/2U);
    Motor_Pwm_Set(mid,mid,mid);

}

/*
    电角度方向及零点校准

    U_align：定子磁场对齐电压，单位V
    Vdc：直流母线电压，单位V

    返回值：
    HAL_OK    ：传感器测量和方向检查通过
    HAL_ERROR ：校准条件不满足或转子未正确跟随
    其他值    ：传感器通信异常

    使用前必须先完成驱动板、PWM和限流安全检查。
    只允许在启动阶段调用，不能在定时器中断中调用。
*/
uint8_t Motor_CalibrationElectricAngle(float U_align, float Vdc)
{
    float theta_0;
    float theta_90;
    float theta_zero;
    float delta;
    float expected_delta;

    HAL_StatusTypeDef status;

    Motor_Electric_Calibrated = 0;
    Motor_Calib_DeltaRad = 0.0f;

    /*先关闭驱动板*/
    Motor_CalibrationStop();
    /*检查参数*/
    if(Vdc <= 0.0f || U_align <= 0.0f || U_align >= Vdc / 2.0f)
    {
        return 1;
    }

    /* 第一阶段：建立0°电角度的固定磁场 */
    //Motor_SetPhaseVoltage_dq_SPWM(U_align, 0, 0, Vdc);
    /* 等待PWM比较寄存器更新 */
    //HAL_Delay(5u);
    /* 开始短时通电对齐 */
    //Motor_Enable();

    /* 第一阶段：逐渐增加对齐电压，建立0°固定磁场 */
    Motor_Enable();
    for (int i = 1; i <= 10; i++)
    {
        float u_now = U_align * (float) i / 10.0f;
        Motor_SetPhaseVoltage_dq_SVPWM(u_now, 0, 0, Vdc);
        HAL_Delay(100U);
    }
    
    /* 等待转子稳定 */
    HAL_Delay(500U);

    /* 在驱动板使能期间读取FLT */
    Motor_FLT_State = (uint8_t)HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_10);
    /* 读取第一次对齐后的机械角度 */
    status = Motor_ReadMechanicalAngle(&theta_0);

    if(status != HAL_OK)
    {
        Motor_CalibrationStop();
        return 2;
    }

    /* 第二阶段：让磁场从0°电角度缓慢转到90° 电角度位置*/
    
    /* 1. 根据静止稳态模型确定目标电流幅值 */
    float I_align = U_align / MOTOR_RS;
    /* 2. 在初始90°位置，计算三相目标电流 */
    float Ia = 0.0f;
    float Ib = 0.8660254f * I_align;
    float Ic = -0.8660254f * I_align;
    /* 3. Clarke变换 */
    float Ialpha, Ibeta;
    Motor_Clarke(Ia, Ib, Ic, &Ialpha, &Ibeta);
    /* 4. Park变换 */
    float Id, Iq;
    Motor_Park(Ialpha, Ibeta, MOTOR_PI / 2.0f, &Id, &Iq);
    /* 5. dq物理模型：计算参考电压 */
    float Ud, Uq;
    Motor_DQModel(Id, Iq, 0.0f, 0.0f, 0.0f, &Ud, &Uq);

    for(int i = 1; i <= 9; i++)
    {
        float theta_cmd = (MOTOR_PI /2.0F) * (float)i /9.0f;
        Motor_SetPhaseVoltage_dq_SVPWM(Ud, Uq, theta_cmd, Vdc);//让磁场从0°电角度缓慢转到90° 电角度位置同时用theta_cmd做分步静止稳态近似真实的电机电角度
        HAL_Delay(300U);
    }

    /* 等待转子稳定 */
     HAL_Delay(200U);
    /*记录让转子从 α 轴位置旋转 90° 电角度此时的机械角度*/
    status = Motor_ReadMechanicalAngle(&theta_90);
    if (status != HAL_OK)
    {
        Motor_CalibrationStop();
        return 3;
    }

    /* 计算转子实际机械角度变化 */
    delta = AS5600_Diff_Rad(theta_90, theta_0);
    Motor_Calib_DeltaRad = delta;

    /* 理论机械角度变化：90° / 7 */
    expected_delta = (MOTOR_PI / 2.0f) / MOTOR_POLE_PAIRS;

    /* 检查转子是否大致跟随 */
    if(fabs(delta) < 0.5f * expected_delta || fabs(delta) > 1.5f * expected_delta)
    {
        Motor_CalibrationStop();
        return 4;
    }

    /* 确定AS5600与参考电角度的方向关系 */
    int8_t direction = (delta > 0.0f) ? 1:-1;

    /* 第四阶段：参考电压矢量从90°返回0° */
    for (int i = 8; i >= 0; i--)
    {
        float theta_cmd = (MOTOR_PI / 2.0f) * (float)i / 9.0f;

        Motor_SetPhaseVoltage_dq_SVPWM(U_align, 0, theta_cmd, Vdc);//利用d轴磁场控制电机返回0电角度

        HAL_Delay(300U);
    }

    /* 等待转子重新对齐 */
    HAL_Delay(500U);

    

    /* 读取返回0°位置后的机械角度有可能存在误差 */
    status = Motor_ReadMechanicalAngle(&theta_zero);
    if (status != HAL_OK)
    {
        Motor_CalibrationStop();
        return 5;
    }

    /* 检查转子是否大致回到初始对齐位置 */
    //float zero_error = AS5600_Diff_Rad(theta_zero, theta_0);
    //if (fabsf(zero_error) > (5.0f * MOTOR_PI / 180.0f) / MOTOR_POLE_PAIRS)
    //{
        //Motor_CalibrationStop();
        //return 6;
    //}

    /* 第五阶段：保存传感器方向及电角度零点 */

    /* 1. 关闭电机驱动 */
    Motor_CalibrationStop();

    /* 2. 保存AS5600方向 */
    Motor_Sensor_Direction = direction;

    /* 3. 保存电角度为0°时的实际机械角度 */
    Motor_ElectricZero_MechanicalAngle = theta_zero;

    /* 4. 计算电角度零点偏移 */
    Motor_Electric_Offset = Motor_WrapAngle_Rad((float)Motor_Sensor_Direction *MOTOR_POLE_PAIRS * Motor_ElectricZero_MechanicalAngle);

    /* 5. 标记校准成功 */
    Motor_Electric_Calibrated = 1;

    return 0;
}


/*
    Motor_GetMechanicalAngle_Rad()：
    获取基于定子A相磁轴为零点、逆时针为正的机械角度，单位rad。
*/
float Motor_GetMechanicalAngle_Rad(void)
{
    AS5600_AngelUpdate_Rad();
    float theta_m = (float)Motor_Sensor_Direction * (Motor_Angle - Motor_ElectricZero_MechanicalAngle);
    return theta_m;
}

/*
    Motor_GetElectricAngle_Rad()：
    获取基于转子d轴与定子A相磁轴重合为零点的电角度，单位rad。
*/
float Motor_GetElectricAngle_Rad(void)
{
    float theta_m = Motor_GetMechanicalAngle_Rad();
    return MOTOR_POLE_PAIRS * theta_m;
}


/*
 * 基于PMSM dq物理模型的电压前馈FOC控制
 *
 * id、iq       ：目标dq轴电流，单位A
 * dId、dIq     ：目标dq轴电流变化率，单位A/s
 * theta_e      ：转子实际电角度，单位rad
 * omega_e      ：转子实际电角速度，单位rad/s
 * Vdc          ：直流母线电压，单位V
 *
 * 注意：当前没有电流反馈，属于开环控制。
 */
 void Motor_VoltageFOC(float id, float iq,float dId, float dIq, float theta_e, float omega_e, float Vdc)
 {
    float Ud,Uq;
    Motor_DQModel(id, iq, dId, dIq, omega_e, &Ud, &Uq);
    Motor_SetPhaseVoltage_dq_SVPWM(Ud, Uq, theta_e, Vdc);
 }

 void Motor_Init(void)
 {
    /* 关闭电机驱动 */
    Motor_Disable();

    /* 启动三路PWM */
    Motor_Pwm_Start();
    
    /* 等待转子停止及外部硬件状态稳定 */
    HAL_Delay(1000U);
    /* 执行电角度校准 */
    Motor_Calib_Result = Motor_CalibrationElectricAngle(0.5f, 12.0f);

    /* 校准成功后初始化AS5600 */
    if (Motor_Calib_Result == 0U)
    {
        AS5600_AngleInit_Rad();
    } 

 }

/**
 * @brief 获取SVPWM几何扇区（从0°逆时针编号1~6）
 * @param Ualpha alpha轴电压分量
 * @param Ubeta  beta轴电压分量
 * @retval 几何扇区号1~6，0表示无效编码
 */
static uint8_t Motor_Get_Sector(float Ualpha, float Ubeta)
{
    const float SQRT3 = 1.7320508f;
    uint8_t N;
    /* 零电压矢量方向任意，默认归入第一扇区 */
    if (Ualpha == 0.0f && Ubeta == 0.0f)
    {
        return 1U;
    }

    /* 三条直线的判别结果 */
    float L2 = Ubeta - SQRT3 * Ualpha;
    float L3 = Ubeta + SQRT3 * Ualpha;

    /* 边界归入逆时针方向的下一个扇区 */
    uint8_t A = (Ubeta > 0.0f ||(Ubeta == 0.0f && Ualpha > 0.0f)) ? 1U : 0U;

    uint8_t B = (L2 > 0.0f ||(L2 == 0.0f && Ualpha > 0.0f)) ? 1U : 0U;

    uint8_t C = (L3 > 0.0f ||(L3 == 0.0f && Ualpha > 0.0f)) ? 1U : 0U;

    /* 计算三位二进制编码 */
    N = A + 2U * B + 4U * C;
    return N;

}