#include "as5600.h"
#include "usart.h"
#include "i2c.h"
#include <stdio.h>
#include "gpio.h"

float Motor_Angle;
float Motor_Angle_Now;
float Motor_Angle_Last;




HAL_StatusTypeDef AS5600_ReadRawData(I2C_HandleTypeDef *hi2c, uint16_t *raw_angle_data)
{
    uint8_t temp[2];
    HAL_StatusTypeDef Status;

    if (hi2c == NULL || raw_angle_data == NULL) 
    {
        return HAL_ERROR;
    }

    Status = HAL_I2C_Mem_Read(hi2c, AS5600_ADDR<<1 , AS5600_OUTPUT_HIGH_REG, I2C_MEMADD_SIZE_8BIT, temp, 2, 100);
    if (Status != HAL_OK) 
    {
        return Status;          // 把 HAL_BUSY / HAL_TIMEOUT / HAL_ERROR 原样带给调用方
    }

    *raw_angle_data = ((uint16_t)temp[0] << 8) | temp[1];
    return Status;
}




/*
    AS5600_AngleDataChange()：将原始计数转换成角度（°）
*/
float AS5600_AngleDataChange_Degree(uint16_t raw_data)
{
    float angle;

    angle = (float) raw_data * 360.0f / 4096.0f ;

    return angle;
}



/*
    AS5600_AngleDataChange_Rad()：将原始计数转换成弧度（rad）
*/
float AS5600_AngleDataChange_Rad(uint16_t raw_data)
{
    float angle;

    angle = (float) raw_data * 360.0f  / 4096.0f *( pi / 180.0f);

    return angle;
}


/*
    AS5600_UsartTransmitAngle()：保留两位有效数字并进行USART1串口传递
*/
void AS5600_UsartTransmitAngle(float angle_data)
{
    char tx_buf[32];//数组名在大多数表达式中会自动转换为指向首元素的指针，所以不需要再加 &
    //*tx_buf：解引用
    //char tx_buf[4] = {'A', 'B', 'C', 'D'};
    //char a = *tx_buf;读取第一个元素，即 'A'
    /********************************/
    //char *q = &tx_buf[0];
    //q = &tx_buf[0];  // q 中保存地址 0x2000
    uint16_t angle_100 = (uint16_t)(angle_data * 100.0f + 0.5f);//保留两位有效数字并转换为无符号整数类型
    int len = snprintf(tx_buf, sizeof(tx_buf), "angle: %lu.%02lu\r\n", (unsigned long)(angle_100 / 100U), (unsigned long)(angle_100 % 100U));
    /* 
    angle_100 / 100U 其结果为整数商
    27721 / 100U  结果为277，取商操作
    27721 % 100U   结果为21，取余数操作
    "%lu.%lu"12.5 "%lu.%02lu" 12.05
    */

    HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, len, 100);
    //HAL_UART_Transmit()发送的是字节数组
    //将 tx_buf 转换成 uint8_t * 类型的指针, 此前tx_buf默认为指针类型。

}



/*
    AS5600_AngleInit_Rad()：读取AS5600初始机械角度（单位rad），
    初始化机械角度跟踪变量，为后续机械角度累计计算做准备。
*/
void AS5600_AngleInit_Rad(void)
{
    HAL_Delay(100);
    uint16_t raw_angle_data;

    if (AS5600_ReadRawData(&hi2c2, &raw_angle_data) == HAL_OK)
    {
        Motor_Angle = AS5600_AngleDataChange_Rad(raw_angle_data);
        Motor_Angle_Now = Motor_Angle;
        Motor_Angle_Last = 0.0f;
    }
}

/*
    AS5600_Diff_Degree()：计算相邻两次机械角度增量
    data_now：当前单圈机械角度，单位°
    data_last：上一次单圈机械角度，单位°
*/
float AS5600_Diff_Degree(float data_now, float data_last)
{

    float data_diff = data_now -  data_last;

    if (data_diff < -180.0f)
    {
        data_diff = data_diff + 360.0f;
    }
    else if(data_diff > 180.0f)
    {
        data_diff = data_diff - 360.0f;
    }
    
    return data_diff;
    
}

/*
    AS5600_Diff_Rad()：计算相邻两次机械角度增量
    data_now：当前单圈机械角度，单位rad
    data_last：上一时刻单圈机械角度，单位rad
    返回值：机械角度增量，单位rad
*/
float AS5600_Diff_Rad(float data_now, float data_last)
{

    float data_diff = data_now -  data_last;

    if (data_diff < -pi)
    {
        data_diff = data_diff + 2*pi;
    }
    else if(data_diff > pi)
    {
        data_diff = data_diff - 2*pi;
    }
    
    return data_diff;
    
}

/*
    角度展开（Angle Unwrapping）：保存上一次角度，计算本次角度相对于上一次的真实变化量，再不断累加。
*/
void AS5600_AngelUpdate_Rad(void)
{
    uint16_t data;
    float diff;
    if(AS5600_ReadRawData(&hi2c2 , &data) == HAL_OK)
    {
            Motor_Angle_Last = Motor_Angle_Now;
            Motor_Angle_Now = AS5600_AngleDataChange_Rad(data);
            diff = AS5600_Diff_Rad(Motor_Angle_Now , Motor_Angle_Last);
            Motor_Angle = Motor_Angle + diff;            
    }
    
}





