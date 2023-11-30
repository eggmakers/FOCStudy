#ifndef __BSP_AS5600_H
#define __BSP_AS5600_H

#include "i2c.h"

#define abs(x) ((x) > 0 ? (x) : -(x))
#define _2PI 6.28318530718
#define _PI 3.14159265359

#define AS5600_I2C_HANDLE hi2c1

#define I2C_TIME_OUT_BASE 10
#define I2C_TIME_OUT_BYTE 1

/*
ע��:AS5600�ĵ�ַ0x36��ָ����ԭʼ7λ�豸��ַ,��ST I2C���е��豸��ַ��ָԭʼ�豸��ַ����һλ�õ����豸��ַ
*/

#define AS5600_RAW_ADDR 0x36
#define AS5600_ADDR (AS5600_RAW_ADDR << 1)
#define AS5600_WRITE_ADDR (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR ((AS5600_RAW_ADDR << 1) | 1)

#define AS5600_RESOLUTION 4096 // 12bit Resolution

#define AS5600_RAW_ANGLE_REGISTER 0x0C

void bsp_as5600Init(void);
uint16_t bsp_as5600GetRawAngle(void);
float bsp_as5600GetAngle(void);
float GetAngle(void);
float GetAngle_without_track(void);

#endif /* __BSP_AS5600_H */
