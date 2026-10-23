#ifndef __MPU6050_H
#define __MPU6050_H
//#define MPU6050_ADDR 0x68

#include "stm32f10x.h"
#include <stdint.h>

/* =====================================================
 * MPU6050 六轴传感器驱动（软件 I2C 实现）
 * SCL -> PB6   SDA -> PB7
 * 本驱动只读取原始寄存器数据，滤波在 main.c 中完成
 * ===================================================== */

#define mpu6050_ADDR    0xD0    /* AD0=0 时器件地址（含读写位） */
uint8_t mpu6050_Init(void);                                  /* 初始化：唤醒、设置量程与采样率 */
void   mpu6050_ReadRaw(int16_t* accx, int16_t* accy, int16_t* accz,
                       int16_t* gyrx, int16_t* gyry, int16_t* gyrz); /* 读取加速度与陀螺仪原始值 */

#endif
