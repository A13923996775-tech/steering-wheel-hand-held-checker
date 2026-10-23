#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>

#define SAMPLE_MS          20      /* 采样周期（毫秒）*/
#define HAND_OFF_TIMEOUT_S 2       /* 脱手报警延时（秒）*/
#define VIRTUAL_GRIP_S     3       /* 虚握预警延时（秒）*/
#define GYRO_THRESHOLD     100      /* 转向判定阈值：z 轴角速度（LSB，约 1.2°/s）*/
#define TOUCH_ACTIVE_LEVEL 0       /* TTP223 触摸有效电平（低有效用 0）*/
#define SAMPLE_DECIMATION  2       /* 低功耗：非报警态每 2 次采样才读一次 IMU */

typedef enum {
    ST_NORMAL,      /* 正常握持驾驶 */
    ST_VIRTUAL,     /* 虚握预警 */
    ST_HAND_OFF     /* 脱手报警 */
} SystemState;

static SystemState sys_state = ST_NORMAL;

/* 触摸相关 */
static uint8_t touch_left  = 0;   /* 左握区触摸标志 */
static uint8_t touch_right = 0;   /* 右握区触摸标志 */

/* 滤波相关：z 轴角速度滑动平均（窗口 8） */
#define GYRO_AVG_N  8
static int32_t gyrz_avg_buf[GYRO_AVG_N];
static uint8_t gyrz_avg_idx = 0;

/* 计时相关 */
static uint32_t handoff_tick = 0;   /* 脱手累计毫秒 */
static uint32_t virtual_tick = 0;   /* 虚握累计毫秒 */
static uint32_t sys_tick     = 0;   /* 全局毫秒计时（示意，实际可挂 SysTick）*/

static void DelayMs(uint32_t ms)
{
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 6000; j++);
}

static void GPIO_Config(void)
{
    GPIO_InitTypeDef gpio;

    /* 使能 GPIOA / GPIOB / AFIO */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO, ENABLE);

    /* 触摸输入（PA0、PA1）带上拉：未触摸时读到高电平 */
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOA, &gpio);

    /* LED（PB0~PB2）与蜂鸣器（PB3）推挽输出 */
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_10 | GPIO_Pin_3;
    GPIO_Init(GPIOB, &gpio);
}

#define LED_GREEN  GPIO_Pin_0
#define LED_YELLOW GPIO_Pin_1
#define LED_RED    GPIO_Pin_10
#define BUZZER     GPIO_Pin_3

static void LedsOff(void)
{
    GPIO_ResetBits(GPIOB, LED_GREEN | LED_YELLOW | LED_RED);
    GPIO_ResetBits(GPIOB, BUZZER);
}

#define TOUCH_DEBOUNCE_N 3
static uint8_t ReadTouch(void)
{
    uint8_t t_l, t_r, i, cnt_l, cnt_r;
    /* 简化：统计多次采样中有效次数，达到多数即判定有效（防抖） */
    cnt_l = cnt_r = 0;
    for (i = 0; i < TOUCH_DEBOUNCE_N; i++) {
        t_l = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == TOUCH_ACTIVE_LEVEL);
        t_r = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == TOUCH_ACTIVE_LEVEL);
        cnt_l += t_l;
        cnt_r += t_r;
        DelayMs(1);
    }
    /* 至少采样次数的一半以上为有效，才认定为接触 */
    touch_left  = (cnt_l >= (TOUCH_DEBOUNCE_N + 1) / 2);
    touch_right = (cnt_r >= (TOUCH_DEBOUNCE_N + 1) / 2);
    return (touch_left || touch_right);   /* 任一握区接触即视为手握 */
}

static float GyroLowPass(float in, float prev, float alpha)
{
    return alpha * in + (1.0f - alpha) * prev;   /* 一阶低通：alpha 越小越平滑 */
}

static uint8_t DetectTurning(int16_t gyrz_raw)
{
    static float gyro_filtered = 0.0f;
    int32_t sum = 0;
    uint8_t i;

    /* 一阶低通滤波 */
    gyro_filtered = GyroLowPass((float)gyrz_raw, gyro_filtered, 0.5f);

    /* 滑动平均 */
    gyrz_avg_buf[gyrz_avg_idx] = (int32_t)gyro_filtered;
    gyrz_avg_idx = (gyrz_avg_idx + 1) % GYRO_AVG_N;
    for (i = 0; i < GYRO_AVG_N; i++)
        sum += gyrz_avg_buf[i];
    sum /= GYRO_AVG_N;

    /* 取绝对值与阈值比较：|角速度| 超过阈值视为有转向动作 */
    if (sum < 0) sum = -sum;
    return (sum > GYRO_THRESHOLD) ? 1 : 0;
}


static void UpdateSystem(uint8_t touch_on, uint8_t turning)
{
    switch (sys_state) {

    case ST_NORMAL:
        if (touch_on && turning) {
            /* 双条件同时满足：正常握持驾驶 */
            LedsOff();
            GPIO_SetBits(GPIOB, LED_GREEN);
						GPIO_ResetBits(GPIOB, LED_YELLOW);
						GPIO_ResetBits(GPIOB, LED_RED);
						GPIO_ResetBits(GPIOB, BUZZER);
            handoff_tick = 0;
            virtual_tick = 0;
        } else if (touch_on && !turning) {
            /* 触摸但不转向：进入虚握状态 */
						LedsOff();
            sys_state = ST_VIRTUAL;
            virtual_tick = 0;
        } else if (!touch_on) {
            /* 完全脱手：进入脱手计时 */
						LedsOff();
            sys_state = ST_HAND_OFF;
            handoff_tick = 0;
        }
        break;

    case ST_VIRTUAL:
        if (!touch_on) {
            /* 虚握期间手又离开：转为脱手 */
            sys_state = ST_HAND_OFF;
            handoff_tick = 0;
        } else if (turning) {
            /* 虚握期间恢复转向：回到正常 */
            sys_state = ST_NORMAL;
            virtual_tick = 0;
        } else {
            /* 持续虚握：累计时间，超时黄灯预警 */
            virtual_tick += SAMPLE_MS;
						GPIO_ResetBits(GPIOB, LED_GREEN);
						GPIO_ResetBits(GPIOB, LED_RED);
						GPIO_ResetBits(GPIOB, BUZZER);
						
            if (virtual_tick >= VIRTUAL_GRIP_S * 1000UL) {
                /* 黄灯闪烁预警（约 2Hz） */
                LedsOff();
                if ((sys_tick / 250) % 2) GPIO_SetBits(GPIOB, LED_YELLOW);
                else                     GPIO_ResetBits(GPIOB, LED_YELLOW);
            }
        }
        break;

    case ST_HAND_OFF:
        if (touch_on) {
            /* 重新握住：回到正常 */
            sys_state = ST_NORMAL;
            handoff_tick = 0;
        } else {
            /* 持续脱手：累计时间，超过 2 秒红灯 + 蜂鸣 */
            handoff_tick += SAMPLE_MS;
            if (handoff_tick >= HAND_OFF_TIMEOUT_S * 1000UL) {
                LedsOff();
								GPIO_ResetBits(GPIOB, LED_GREEN);
								GPIO_ResetBits(GPIOB, LED_YELLOW);
								GPIO_SetBits(GPIOB, LED_RED);
								GPIO_SetBits(GPIOB, BUZZER);
            }
        }
        break;
    }
}



int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitsStructure;
	GPIO_InitsStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitsStructure.GPIO_Pin = GPIO_Pin_All;//member
	GPIO_InitsStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitsStructure);
	
	GPIO_ResetBits(GPIOB, LED_YELLOW);
	GPIO_ResetBits(GPIOB, LED_RED);
	GPIO_ResetBits(GPIOB, BUZZER);
	GPIO_ResetBits(GPIOB, LED_GREEN);
	
	
	uint8_t touch_on, turning;
    uint16_t sample_cnt = 0;
    int16_t ax, ay, az, gx, gy, gz;

    GPIO_Config();
    mpu6050_Init();

    while (1) {
        /* 低功耗管理：非报警态降低采样频率（示例用分频，实际可用 STOP 模式）*/
        sample_cnt++;
        if (sys_state != ST_HAND_OFF && (sample_cnt % SAMPLE_DECIMATION) != 0) {
            DelayMs(SAMPLE_MS);
            sys_tick += SAMPLE_MS;
            continue;
        }

        /* 1. 读取触摸（含防抖） */
        touch_on = ReadTouch();

        /* 2. 读取 IMU 并判断是否有转向动作 */
        mpu6050_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz);
        turning = DetectTurning(gz);

        /* 3. 双条件状态机判定 */
        UpdateSystem(touch_on, turning);

        /* 4. 按采样周期推进 */
        DelayMs(SAMPLE_MS);
        sys_tick += SAMPLE_MS;
    }
}
