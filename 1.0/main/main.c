#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "mpu6050.h"

#define SAMPLE_MS          20      /* 采样周期（毫秒）*/
#define HAND_OFF_TIMEOUT_S 2       /* 脱手报警延时（秒）*/
#define GYRO_THRESHOLD     100     /* 转向判定阈值：z 轴角速度（LSB，约 1.5°/s）*/
#define TOUCH_ACTIVE_LEVEL 1       /* TTP223 触摸时为高电平 */
#define TOUCH_DEBOUNCE_N   3       /* 触摸防抖采样次数 */
#define GYRO_AVG_N         3       /* 陀螺仪滑动平均窗口（20ms×3=60ms）*/
#define BLINK_HALF_MS      250     /* 2Hz 闪烁的半周期（毫秒）*/

typedef enum {
    ST_NORMAL,          /* 正常握持：有触摸 + 有转动 */
    ST_VIRTUAL,         /* 虚握预警：有触摸 + 无转动 */
    ST_HANDOFF_PRE,     /* 脱手预警：无触摸，未满 2 秒 */
    ST_HANDOFF_ALARM    /* 脱手报警：无触摸，已满 2 秒 */
} SystemState;

static SystemState sys_state = ST_VIRTUAL;

static uint8_t touch_left  = 0;
static uint8_t touch_right = 0;

/* 陀螺仪滤波 */
static int32_t gyrz_avg_buf[GYRO_AVG_N];
static uint8_t gyrz_avg_idx = 0;

static uint32_t sys_tick = 0;
static uint32_t handoff_tick = 0;
static uint8_t  handoff_active = 0;

#define LED_GREEN  GPIO_Pin_0
#define LED_YELLOW GPIO_Pin_1
#define LED_RED    GPIO_Pin_10
#define BUZZER     GPIO_Pin_3

static void AppDelayMs(uint32_t ms)
{
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 6000; j++);
}

static void SetOutputs(uint8_t green, uint8_t yellow, uint8_t red, uint8_t buzzer)
{
    uint32_t set_high = 0;
    uint32_t set_low  = 0;

    if (green)  set_low |= LED_GREEN;   else set_high |= LED_GREEN;
    if (yellow) set_low |= LED_YELLOW;  else set_high |= LED_YELLOW;
    if (red)    set_low |= LED_RED;     else set_high |= LED_RED;
    if (buzzer) set_low |= BUZZER; else set_high |= BUZZER;
    

    GPIOB->BSRR = (set_low << 16) | set_high;
}

static void ApplyOutputs(SystemState state)
{
    uint8_t blink_on = (((sys_tick / BLINK_HALF_MS) % 2U) == 0U);
    switch (state) {
    case ST_NORMAL:
        SetOutputs(1, 0, 0, 0);
        break;
    case ST_VIRTUAL:
    case ST_HANDOFF_PRE:
        SetOutputs(0, blink_on ? 1U : 0U, 0, 0);
        break;
    case ST_HANDOFF_ALARM:
        SetOutputs(0, 0, 1, 1);
        break;
    default:
        SetOutputs(0, 1, 0, 0);
        break;
    }
}

static void GPIO_Config(void)
{
    GPIO_InitTypeDef gpio;
    /* 打开GPIOA / GPIOB / AFIO的时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO, ENABLE);
    /* 禁用 SWJ JTAG，释放 PB3 作为蜂鸣器输出 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    /* TTP223 触摸时为高电平，PA0/PA1 改为下拉输入 */
    gpio.GPIO_Mode  = GPIO_Mode_IPD;
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
		/* 初始化GPIOA */
    GPIO_Init(GPIOA, &gpio);
    /* 先预置黄灯 ODR，再切换为输出，避免输出使能瞬间三灯全灭 */
    GPIOB->BSRR = LED_YELLOW;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin   = LED_GREEN | LED_YELLOW | LED_RED | BUZZER;
    GPIO_Init(GPIOB, &gpio);
    /* 上电默认黄灯常亮，第一次采样后立即进入实际状态 */
    SetOutputs(0, 1, 0, 0);
}

/* 触摸 3 次防抖：每次间隔约 1ms，2/3 有效即判定触摸 */
static uint8_t ReadTouch(void)
{
    uint8_t i;
    uint8_t cnt_l = 0;
    uint8_t cnt_r = 0;
    for (i = 0; i < TOUCH_DEBOUNCE_N; i++) {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == TOUCH_ACTIVE_LEVEL)
            cnt_l++;
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == TOUCH_ACTIVE_LEVEL)
            cnt_r++;
        AppDelayMs(1);
    }
    touch_left  = (cnt_l >= (TOUCH_DEBOUNCE_N + 1) / 2) ? 1U : 0U;
    touch_right = (cnt_r >= (TOUCH_DEBOUNCE_N + 1) / 2) ? 1U : 0U;
    /* 左右任一握区有触摸，即视为手握 */
    return (touch_left || touch_right) ? 1U : 0U;
}

/* 一阶低通滤波：alpha 越小越平滑，但响应越慢 */
static float GyroLowPass(float in, float prev, float alpha)
{
    return alpha * in + (1.0f - alpha) * prev;
}

/* 一阶低通 + 3 点滑动平均，再与阈值比较 */
static uint8_t DetectTurning(int16_t gyrz_raw)
{
    static float gyro_filtered = 0.0f;
    int32_t sum = 0;
    uint8_t i;
    gyro_filtered = GyroLowPass((float)gyrz_raw, gyro_filtered, 0.5f);
    gyrz_avg_buf[gyrz_avg_idx] = (int32_t)gyro_filtered;
    gyrz_avg_idx = (gyrz_avg_idx + 1) % GYRO_AVG_N;
    for (i = 0; i < GYRO_AVG_N; i++)
        sum += gyrz_avg_buf[i];
    sum /= GYRO_AVG_N;
    /* 取绝对值与阈值比较：|角速度| 超过阈值视为有转动 */
    if (sum < 0) sum = -sum;
    return (sum > GYRO_THRESHOLD) ? 1U : 0U;
}

static void UpdateSystem(uint8_t touch_on, uint8_t turning)
{
    if (touch_on) {
        handoff_active = 0;
        if (turning) {
            sys_state = ST_NORMAL;
        } else {
            sys_state = ST_VIRTUAL;
        }
    } else {
        if (!handoff_active) {
            handoff_active = 1;
            handoff_tick = sys_tick;
        }
        if ((sys_tick - handoff_tick) >= HAND_OFF_TIMEOUT_S * 1000UL) {
            sys_state = ST_HANDOFF_ALARM;
        } else {
            sys_state = ST_HANDOFF_PRE;
        }
    }
    ApplyOutputs(sys_state);
}

int main(void)
{
    uint8_t touch_on, turning;
    int16_t ax, ay, az, gx, gy, gz;
    GPIO_Config();
    /* 给 MPU6050 上电稳定时间，避免 WHO_AM_I 读失败 */
    AppDelayMs(100);
    /* 恢复 IMU 初始化；失败则红灯 + 蜂鸣器响，提示检查接线 */
    if (mpu6050_Init() != 0) {
        SetOutputs(0, 0, 1, 1);
        AppDelayMs(500);
    }
    while (1) {
        /* 读取触摸状态 */
        touch_on = ReadTouch();
        /* 读取 MPU6050，判断本次采样是否有转动 */
        mpu6050_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz);
        turning = DetectTurning(gz);
        /* 根据当前触摸/转动状态刷新输出 */
        UpdateSystem(touch_on, turning);
        /* 保持 20ms 采样周期 */
        AppDelayMs(SAMPLE_MS);
        sys_tick += SAMPLE_MS;
    }
}
