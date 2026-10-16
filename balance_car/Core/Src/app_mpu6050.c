#include "app_mpu6050.h"
#include "i2c.h"       // CubeMX 生成的，里面有 hi2c2
#include "main.h"
#include "task.h"      // PERIODIC 宏
#include "math.h"

// MPU6050 的 7 位 I2C 地址
#define MPU6050_ADDR   0x68

// 滑动平均窗口大小（越大越平滑，但响应越慢）
#define FILTER_SIZE    5

// 静态变量：保存转换后的物理量
static float ax, ay, az;          // 加速度，单位 g
static float temperature;         // 温度，单位摄氏度
static float gx, gy, gz;          // 角速度，单位度/秒
static float yaw, pitch, roll;    // 欧拉角，单位度

// 滑动平均缓冲区
static float ax_buf[FILTER_SIZE] = {0};
static float ay_buf[FILTER_SIZE] = {0};
static float az_buf[FILTER_SIZE] = {0};
static float gx_buf[FILTER_SIZE] = {0};
static float gy_buf[FILTER_SIZE] = {0};
static float gz_buf[FILTER_SIZE] = {0};

static void reg_write(uint8_t reg, uint8_t value);
static uint8_t reg_read(uint8_t reg);

/**
  * @brief  向 MPU6050 写寄存器
  * @param  reg:   寄存器地址
  * @param  value: 要写入的值
  * @note   用 I2C2（PB10/PB11），直接调 HAL 库
  */
static void reg_write(uint8_t reg, uint8_t value)
{
    // MPU6050_ADDR 左移一位：HAL 库要求 8 位地址（7 位地址 << 1）
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, reg,
                      I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

/**
  * @brief  读 MPU6050 寄存器
  * @param  reg: 寄存器地址
  * @return 读到的值
  */
static uint8_t reg_read(uint8_t reg)
{
    uint8_t value = 0;
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR << 1, reg,
                     I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
    return value;
}

/**
  * @brief  初始化 MPU6050
  * @note   必须在 MX_I2C2_Init() 之后调用
  */
void App_MPU6050_Init(void)
{
    reg_write(0x6B, 0x80);       // 复位 MPU6050
    HAL_Delay(100);              // 等复位完成
    reg_write(0x6B, 0x00);       // 唤醒（退出休眠模式）
    reg_write(0x1B, 0x18);       // 陀螺仪量程 ±2000°/s
    reg_write(0x1C, 0x00);       // 加速度计量程 ±2g
}

/**
  * @brief  MPU6050 进程函数
  * @note   每 5ms 执行一次
  *         互补滤波：陀螺仪积分 + 加速度计修正
  */
void App_MPU6050_Proc(void)
{
    PERIODIC(5)                  // 每 5ms 执行一次

    App_MPU6050_Update();        // 更新传感器的值

    // 通过陀螺仪的测量结果积分算欧拉角（短期准，会漂）
    float yaw_g   = yaw + gz * 0.005f;
    float pitch_g = pitch + gx * 0.005f;
    float roll_g  = roll  - gy * 0.005f;

    // 通过加速度计计算欧拉角（长期准，受振动影响）
    float pitch_a = atan2(ay, az) / 3.1415927f * 180.0f;
    float roll_a  = atan2(ax, az) / 3.1415927f * 180.0f;

    // 互补滤波：融合陀螺仪和加速度计
    // 陀螺仪占 95.238%，加速度计占 4.762%
    yaw   = yaw_g;
    pitch = 0.95238f * pitch_g + (1 - 0.95238f) * pitch_a;
    roll  = 0.95238f * roll_g  + (1 - 0.95238f) * roll_a;
}

/**
  * @brief  更新 MPU6050 数据
  * @note   读原始数据，转换成物理量，做滑动平均
  *         滑动平均：把最近 FILTER_SIZE 次的数据求平均，减小抖动
  */
void App_MPU6050_Update(void)
{
    // ---- 读加速度原始数据（高字节 + 低字节）----
    int16_t ax_raw = (int16_t)((reg_read(0x3B) << 8) + reg_read(0x3C));
    int16_t ay_raw = (int16_t)((reg_read(0x3D) << 8) + reg_read(0x3E));
    int16_t az_raw = (int16_t)((reg_read(0x3F) << 8) + reg_read(0x40));

    // ---- 读温度原始数据 ----
    int16_t temperature_raw = (int16_t)((reg_read(0x41) << 8) + reg_read(0x42));
    temperature = temperature_raw / 340.0f + 36.53f;

    // ---- 读角速度原始数据 ----
    int16_t gx_raw = (int16_t)((reg_read(0x43) << 8) + reg_read(0x44));
    int16_t gy_raw = (int16_t)((reg_read(0x45) << 8) + reg_read(0x46));
    int16_t gz_raw = (int16_t)((reg_read(0x47) << 8) + reg_read(0x48));

    // ---- 转换成物理量 ----
    // ±2g 量程下，1g = 16384 LSB，所以乘以 1/16384 = 6.1035e-5
    float ax_new = ax_raw * 6.1035e-5f;
    float ay_new = ay_raw * 6.1035e-5f;
    float az_new = az_raw * 6.1035e-5f;

    // ±2000°/s 量程下，1°/s = 16.4 LSB，所以乘以 1/16.4 = 6.1035e-2
    float gx_new = gx_raw * 6.1035e-2f;
    float gy_new = gy_raw * 6.1035e-2f;
    float gz_new = gz_raw * 6.1035e-2f;

    // ---- 滑动平均 ----
    // 1. 数据整体左移，空出最后一个位置
    for(int i = 0; i < FILTER_SIZE - 1; i++)
    {
        ax_buf[i] = ax_buf[i+1];
        ay_buf[i] = ay_buf[i+1];
        az_buf[i] = az_buf[i+1];
        gx_buf[i] = gx_buf[i+1];
        gy_buf[i] = gy_buf[i+1];
        gz_buf[i] = gz_buf[i+1];
    }

    // 2. 新值放到最后一个位置
    ax_buf[FILTER_SIZE-1] = ax_new;
    ay_buf[FILTER_SIZE-1] = ay_new;
    az_buf[FILTER_SIZE-1] = az_new;
    gx_buf[FILTER_SIZE-1] = gx_new;
    gy_buf[FILTER_SIZE-1] = gy_new;
    gz_buf[FILTER_SIZE-1] = gz_new;

    // 3. 求平均值
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    for(int i = 0; i < FILTER_SIZE; i++)
    {
        ax_sum += ax_buf[i];
        ay_sum += ay_buf[i];
        az_sum += az_buf[i];
        gx_sum += gx_buf[i];
        gy_sum += gy_buf[i];
        gz_sum += gz_buf[i];
    }

    // 4. 赋给全局变量
    ax = ax_sum / FILTER_SIZE;
    ay = ay_sum / FILTER_SIZE;
    az = az_sum / FILTER_SIZE;

    gx = gx_sum / FILTER_SIZE;
    gy = gy_sum / FILTER_SIZE;
    gz = gz_sum / FILTER_SIZE;
}

// 加速度获取
float App_MPU6050_GetAx(void) { return ax; }
float App_MPU6050_GetAy(void) { return ay; }
float App_MPU6050_GetAz(void) { return az; }

// 温度获取
float App_MPU6050_GetTemperature(void) { return temperature; }

// 角速度获取
float App_MPU6050_GetGx(void) { return gx; }
float App_MPU6050_GetGy(void) { return gy; }
float App_MPU6050_GetGz(void) { return gz; }

// 欧拉角获取
float App_MPU6050_GetYaw(void)   { return yaw; }
float App_MPU6050_GetPitch(void) { return pitch; }
float App_MPU6050_GetRoll(void)  { return roll; }