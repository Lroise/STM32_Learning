#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "main.h"
#include "delay.h"
#include "uart.h"
#include "mpuiic.h"
#include "OLED.h"  

#define MPU6050                         
#define MOTION_DRIVER_TARGET_MSP430     

#if defined MOTION_DRIVER_TARGET_MSP430
#define i2c_write   MPU_Write_Len
#define i2c_read    MPU_Read_Len
#define delay_ms    HAL_Delay  // 【关键】强制使用官方延时，绝对不卡死
#define get_ms      mget_ms
#define log_i 		printf	
#define log_e  		printf	
#define fabs        fabsf
#define min(a,b) ((a<b)?a:b)
#else
#error  Gyro driver is missing the system layer implementations.
#endif

#if !defined MPU6050 && !defined MPU9150 && !defined MPU6500 && !defined MPU9250
#error  Which gyro are you using? Define MPUxxxx in your compiler options.
#endif

static int set_int_enable(unsigned char enable);

struct gyro_reg_s {
    unsigned char who_am_i;
    unsigned char rate_div;
    unsigned char lpf;
    unsigned char prod_id;
    unsigned char user_ctrl;
    unsigned char fifo_en;
    unsigned char gyro_cfg;
    unsigned char accel_cfg;
    unsigned char motion_thr;
    unsigned char motion_dur;
    unsigned char fifo_count_h;
    unsigned char fifo_r_w;
    unsigned char raw_gyro;
    unsigned char raw_accel;
    unsigned char temp;
    unsigned char int_enable;
    unsigned char dmp_int_status;
    unsigned char int_status;
    unsigned char pwr_mgmt_1;
    unsigned char pwr_mgmt_2;
    unsigned char int_pin_cfg;
    unsigned char mem_r_w;
    unsigned char accel_offs;
    unsigned char i2c_mst;
    unsigned char bank_sel;
    unsigned char mem_start_addr;
    unsigned char prgm_start_h;
};

struct hw_s {
    unsigned char addr;
    unsigned short max_fifo;
    unsigned char num_reg;
    unsigned short temp_sens;
    short temp_offset;
    unsigned short bank_size;
};

struct motion_int_cache_s {
    unsigned short gyro_fsr;
    unsigned char accel_fsr;
    unsigned short lpf;
    unsigned short sample_rate;
    unsigned char sensors_on;
    unsigned char fifo_sensors;
    unsigned char dmp_on;
};

struct chip_cfg_s {
    unsigned char gyro_fsr;
    unsigned char accel_fsr;
    unsigned char sensors;
    unsigned char lpf;
    unsigned char clk_src;
    unsigned short sample_rate;
    unsigned char fifo_enable;
    unsigned char int_enable;
    unsigned char bypass_mode;
    unsigned char accel_half;
    unsigned char lp_accel_mode;
    unsigned char int_motion_only;
    struct motion_int_cache_s cache;
    unsigned char active_low_int;
    unsigned char latched_int;
    unsigned char dmp_on;
    unsigned char dmp_loaded;
    unsigned short dmp_sample_rate;
};

struct test_s {
    unsigned long gyro_sens;
    unsigned long accel_sens;
    unsigned char reg_rate_div;
    unsigned char reg_lpf;
    unsigned char reg_gyro_fsr;
    unsigned char reg_accel_fsr;
    unsigned short wait_ms;
    unsigned char packet_thresh;
    float min_dps;
    float max_dps;
    float max_gyro_var;
    float min_g;
    float max_g;
    float max_accel_var;
};

struct gyro_state_s {
    const struct gyro_reg_s *reg;
    const struct hw_s *hw;
    struct chip_cfg_s chip_cfg;
    const struct test_s *test;
};

enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};

enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

const struct gyro_reg_s reg = {
0x75, 0x19, 0x1A, 0x0C, 0x6A, 0x23, 0x1B, 0x1C, 0x1F, 0x20, 0x72, 0x74,
0x43, 0x3B, 0x41, 0x38, 0x39, 0x3A, 0x6B, 0x6C, 0x37, 0x6F, 0x06, 0x24,
0x6D, 0x6E, 0x70
};

const struct hw_s hw={
  0x68, 1024, 118, 340, -521, 256
};

const struct test_s test={
32768/250, 32768/16, 0, 1, 0, 0x18, 50, 5, 10.0f, 105.0f, 0.14f, 0.3f, 0.95f, 0.14f
};

static struct gyro_state_s st={
  &reg, &hw, {0}, &test
};

#define MAX_PACKET_LENGTH (12)

static int set_int_enable(unsigned char enable)
{
    unsigned char tmp;
    if (st.chip_cfg.dmp_on) {
        if (enable) tmp = BIT_DMP_INT_EN;
        else tmp = 0x00;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp)) return -1;
        st.chip_cfg.int_enable = tmp;
    } else {
        if (!st.chip_cfg.sensors) return -1;
        if (enable && st.chip_cfg.int_enable) return 0;
        if (enable) tmp = BIT_DATA_RDY_EN;
        else tmp = 0x00;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp)) return -1;
        st.chip_cfg.int_enable = tmp;
    }
    return 0;
}

int mpu_read_reg(unsigned char reg, unsigned char *data)
{
    if (reg == st.reg->fifo_r_w || reg == st.reg->mem_r_w) return -1;
    if (reg >= st.hw->num_reg) return -1;
    return i2c_read(st.hw->addr, reg, 1, data);
}

int mpu_init(void)
{
    unsigned char data[6], rev;

    OLED_ShowString(1, 3, (uint8_t *)"m_init 1      ", 16);
    data[0] = BIT_RESET;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data)) return -1;
    delay_ms(100);

    OLED_ShowString(1, 3, (uint8_t *)"m_init 2      ", 16);
    data[0] = 0x00;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data)) return -1;
    delay_ms(100);

    // 强制跳过坑人的版本检查
    st.chip_cfg.accel_half = 0;

    st.chip_cfg.sensors = 0xFF;
    st.chip_cfg.gyro_fsr = 0xFF;
    st.chip_cfg.accel_fsr = 0xFF;
    st.chip_cfg.lpf = 0xFF;
    st.chip_cfg.sample_rate = 0xFFFF;
    st.chip_cfg.fifo_enable = 0xFF;
    st.chip_cfg.bypass_mode = 0xFF;
    st.chip_cfg.clk_src = INV_CLK_PLL;
    st.chip_cfg.active_low_int = 1;
    st.chip_cfg.latched_int = 0;
    st.chip_cfg.int_motion_only = 0;
    st.chip_cfg.lp_accel_mode = 0;
    memset(&st.chip_cfg.cache, 0, sizeof(st.chip_cfg.cache));
    st.chip_cfg.dmp_on = 0;
    st.chip_cfg.dmp_loaded = 0;
    st.chip_cfg.dmp_sample_rate = 0;

    OLED_ShowString(1, 3, (uint8_t *)"m_init 3      ", 16);
    if (mpu_set_gyro_fsr(2000)) return -1;
    
    OLED_ShowString(1, 3, (uint8_t *)"m_init 4      ", 16);
    if (mpu_set_accel_fsr(2)) return -1;
    
    OLED_ShowString(1, 3, (uint8_t *)"m_init 5      ", 16);
    if (mpu_set_lpf(42)) return -1;
    
    OLED_ShowString(1, 3, (uint8_t *)"m_init 6      ", 16);
    if (mpu_set_sample_rate(50)) return -1;
    
    OLED_ShowString(1, 3, (uint8_t *)"m_init 7      ", 16);
    if (mpu_configure_fifo(0)) return -1;

    OLED_ShowString(1, 3, (uint8_t *)"m_init 8      ", 16);
    if (mpu_set_bypass(0)) return -1;

    OLED_ShowString(1, 3, (uint8_t *)"m_init 9      ", 16);
    //mpu_set_sensors(0);
    
    return 0;
}

int mpu_get_gyro_reg(short *data, unsigned long *timestamp)
{
    unsigned char tmp[6];
    if (!(st.chip_cfg.sensors & INV_XYZ_GYRO)) return -1;
    if (i2c_read(st.hw->addr, st.reg->raw_gyro, 6, tmp)) return -1;
    data[0] = (tmp[0] << 8) | tmp[1];
    data[1] = (tmp[2] << 8) | tmp[3];
    data[2] = (tmp[4] << 8) | tmp[5];
    if (timestamp) get_ms(timestamp);
    return 0;
}

int mpu_get_accel_reg(short *data, unsigned long *timestamp)
{
    unsigned char tmp[6];
    if (!(st.chip_cfg.sensors & INV_XYZ_ACCEL)) return -1;
    if (i2c_read(st.hw->addr, st.reg->raw_accel, 6, tmp)) return -1;
    data[0] = (tmp[0] << 8) | tmp[1];
    data[1] = (tmp[2] << 8) | tmp[3];
    data[2] = (tmp[4] << 8) | tmp[5];
    if (timestamp) get_ms(timestamp);
    return 0;
}

int mpu_reset_fifo(void)
{
    unsigned char data;
    if (!(st.chip_cfg.sensors)) return -1;
    data = 0;
    if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data)) return -1;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data)) return -1;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data)) return -1;

    if (st.chip_cfg.dmp_on) {
        data = BIT_FIFO_RST | BIT_DMP_RST;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data)) return -1;
        HAL_Delay(50); 
        data = BIT_DMP_EN | BIT_FIFO_EN;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data)) return -1;
        if (st.chip_cfg.int_enable) data = BIT_DMP_INT_EN;
        else data = 0;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data)) return -1;
        data = 0;
        if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data)) return -1;
    } else {
        data = BIT_FIFO_RST;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data)) return -1;
        if (st.chip_cfg.bypass_mode) data = BIT_FIFO_EN;
        else data = BIT_FIFO_EN;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data)) return -1;
        HAL_Delay(50); 
        if (st.chip_cfg.int_enable) data = BIT_DATA_RDY_EN;
        else data = 0;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data)) return -1;
        if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &st.chip_cfg.fifo_enable)) return -1;
    }
    return 0;
}

int mpu_get_gyro_fsr(unsigned short *fsr)
{
    switch (st.chip_cfg.gyro_fsr) {
    case INV_FSR_250DPS: fsr[0] = 250; break;
    case INV_FSR_500DPS: fsr[0] = 500; break;
    case INV_FSR_1000DPS: fsr[0] = 1000; break;
    case INV_FSR_2000DPS: fsr[0] = 2000; break;
    default: fsr[0] = 0; break;
    }
    return 0;
}

int mpu_set_gyro_fsr(unsigned short fsr)
{
    unsigned char data;
    if (!(st.chip_cfg.sensors)) return -1;
    switch (fsr) {
    case 250: data = INV_FSR_250DPS << 3; break;
    case 500: data = INV_FSR_500DPS << 3; break;
    case 1000: data = INV_FSR_1000DPS << 3; break;
    case 2000: data = INV_FSR_2000DPS << 3; break;
    default: return -1;
    }
    if (st.chip_cfg.gyro_fsr == (data >> 3)) return 0;
    if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, &data)) return -1;
    st.chip_cfg.gyro_fsr = data >> 3;
    return 0;
}

int mpu_get_accel_fsr(unsigned char *fsr)
{
    switch (st.chip_cfg.accel_fsr) {
    case INV_FSR_2G: fsr[0] = 2; break;
    case INV_FSR_4G: fsr[0] = 4; break;
    case INV_FSR_8G: fsr[0] = 8; break;
    case INV_FSR_16G: fsr[0] = 16; break;
    default: return -1;
    }
    if (st.chip_cfg.accel_half) fsr[0] <<= 1;
    return 0;
}

int mpu_set_accel_fsr(unsigned char fsr)
{
    unsigned char data;
    if (!(st.chip_cfg.sensors)) return -1;
    switch (fsr) {
    case 2: data = INV_FSR_2G << 3; break;
    case 4: data = INV_FSR_4G << 3; break;
    case 8: data = INV_FSR_8G << 3; break;
    case 16: data = INV_FSR_16G << 3; break;
    default: return -1;
    }
    if (st.chip_cfg.accel_fsr == (data >> 3)) return 0;
    if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, &data)) return -1;
    st.chip_cfg.accel_fsr = data >> 3;
    return 0;
}

int mpu_get_gyro_sens(float *sens)
{
    switch (st.chip_cfg.gyro_fsr) {
    case INV_FSR_250DPS: sens[0] = 131.f; break;
    case INV_FSR_500DPS: sens[0] = 65.5f; break;
    case INV_FSR_1000DPS: sens[0] = 32.8f; break;
    case INV_FSR_2000DPS: sens[0] = 16.4f; break;
    default: return -1;
    }
    return 0;
}

int mpu_get_accel_sens(unsigned short *sens)
{
    switch (st.chip_cfg.accel_fsr) {
    case INV_FSR_2G: sens[0] = 16384; break;
    case INV_FSR_4G: sens[0] = 8092; break;
    case INV_FSR_8G: sens[0] = 4096; break;
    case INV_FSR_16G: sens[0] = 2048; break;
    default: return -1;
    }
    if (st.chip_cfg.accel_half) sens[0] >>= 1;
    return 0;
}

int mpu_get_lpf(unsigned short *lpf)
{
    switch (st.chip_cfg.lpf) {
    case INV_FILTER_188HZ: lpf[0] = 188; break;
    case INV_FILTER_98HZ: lpf[0] = 98; break;
    case INV_FILTER_42HZ: lpf[0] = 42; break;
    case INV_FILTER_20HZ: lpf[0] = 20; break;
    case INV_FILTER_10HZ: lpf[0] = 10; break;
    case INV_FILTER_5HZ: lpf[0] = 5; break;
    default: lpf[0] = 0; break;
    }
    return 0;
}

int mpu_set_lpf(unsigned short lpf)
{
    unsigned char data;
    if (!(st.chip_cfg.sensors)) return -1;
    if (lpf >= 188) data = INV_FILTER_188HZ;
    else if (lpf >= 98) data = INV_FILTER_98HZ;
    else if (lpf >= 42) data = INV_FILTER_42HZ;
    else if (lpf >= 20) data = INV_FILTER_20HZ;
    else if (lpf >= 10) data = INV_FILTER_10HZ;
    else data = INV_FILTER_5HZ;

    if (st.chip_cfg.lpf == data) return 0;
    if (i2c_write(st.hw->addr, st.reg->lpf, 1, &data)) return -1;
    st.chip_cfg.lpf = data;
    return 0;
}

int mpu_get_sample_rate(unsigned short *rate)
{
    if (st.chip_cfg.dmp_on) return -1;
    else rate[0] = st.chip_cfg.sample_rate;
    return 0;
}

int mpu_set_sample_rate(unsigned short rate)
{
    unsigned char data;
    if (!(st.chip_cfg.sensors)) return -1;
    if (st.chip_cfg.dmp_on) return -1;
    else {
        if (rate < 4) rate = 4;
        else if (rate > 1000) rate = 1000;
        data = 1000 / rate - 1;
        if (i2c_write(st.hw->addr, st.reg->rate_div, 1, &data)) return -1;
        st.chip_cfg.sample_rate = 1000 / (1 + data);
        mpu_set_lpf(st.chip_cfg.sample_rate >> 1);
        return 0;
    }
}

int mpu_get_fifo_config(unsigned char *sensors)
{
    sensors[0] = st.chip_cfg.fifo_enable;
    return 0;
}

int mpu_configure_fifo(unsigned char sensors)
{
    unsigned char prev;
    int result = 0;
    sensors &= ~INV_XYZ_COMPASS;
    if (st.chip_cfg.dmp_on) return 0;
    else {
        if (!(st.chip_cfg.sensors)) return -1;
        prev = st.chip_cfg.fifo_enable;
        st.chip_cfg.fifo_enable = sensors & st.chip_cfg.sensors;
        if (st.chip_cfg.fifo_enable != sensors) result = -1;
        else result = 0;
        if (sensors || st.chip_cfg.lp_accel_mode) set_int_enable(1);
        else set_int_enable(0);
        if (sensors) {
            if (mpu_reset_fifo()) {
                st.chip_cfg.fifo_enable = prev;
                return -1;
            }
        }
    }
    return result;
}

int mpu_set_sensors(unsigned char sensors)
{
    unsigned char data;
    if (sensors & INV_XYZ_GYRO) data = INV_CLK_PLL;
    else if (sensors) data = 0;
    else data = BIT_SLEEP;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, &data)) {
        st.chip_cfg.sensors = 0;
        return -1;
    }
    st.chip_cfg.clk_src = data & ~BIT_SLEEP;

    data = 0;
    if (!(sensors & INV_X_GYRO)) data |= BIT_STBY_XG;
    if (!(sensors & INV_Y_GYRO)) data |= BIT_STBY_YG;
    if (!(sensors & INV_Z_GYRO)) data |= BIT_STBY_ZG;
    if (!(sensors & INV_XYZ_ACCEL)) data |= BIT_STBY_XYZA;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_2, 1, &data)) {
        st.chip_cfg.sensors = 0;
        return -1;
    }
    if (sensors && (sensors != INV_XYZ_ACCEL)) mpu_set_int_latched(0);

    st.chip_cfg.sensors = sensors;
    st.chip_cfg.lp_accel_mode = 0;
    delay_ms(50);
    return 0;
}

int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp, unsigned char *sensors, unsigned char *more)
{
    unsigned char data[MAX_PACKET_LENGTH];
    unsigned char packet_size = 0;
    unsigned short fifo_count, index = 0;
    if (st.chip_cfg.dmp_on) return -1;
    sensors[0] = 0;
    if (!st.chip_cfg.sensors) return -1;
    if (!st.chip_cfg.fifo_enable) return -1;

    if (st.chip_cfg.fifo_enable & INV_X_GYRO) packet_size += 2;
    if (st.chip_cfg.fifo_enable & INV_Y_GYRO) packet_size += 2;
    if (st.chip_cfg.fifo_enable & INV_Z_GYRO) packet_size += 2;
    if (st.chip_cfg.fifo_enable & INV_XYZ_ACCEL) packet_size += 6;

    if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data)) return -1;
    fifo_count = (data[0] << 8) | data[1];
    if (fifo_count < packet_size) return 0;
    if (fifo_count > (st.hw->max_fifo >> 1)) {
        if (i2c_read(st.hw->addr, st.reg->int_status, 1, data)) return -1;
        if (data[0] & BIT_FIFO_OVERFLOW) {
            mpu_reset_fifo();
            return -2;
        }
    }
    get_ms((unsigned long*)timestamp);
    if (i2c_read(st.hw->addr, st.reg->fifo_r_w, packet_size, data)) return -1;
    more[0] = fifo_count / packet_size - 1;
    sensors[0] = 0;

    if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_XYZ_ACCEL) {
        accel[0] = (data[index+0] << 8) | data[index+1];
        accel[1] = (data[index+2] << 8) | data[index+3];
        accel[2] = (data[index+4] << 8) | data[index+5];
        sensors[0] |= INV_XYZ_ACCEL;
        index += 6;
    }
    if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_X_GYRO) {
        gyro[0] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_X_GYRO;
        index += 2;
    }
    if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_Y_GYRO) {
        gyro[1] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_Y_GYRO;
        index += 2;
    }
    if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_Z_GYRO) {
        gyro[2] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_Z_GYRO;
        index += 2;
    }
    return 0;
}

int mpu_read_fifo_stream(unsigned short length, unsigned char *data, unsigned char *more)
{
    unsigned char tmp[2];
    unsigned short fifo_count;
    if (!st.chip_cfg.dmp_on) return -1;
    if (!st.chip_cfg.sensors) return -1;

    if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, tmp)) return -1;
    fifo_count = (tmp[0] << 8) | tmp[1];
    if (fifo_count < length) {
        more[0] = 0;
        return -1;
    }
    if (fifo_count > (st.hw->max_fifo >> 1)) {
        if (i2c_read(st.hw->addr, st.reg->int_status, 1, tmp)) return -1;
        if (tmp[0] & BIT_FIFO_OVERFLOW) {
            mpu_reset_fifo();
            return -2;
        }
    }
    if (i2c_read(st.hw->addr, st.reg->fifo_r_w, length, data)) return -1;
    more[0] = fifo_count / length - 1;
    return 0;
}

int mpu_set_bypass(unsigned char bypass_on)
{
    unsigned char tmp;
    if (st.chip_cfg.bypass_mode == bypass_on) return 0;

    if (bypass_on) {
        if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp)) return -1;
        tmp &= ~BIT_AUX_IF_EN;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp)) return -1;
        delay_ms(3);
        tmp = BIT_BYPASS_EN;
        if (st.chip_cfg.active_low_int) tmp |= BIT_ACTL;
        if (st.chip_cfg.latched_int) tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp)) return -1;
    } else {
        if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp)) return -1;
        tmp &= ~BIT_AUX_IF_EN;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp)) return -1;
        delay_ms(3);
        if (st.chip_cfg.active_low_int) tmp = BIT_ACTL;
        else tmp = 0;
        if (st.chip_cfg.latched_int) tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp)) return -1;
    }
    st.chip_cfg.bypass_mode = bypass_on;
    return 0;
}

int mpu_set_int_level(unsigned char active_low)
{
    st.chip_cfg.active_low_int = active_low;
    return 0;
}

int mpu_set_int_latched(unsigned char enable)
{
    unsigned char tmp;
    if (st.chip_cfg.latched_int == enable) return 0;
    if (enable) tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR;
    else tmp = 0;
    if (st.chip_cfg.bypass_mode) tmp |= BIT_BYPASS_EN;
    if (st.chip_cfg.active_low_int) tmp |= BIT_ACTL;
    if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp)) return -1;
    st.chip_cfg.latched_int = enable;
    return 0;
}

int mpu_write_mem(unsigned short mem_addr, unsigned short length, unsigned char *data)
{
    unsigned char tmp[2];
    if (!data) return -1;
    if (!st.chip_cfg.sensors) return -1;
    tmp[0] = (unsigned char)(mem_addr >> 8);
    tmp[1] = (unsigned char)(mem_addr & 0xFF);
    if (tmp[1] + length > st.hw->bank_size) return -1;
    if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp)) return -1;
    if (i2c_write(st.hw->addr, st.reg->mem_r_w, length, data)) return -1;
    return 0;
}

int mpu_read_mem(unsigned short mem_addr, unsigned short length, unsigned char *data)
{
    unsigned char tmp[2];
    if (!data) return -1;
    if (!st.chip_cfg.sensors) return -1;
    tmp[0] = (unsigned char)(mem_addr >> 8);
    tmp[1] = (unsigned char)(mem_addr & 0xFF);
    if (tmp[1] + length > st.hw->bank_size) return -1;
    if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp)) return -1;
    if (i2c_read(st.hw->addr, st.reg->mem_r_w, length, data)) return -1;
    return 0;
}

int mpu_load_firmware(unsigned short length, const unsigned char *firmware, unsigned short start_addr, unsigned short sample_rate)
{
    unsigned short ii;
    unsigned short this_write;
#define LOAD_CHUNK  (16)
    unsigned char cur[LOAD_CHUNK], tmp[2];
    if (st.chip_cfg.dmp_loaded) return -1;
    if (!firmware) return -1;
    for (ii = 0; ii < length; ii += this_write) {
        this_write = min(LOAD_CHUNK, length - ii);
        if (mpu_write_mem(ii, this_write, (unsigned char*)&firmware[ii])) return -1;
        if (mpu_read_mem(ii, this_write, cur)) return -1;
        if (memcmp(firmware+ii, cur, this_write)) return -2;
    }
    tmp[0] = start_addr >> 8;
    tmp[1] = start_addr & 0xFF;
    if (i2c_write(st.hw->addr, st.reg->prgm_start_h, 2, tmp)) return -1;
    st.chip_cfg.dmp_loaded = 1;
    st.chip_cfg.dmp_sample_rate = sample_rate;
    return 0;
}

int mpu_set_dmp_state(unsigned char enable)
{
    unsigned char tmp;
    if (st.chip_cfg.dmp_on == enable) return 0;

    if (enable) {
        if (!st.chip_cfg.dmp_loaded) return -1;
        set_int_enable(0);
        mpu_set_bypass(0);
        mpu_set_sample_rate(st.chip_cfg.dmp_sample_rate);
        tmp = 0;
        i2c_write(st.hw->addr, 0x23, 1, &tmp);
        st.chip_cfg.dmp_on = 1;
        set_int_enable(1);
        mpu_reset_fifo();
    } else {
        set_int_enable(0);
        tmp = st.chip_cfg.fifo_enable;
        i2c_write(st.hw->addr, 0x23, 1, &tmp);
        st.chip_cfg.dmp_on = 0;
        mpu_reset_fifo();
    }
    return 0;
}

int mpu_get_dmp_state(unsigned char *enabled)
{
    enabled[0] = st.chip_cfg.dmp_on;
    return 0;
}

#define q30  1073741824.0f

static signed char gyro_orientation[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;
    if (row[0] > 0) b = 0;
    else if (row[0] < 0) b = 4;
    else if (row[1] > 0) b = 1;
    else if (row[1] < 0) b = 5;
    else if (row[2] > 0) b = 2;
    else if (row[2] < 0) b = 6;
    else b = 7;
    return b;
}

unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar; 
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}

void mget_ms(unsigned long *time)
{
}

u8 mpu_dmp_init(void)
{
    u8 res;
    
    OLED_ShowString(1, 3, (uint8_t *)"Init IIC...   ", 16);
    MPU_IIC_Init();
    
    // 1. 直接通过寄存器复位并唤醒芯片（彻底抛弃官方那个慢吞吞的 mpu_init）
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x80); // 复位
    HAL_Delay(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x00); // 唤醒
    HAL_Delay(100);
    
    // 2. 手动设置量程和采样率
    MPU_Write_Byte(MPU_GYRO_CFG_REG, 3<<3);     // 陀螺仪 ±2000dps
    MPU_Write_Byte(MPU_ACCEL_CFG_REG, 0<<3);    // 加速度 ±2g
    MPU_Write_Byte(MPU_CFG_REG, 3);             // LPF 42Hz
    MPU_Write_Byte(MPU_SAMPLE_RATE_REG, 9);     // 100Hz (1000/100 - 1 = 9)
    
    // 3. 强制初始化内部状态（欺骗官方库，防止死循环）
    st.chip_cfg.sensors = INV_XYZ_GYRO | INV_XYZ_ACCEL;
    st.chip_cfg.gyro_fsr = INV_FSR_2000DPS;
    st.chip_cfg.accel_fsr = INV_FSR_2G;
    st.chip_cfg.lpf = INV_FILTER_42HZ;
    st.chip_cfg.sample_rate = 100;
    st.chip_cfg.bypass_mode = 0;
    st.chip_cfg.dmp_on = 0;
    st.chip_cfg.dmp_loaded = 0;
    st.chip_cfg.int_enable = 0;

    // 4. 写入 DMP 固件
    OLED_ShowString(1, 3, (uint8_t *)"Load DMP...   ", 16);
    res = dmp_load_motion_driver_firmware();
    if(res) return 4;
    
    // 5. 设置方向
    OLED_ShowString(1, 3, (uint8_t *)"Set Orient... ", 16);
    res = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    if(res) return 5;
    
    // 6. 开启 DMP 功能
    OLED_ShowString(1, 3, (uint8_t *)"Enable DMP... ", 16);
    res = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |  
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL);
    if(res) return 6;
    
    // 7. 设置输出速率
    OLED_ShowString(1, 3, (uint8_t *)"Set Rate...   ", 16);
    res = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    if(res) return 7;
    
    // 8. 正式启动 DMP 引擎
    OLED_ShowString(1, 3, (uint8_t *)"Start DMP...  ", 16);
    res = mpu_set_dmp_state(1);
    if(res) return 9;
    
    return 0;
}
u8 mpu_dmp_get_data(float *pitch,float *roll,float *yaw)
{
    float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4]; 
    if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))return 1;
    if(sensors&INV_WXYZ_QUAT) 
    {
        q0 = quat[0] / q30; 
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30; 
        *pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;                                 
        *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 
        *yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;               
    }else return 2;
    return 0;
}