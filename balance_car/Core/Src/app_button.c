#include"app_button.h"
#include"app_pwm.h"

//=====按键配置区，在这改引脚=====
#define KEY1_PORT GPIOB
#define KEY1_PIN GPIO_PIN_1

#define KEY2_PORT GPIOB
#define KEY2_PIN GPIO_PIN_12
//================================
//按键 1 变量
uint8_t key1_last = 1;            //上一次读到的按键电平，按键松开是高电平1
uint32_t key1_cnt = 0;            //消抖计数器
uint8_t key1_press_flag = 0;      //标记：是否确认按键已经按下 0=没按下， 1=按下

//按键 2 变量
uint8_t key2_last = 1;
uint32_t key2_cnt = 0;
uint8_t key2_press_flag = 0;

uint8_t motor_run_flag = 0;
//=================================
//按键1单击回调，static代表这个函数只允许本文件内部可调用，外部看不见
static void Key1_Click(void)
{
	motor_run_flag = !motor_run_flag;      //取反翻转，0变1，1变0，点一次启动，再点一次停止
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	if(motor_run_flag  == 1)
	{
		Motor_SetSpeed(300, 300);  // ✅ 按键控制电机转
	}
	else
	{
		Motor_SetSpeed(0, 0);      // ✅ 按键控制电机停
	}
}
static void Key2_Click(void)
{
	//预留，后面平衡车功能写这里
}
void App_Button_Init(void)
{
	//GPIO引脚初始化已经CubMX在gpio.c里面做完，这里空函数
}
void App_Button_Proc(void)
{
	//------------按键1 PB1扫描--------------
	uint8_t key1_now = HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN);
	if(key1_now != key1_last)
	{
		key1_cnt = 2;              // 消抖计数改成 2，按一下就能触发
		key1_last = key1_now;
	}
	if(key1_cnt>0)key1_cnt--;
	if(key1_cnt == 0)
	{
		if(key1_now == 0 && key1_press_flag == 0)
		{
			key1_press_flag = 1;
		}
		if(key1_now == 1 && key1_press_flag == 1)
		{
			key1_press_flag = 0;
			Key1_Click();
		}
	}

	//------------按键2 PB12扫描--------------
	uint8_t key2_now = HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN);
	if(key2_now != key2_last)
	{
		key2_cnt = 2;              // 消抖计数改成 2
		key2_last = key2_now;
	}
	if(key2_cnt>0)key2_cnt--;
	if(key2_cnt == 0)
	{
		if(key2_now == 0 && key2_press_flag == 0)
		{
			key2_press_flag = 1;
		}
		if(key2_now == 1 && key2_press_flag == 1)
		{
			key2_press_flag = 0;
			Key2_Click();
		}
	}
}