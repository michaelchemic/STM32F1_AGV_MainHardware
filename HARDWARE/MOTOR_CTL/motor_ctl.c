// 用来控制4路BLDC电机驱动器
#include "motor_ctl.h"
#include "usart.h"

extern uint8_t function_code; // 前后左右运动功能
void Motor_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

	GPIO_InitStructure.GPIO_Pin = Motor1_EN | Motor2_EN | Motor1_Dir | Motor2_Dir | Motor1_Speed | Motor2_Speed; // LED0-->PA.8 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;															 // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;															 // IO口速度为50MHz
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_SetBits(GPIOF, Motor1_EN | Motor2_EN | Motor1_Dir | Motor2_Dir | Motor1_Speed | Motor2_Speed); // PA.8 输出高
}

// 前进，后退，左转右转函数

// 电机速度通过通信协议来设置占空比
// 下面函数为连续运动模式函数
void continuous(unsigned int motor1_dir, unsigned int motor2_dir)
{
	// 进入函数解锁电机，硬件上设置转速，但是这里要给个电压信号高电平。
	// GPIO_SetBits(GPIOF, Motor1_EN);//这样设置让电机1保持停止
	GPIO_ResetBits(GPIOF, Motor1_EN);
	GPIO_SetBits(GPIOF, Motor1_Speed);

	// GPIO_SetBits(GPIOF, Motor2_EN);//设置让电机2保持停止
	GPIO_ResetBits(GPIOF, Motor2_EN);
	GPIO_SetBits(GPIOF, Motor2_Speed);

	switch (motor1_dir) // 设置的电机旋转方向
	{

	case 0:
		GPIO_SetBits(GPIOF, Motor1_Dir); // 置1
		break;
	case 1:
		GPIO_ResetBits(GPIOF, Motor1_Dir); // 置0
		break;
	default:
		GPIO_SetBits(GPIOF, Motor1_EN); // 默认情况关闭电机
		break;
	}

	switch (motor2_dir)
	{

	case 0:
		GPIO_ResetBits(GPIOF, Motor2_Dir); // 置0
		break;
	case 1:
		GPIO_SetBits(GPIOF, Motor2_Dir); // 置1
		break;
	default:
		GPIO_SetBits(GPIOF, Motor2_EN); // 默认情况关闭电机
		break;
	}
}
// 下面函数为停止模式函数.就是把电机EN关闭
void motor_stop(void)
{

	GPIO_SetBits(GPIOF, Motor1_EN); // 这样设置让电机1保持停止
	GPIO_SetBits(GPIOF, Motor2_EN); // 这样设置让电机2保持停止
}

// 下面函数为点动模式函数,延时1S后关闭电机，从而实现点动。。。
void point_mode(unsigned int motor1_dir, unsigned int motor2_dir)
{
	// 进入函数解锁电机，硬件上设置转速，但是这里要给个电压信号高电平。
	// GPIO_SetBits(GPIOF, Motor1_EN);//这样设置让电机1保持停止
	GPIO_ResetBits(GPIOF, Motor1_EN);
	GPIO_SetBits(GPIOF, Motor1_Speed);

	// GPIO_SetBits(GPIOF, Motor2_EN);//设置让电机2保持停止
	GPIO_ResetBits(GPIOF, Motor2_EN);
	GPIO_SetBits(GPIOF, Motor2_Speed);

	switch (motor1_dir) // 设置的电机旋转方向
	{

	case 0:
		GPIO_SetBits(GPIOF, Motor1_Dir); // 置1
		delay_ms(500);
		motor_stop();
		break;
	case 1:
		GPIO_ResetBits(GPIOF, Motor1_Dir); // 置0
		delay_ms(500);
		motor_stop();
		break;
	default:
		GPIO_SetBits(GPIOF, Motor1_EN); // 默认情况关闭电机
		break;
	}

	switch (motor2_dir)
	{

	case 0:
		GPIO_ResetBits(GPIOF, Motor2_Dir); // 置0
										   // delay_ms (10);
		motor_stop();
		break;
	case 1:
		GPIO_SetBits(GPIOF, Motor2_Dir); // 置1
										 // delay_ms (10);
		motor_stop();
		break;
	default:
		GPIO_SetBits(GPIOF, Motor2_EN); // 默认情况关闭电机
		break;
	}

	function_code = 0x00; // 清除数据
}
