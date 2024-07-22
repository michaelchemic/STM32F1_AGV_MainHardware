#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "motor_ctl.h"
#include "can.h"

extern uint8_t motor_mode;    // 点动，连续，停止
extern uint8_t function_code; // 前后左右运动功能

extern uint8_t rx_buffer[RX_BUFFER_SIZE];
extern uint8_t rx_index;

void can_transmit_tx_msg_stop(void);
void can_transmit_clockwise_m3(void); // 发送测试函数声明
void can_transmit_clockwise_m4(void); // 发送测试函数声明
void can_transmit_anticlockwise_m3(void); // 发送测试函数声明
void can_transmit_anticlockwise_m4(void); // 发送测试函数声明

void can_receive_test(void);  // 接收测试函数声明

// can发送测试 波特率 800kbps
volatile int i    = 0;
// can 停止报文
uint8_t tx_msg_stop[8] = {0xaa, 0x03, 0x00, 0x00, 0x55, 0x06, 0x07, 0x08}; // 发送data数据

uint8_t tx_msg_clockwise[8] = {0xaa, 0x02, 0x02, 0x00, 0x55, 0x06, 0x07, 0x08};
uint8_t tx_msg_anticlockwise[8] = {0xaa, 0x02, 0x01, 0x00, 0x55, 0x06, 0x07, 0x08};

uint8_t rx_msg[8];                                                    // 接收数组
uint8_t len;                                                          // 长度
uint32_t id;   

int main(void)
{
	  SystemInit(); // 系统初始化
    CAN_Config(); // 初始化CAN
    delay_init();                                   // 延时函数初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置中断优先级分组2
    uart1_init(115200);
    //USART_Send_String(USART1, "USART1 Send OK!");
    LED_Init();   // LED驱动初始化
    Motor_Init(); // 电机驱动IO初始化

    while (1) {
			
//         if (motor_mode == 0x01) {
//            switch (function_code) {
//                case 0x01:
//                    point_mode(0, 0);
//                    break;
//                case 0x02:
//                    point_mode(1, 1);
//                    break;
//                case 0x03:
//                    point_mode(0, 1);
//                    break;
//                case 0x04:
//                    point_mode(1, 0);
//                    break;
//                default:
//                    motor_stop();
//            }
//        }
         
        if (motor_mode == 0x02) {
            switch (function_code) {
                case 0x01:
                    continuous(0, 0); // 前进
								    can_transmit_clockwise_m3();
							    	can_transmit_anticlockwise_m4();
                    break;
                case 0x02:
                    continuous(1, 1); // 后退
									  can_transmit_anticlockwise_m3();
							    	can_transmit_clockwise_m4();
                    break;
                case 0x03:
                    continuous(0, 1); // 原地左转
									  can_transmit_anticlockwise_m3();
							    	can_transmit_anticlockwise_m4();
                    break;
                case 0x04:
                    continuous(1, 0); // 原地右转
								    can_transmit_clockwise_m3();
							    	can_transmit_clockwise_m4();
                default:
                    motor_stop(); // 电机停止
            }
        }
        
            if (motor_mode == 0x03) {
                    motor_stop(); // 电机停止
										can_transmit_tx_msg_stop();
            }
						
        }
}

void can_transmit_clockwise_m3(void)

{
    /* 发送CAN消息 */
    CAN_TransmitMessage(0x120, tx_msg_clockwise, 8);

    /* 发送延迟 */
    delay_ms(100);
}

void can_transmit_clockwise_m4(void)

{
    /* 发送CAN消息 */
    CAN_TransmitMessage(0x121, tx_msg_clockwise, 8);

    /* 发送延迟 */
    delay_ms(100);
}

void can_transmit_anticlockwise_m3(void)

{
    /* 发送CAN消息 */
    CAN_TransmitMessage(0x120, tx_msg_anticlockwise, 8);

    /* 发送延迟 */
    delay_ms(100);
}

void can_transmit_anticlockwise_m4(void)

{
    /* 发送CAN消息 */
    CAN_TransmitMessage(0x121, tx_msg_anticlockwise, 8);

    /* 发送延迟 */
    delay_ms(100);
}

void can_transmit_tx_msg_stop(void)

{
    /* 发送CAN消息 */
    CAN_TransmitMessage(0x120, tx_msg_stop, 8);
	
	  CAN_TransmitMessage(0x121, tx_msg_stop, 8);
	
}


