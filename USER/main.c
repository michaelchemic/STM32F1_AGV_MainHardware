#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "motor_ctl.h"
#include "can.h"

extern uint8_t motor_mode;    // �㶯��������ֹͣ
extern uint8_t function_code; // ǰ�������˶�����

extern uint8_t rx_buffer[RX_BUFFER_SIZE];
extern uint8_t rx_index;

void can_transmit_tx_msg_stop(void);
void can_transmit_clockwise_m3(void); // ���Ͳ��Ժ�������
void can_transmit_clockwise_m4(void); // ���Ͳ��Ժ�������
void can_transmit_anticlockwise_m3(void); // ���Ͳ��Ժ�������
void can_transmit_anticlockwise_m4(void); // ���Ͳ��Ժ�������

void can_receive_test(void);  // ���ղ��Ժ�������

// can���Ͳ��� ������ 800kbps
volatile int i    = 0;
// can ֹͣ����
uint8_t tx_msg_stop[8] = {0xaa, 0x03, 0x00, 0x00, 0x55, 0x06, 0x07, 0x08}; // ����data����

uint8_t tx_msg_clockwise[8] = {0xaa, 0x02, 0x02, 0x00, 0x55, 0x06, 0x07, 0x08};
uint8_t tx_msg_anticlockwise[8] = {0xaa, 0x02, 0x01, 0x00, 0x55, 0x06, 0x07, 0x08};

uint8_t rx_msg[8];                                                    // ��������
uint8_t len;                                                          // ����
uint32_t id;   

int main(void)
{
	  SystemInit(); // ϵͳ��ʼ��
    CAN_Config(); // ��ʼ��CAN
    delay_init();                                   // ��ʱ������ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // �����ж����ȼ�����2
    uart1_init(115200);
    //USART_Send_String(USART1, "USART1 Send OK!");
    LED_Init();   // LED������ʼ��
    Motor_Init(); // �������IO��ʼ��

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
                    continuous(0, 0); // ǰ��
								    can_transmit_clockwise_m3();
							    	can_transmit_anticlockwise_m4();
                    break;
                case 0x02:
                    continuous(1, 1); // ����
									  can_transmit_anticlockwise_m3();
							    	can_transmit_clockwise_m4();
                    break;
                case 0x03:
                    continuous(0, 1); // ԭ����ת
									  can_transmit_anticlockwise_m3();
							    	can_transmit_anticlockwise_m4();
                    break;
                case 0x04:
                    continuous(1, 0); // ԭ����ת
								    can_transmit_clockwise_m3();
							    	can_transmit_clockwise_m4();
                default:
                    motor_stop(); // ���ֹͣ
            }
        }
        
            if (motor_mode == 0x03) {
                    motor_stop(); // ���ֹͣ
										can_transmit_tx_msg_stop();
            }
						
        }
}

void can_transmit_clockwise_m3(void)

{
    /* ����CAN��Ϣ */
    CAN_TransmitMessage(0x120, tx_msg_clockwise, 8);

    /* �����ӳ� */
    delay_ms(100);
}

void can_transmit_clockwise_m4(void)

{
    /* ����CAN��Ϣ */
    CAN_TransmitMessage(0x121, tx_msg_clockwise, 8);

    /* �����ӳ� */
    delay_ms(100);
}

void can_transmit_anticlockwise_m3(void)

{
    /* ����CAN��Ϣ */
    CAN_TransmitMessage(0x120, tx_msg_anticlockwise, 8);

    /* �����ӳ� */
    delay_ms(100);
}

void can_transmit_anticlockwise_m4(void)

{
    /* ����CAN��Ϣ */
    CAN_TransmitMessage(0x121, tx_msg_anticlockwise, 8);

    /* �����ӳ� */
    delay_ms(100);
}

void can_transmit_tx_msg_stop(void)

{
    /* ����CAN��Ϣ */
    CAN_TransmitMessage(0x120, tx_msg_stop, 8);
	
	  CAN_TransmitMessage(0x121, tx_msg_stop, 8);
	
}


