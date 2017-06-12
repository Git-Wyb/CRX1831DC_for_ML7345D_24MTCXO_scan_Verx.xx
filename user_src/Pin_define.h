#ifndef __PIN_DEFINE_H__
#define __PIN_DEFINE_H__
/***********************************************************************/
/*  File Name   :Pin_Define.h                                          */
/*  DATE        :Mar, 2014                                             */
/*  Programmer	:xiang 'R                                              */
/*  CPU TYPE    :STM8L151G6     Crystal: 16M HSI                       */
/*  DESCRIPTION :                                                      */
/*  Mark        :ver 1.0                                               */
/***********************************************************************/
#include "Timer.h"
//������IO����
/********************LED�Ĵ���*****************************************/
extern u8 LED_Cache;
#define LED_ON 1
#define LED_OFF 0
#define LED_YELLOW LED_Cache       //�Ƶ�
#define LED_YELLOW_DDR PC_DDR_DDR0 //�ƵƷ���
#define LED_YELLOW_CR1 PC_CR1_C10  //�Ƶ�����
#define LED_YELLOW_CR2 PC_CR2_C20  //�Ƶ����Ƶ��

#define LED_RED LED_Cache       //���
#define LED_RED_DDR PC_DDR_DDR1 //����
#define LED_RED_CR1 PC_CR1_C11  //����
#define LED_RED_CR2 PC_CR2_C21  //���Ƶ��

#define Receiver_LED_TX LED_Cache         //PC_ODR_ODR1       // Output   ���Ż�����ָʾ  �ߵ�ƽ��Ч
#define Receiver_LED_TX_direc PC_DDR_DDR1 // Output   ���Ż�����ָʾ  �ߵ�ƽ��Ч
#define Receiver_LED_TX_CR1 PC_CR1_C11    // Output   ���Ż�����ָʾ  �ߵ�ƽ��Ч

#define Receiver_LED_RX PC_ODR_ODR1       //PC_ODR_ODR0       // Output   ���Ż�����ָʾ  �ߵ�ƽ��Ч
#define Receiver_LED_RX_direc PC_DDR_DDR0 // Output   ���Ż�����ָʾ  �ߵ�ƽ��Ч
#define Receiver_LED_RX_CR1 PC_CR1_C10    // Output   ���Ż�����ָʾ  �ߵ�ƽ��Ч

#define Receiver_LED_OUT PC_ODR_ODR0       //PC_ODR_ODR4       // Output   ���Ż��̵����������  �ߵ�ƽ��Ч
#define Receiver_LED_OUT_direc PC_DDR_DDR4 // Output   ���Ż��̵����������  �ߵ�ƽ��Ч
#define Receiver_LED_OUT_CR1 PC_CR1_C14    // Output   ���Ż��̵����������  �ߵ�ƽ��Ч

#define YELLOWLED_FLASH_SECOND() \
    {                            \
        LED_YELLOW = LED_ON;     \
        LedYELLOWTimer = 750;    \
    }
#define YELLOWLED_OFF()       \
    {                         \
        LED_YELLOW = LED_OFF; \
        LedYELLOWTimer = 0;   \
    }
#define YELLOWLED_FLASH()         \
    {                             \
        LED_YELLOW = !LED_YELLOW; \
        LedYELLOWTimer = 60;      \
    }

#define REDLED_FLASH_SECOND() \
    {                         \
        LED_RED = LED_ON;     \
        LedREDTimer = 750;    \
    }
#define REDLED_OFF()       \
    {                      \
        LED_RED = LED_OFF; \
        LedREDTimer = 0;   \
    }
#define REDLED_FLASH()      \
    {                       \
        LED_RED = !LED_RED; \
        LedREDTimer = 60;   \
    }
/******************������ADF7030-1�Ĵ���************************************/
/* ADF7030-1 register interface */
#define ADF7030_REST ADF7030_REST_Cache
#define ADF7030_REST_DDR ADF7030_REST_Cache
#define ADF7030_REST_CR1 ADF7030_REST_Cache
#define ADF7030_REST_CR2 ADF7030_REST_Cache

#define ADF7030_GPIO2 PC_IDR_IDR5
#define ADF7030_GPIO2_DDR PC_DDR_DDR5
#define ADF7030_GPIO2_CR1 PC_CR1_C15
#define ADF7030_GPIO2_CR2 PC_CR2_C25

#define ADF7030_GPIO3 PD_IDR_IDR4
#define ADF7030_GPIO3_DDR PD_DDR_DDR4
#define ADF7030_GPIO3_CR1 PD_CR1_C14
#define ADF7030_GPIO3_CR2 PD_CR2_C24
//
#define ADF7030_GPIO4 PC_IDR_IDR4
#define ADF7030_GPIO4_DDR PC_DDR_DDR4
#define ADF7030_GPIO4_CR1 PC_CR1_C14
#define ADF7030_GPIO4_CR2 PC_CR2_C24

#define ADF7030_GPIO5 PC_IDR_IDR6
#define ADF7030_GPIO5_DDR PC_DDR_DDR6
#define ADF7030_GPIO5_CR1 PC_CR1_C16
#define ADF7030_GPIO5_CR2 PC_CR2_C26

#define ADF7030CLK ADF7030_GPIO4
#define ADF7030DATA ADF7030_GPIO5
/******************������KEY�Ĵ���*******����*****************************/
//#define KEY_SW2 PA_IDR_IDR4
//#define KEY_SW2_DDR PA_DDR_DDR4
//#define KEY_SW2_CR1 PA_CR1_C14
//#define KEY_SW2_CR2 PA_CR2_C24
//
//#define KEY_SW3 PA_IDR_IDR5
//#define KEY_SW3_DDR PA_DDR_DDR5
//#define KEY_SW3_CR1 PA_CR1_C15
//#define KEY_SW3_CR2 PA_CR2_C25
//
//#define KEY_SW4 PA_IDR_IDR2
//#define KEY_SW4_DDR PA_DDR_DDR2
//#define KEY_SW4_CR1 PA_CR1_C12
//#define KEY_SW4_CR2 PA_CR2_C22

// #define Receiver_Login PC_IDR_IDR6       // Input   ���Ż���¼��   �͵�ƽ��Ч
// #define Receiver_Login_direc PC_DDR_DDR6 // Input   ���Ż���¼��   �͵�ƽ��Ч
// #define Receiver_Login_CR1 PC_CR1_C16    // Input   ���Ż���¼��   �͵�ƽ��Ч

#define Receiver_Login PA_IDR_IDR2       // Input   ���Ż���¼��   �͵�ƽ��Ч
#define Receiver_Login_direc PA_DDR_DDR2 // Input   ���Ż���¼��   �͵�ƽ��Ч
#define Receiver_Login_CR1 PA_CR1_C12    // Input   ���Ż���¼��   �͵�ƽ��Ч
#define Receiver_Login_CR2 PA_CR2_C22    // Input   ���Ż���¼��   �͵�ƽ��Ч

#define KEY_Empty 0
#define KEY_SW2_Down 1
#define KEY_SW3_Down 2
#define KEY_SW4_Down 3

/**���Ż�ʹ�õ�IO HA �쳣�ź� �Ĵ��� */
#define HA_L_signal PA_IDR_IDR4       // Input   HA �����ź�   �͵�ƽ��Ч
#define HA_L_signal_direc PA_DDR_DDR4 // Input   HA �����ź�   �͵�ƽ��Ч
#define HA_L_signal_CR1 PA_CR1_C14    // Input   HA �����ź�   �͵�ƽ��Ч
#define HA_L_signal_CR2 PA_CR2_C24

#define HA_ERR_signal PA_IDR_IDR5       // Input   HA �쳣�ź�  �͵�ƽ��Ч
#define HA_ERR_signal_direc PA_DDR_DDR5 // Input   HA �쳣�ź�  �͵�ƽ��Ч
#define HA_ERR_signal_CR1 PA_CR1_C15    // Input   HA �쳣�ź�  �͵�ƽ��Ч
#define HA_ERR_signal_CR2 PA_CR2_C25

#define HA_Sensor_signal PA_IDR_IDR3       // Input   HA �������ź�  �͵�ƽ��Ч
#define HA_Sensor_signal_direc PA_DDR_DDR3 // Input   HA �������ź�  �͵�ƽ��Ч
#define HA_Sensor_signal_CR1 PA_CR1_C13    // Input   HA �������ź�  �͵�ƽ��Ч
#define HA_Sensor_signal_CR2 PA_CR2_C23

/**WORK/TEST�л����żĴ���*****/
#define WORK_TEST PB_IDR_IDR0     // Input ���Ż����Խ�  �ߵ�ƽ��Ч
#define WORK_TEST_DDR PB_DDR_DDR0 // Input ���Ż����Խ�  �ߵ�ƽ��Ч
#define WORK_TEST_CR1 PB_CR1_C10  // Input ���Ż����Խ�  �ߵ�ƽ��Ч
#define WORK_TEST_CR2 PB_CR2_C20  // Input ���Ż����Խ�  �ߵ�ƽ��Ч

#define Receiver_test PB_IDR_IDR0       // Input ���Ż����Խ�  �ߵ�ƽ��Ч
#define Receiver_test_direc PB_DDR_DDR0 // Input ���Ż����Խ�  �ߵ�ƽ��Ч
#define Receiver_test_CR1 PB_CR1_C10    // Input ���Ż����Խ�  �ߵ�ƽ��Ч

#define ChannelTimerTest PB_ODR_ODR1     // Input ���Ż����Խ�  �ߵ�ƽ��Ч
#define ChannelTimerTest_DDR PB_DDR_DDR1 // Input ���Ż����Խ�  �ߵ�ƽ��Ч
#define ChannelTimerTest_CR1 PB_CR1_C11  // Input ���Ż����Խ�  �ߵ�ƽ��Ч
#define ChannelTimerTest_CR2 PB_CR2_C21  // Input ���Ż����Խ�  �ߵ�ƽ��Ч

/********************�����л� CG2214M6�������żĴ���*****************************************/
#define CG2214M6_VC1 PB_ODR_ODR2     //VC1
#define CG2214M6_VC1_DDR PB_DDR_DDR2 //����
#define CG2214M6_VC1_CR1 PB_CR1_C12  //ģʽMODE
#define CG2214M6_VC1_CR2 PB_CR2_C22  //���Ƶ�ʻ����жϿ���

#define CG2214M6_VC2 PB_ODR_ODR3     //VC2
#define CG2214M6_VC2_DDR PB_DDR_DDR3 //����
#define CG2214M6_VC2_CR1 PB_CR1_C13  //ģʽMODE
#define CG2214M6_VC2_CR2 PB_CR2_C23  //���Ƶ�ʻ����жϿ���

#define CG2214M6_VC1_USE  \
    {                     \
        CG2214M6_VC1 = 1; \
        CG2214M6_VC2 = 0; \
    }
#define CG2214M6_VC2_USE  \
    {                     \
        CG2214M6_VC2 = 0; \
        CG2214M6_VC2 = 1; \
    }

#define CG2214M6_USE_T CG2214M6_VC2_USE
#define CG2214M6_USE_R CG2214M6_VC1_USE

/******************������data�Ĵ���************************************/

// #define Receiver_vent PC_IDR_IDR5       // Input   ���Ż���������ON/OFF
// #define Receiver_vent_direc PC_DDR_DDR5 // Input   ���Ż���������ON/OFF
// #define Receiver_vent_CR1 PC_CR1_C15    // Input   ���Ż���������ON/OFF
#define Receiver_vent Receiver_vent_Cache       // Input   ���Ż���������ON/OFF
#define Receiver_vent_direc Receiver_vent_Cache // Input   ���Ż���������ON/OFF
#define Receiver_vent_CR1 Receiver_vent_Cache   // Input   ���Ż���������ON/OFF

#define PIN_BEEP PA_ODR_ODR0       // Output   ������
#define PIN_BEEP_direc PA_DDR_DDR0 // Output   ������
#define PIN_BEEP_CR1 PA_CR1_C10    // Output   ������

#define Receiver_OUT_OPEN PD_ODR_ODR3       // Output   ���Ż��̵���OPEN  �ߵ�ƽ��Ч
#define Receiver_OUT_OPEN_direc PD_DDR_DDR3 // Output   ���Ż��̵���OPEN  �ߵ�ƽ��Ч
#define Receiver_OUT_OPEN_CR1 PD_CR1_C13    // Output   ���Ż��̵���OPEN  �ߵ�ƽ��Ч

#define Receiver_OUT_CLOSE PD_ODR_ODR2       // Output   ���Ż��̵���close  �ߵ�ƽ��Ч
#define Receiver_OUT_CLOSE_direc PD_DDR_DDR2 // Output   ���Ż��̵���close  �ߵ�ƽ��Ч
#define Receiver_OUT_CLOSE_CR1 PD_CR1_C12    // Output   ���Ż��̵���close  �ߵ�ƽ��Ч

#define Receiver_OUT_STOP PD_ODR_ODR1       // Output   ���Ż��̵���stop  �ߵ�ƽ��Ч
#define Receiver_OUT_STOP_direc PD_DDR_DDR1 // Output   ���Ż��̵���stop  �ߵ�ƽ��Ч
#define Receiver_OUT_STOP_CR1 PD_CR1_C11    // Output   ���Ż��̵���stop  �ߵ�ƽ��Ч

#define Receiver_OUT_VENT PD_ODR_ODR0       // Output   ���Ż��̵���VENT  �ߵ�ƽ��Ч
#define Receiver_OUT_VENT_direc PD_DDR_DDR0 // Output ���Ż��̵���VENT  �ߵ�ƽ��Ч
#define Receiver_OUT_VENT_CR1 PD_CR1_C10    // Output ���Ż��̵���VENT  �ߵ�ƽ��Ч

#define Inverters_OUT PA_IDR_IDR2       // ����   �̵�������źŷ���   �͵�ƽ��Ч
#define Inverters_OUT_direc PA_DDR_DDR2 // ����   �̵�������źŷ���   �͵�ƽ��Ч
#define Inverters_OUT_CR1 PA_CR1_C12    // ����   �̵�������źŷ���   �͵�ƽ��Ч

/*********************************************************************************/

#endif
