#ifndef __RAM_H__
#define __RAM_H__
//CPU?????
//?????ADF030???????
//#define TX_TEST_CARRIER_MODE 0
#include "initial.h" // 锟斤拷始锟斤�? 预锟斤拷锟斤�?

#define ID_Max_PCS   255

//#define DEF_test_MAX_32pcs
extern u8 DEF_APP_TX_freq;

#define DEF_TX_TIMEOUT  250 //ms

extern uFLAG FLAG_test1;
//************************************************
#define FLAG_test1_BYTE FLAG_test1.BYTE
//------------------------------------------------
#define BIT_SIO FLAG_test1.BIT.Bit0
#define FG_auto_out FLAG_test1.BIT.Bit1
#define FG_OUT_OPEN_CLOSE FLAG_test1.BIT.Bit2 //锟斤拷史锟斤拷录   锟斤拷锟斤拷锟绞诧拷锟斤拷锟斤拷
#define FG_auto_open_time FLAG_test1.BIT.Bit4
#define FG_auto_manual_mode FLAG_test1.BIT.Bit5 //1=auto,0=manual
#define FG_Receiver_LED_RX FLAG_test1.BIT.Bit6
#define FG_First_auto FLAG_test1.BIT.Bit7
//************************************************
//************************************************
extern volatile uFLAG FLAG_test;
#define FLAG_test_BYTE FLAG_test.BYTE
//------------------------------------------------
#define FLAG_Receiver_Scanning FLAG_test.BIT.Bit0
#define FG_test_tx_1010 FLAG_test.BIT.Bit1
#define X_HIS FLAG_test.BIT.Bit2 //鍘嗗彶璁板綍   �?鐮佺巼娴嬭瘯�?
#define FG_test_tx_on FLAG_test.BIT.Bit3
#define FG_test_tx_off FLAG_test.BIT.Bit4
#define FG_test_mode FLAG_test.BIT.Bit5
#define FG_test1 FLAG_test.BIT.Bit6
#define FG_test_rx FLAG_test.BIT.Bit7
//************************************************
//************************************************
extern volatile uFLAG FLAG0;
#define FLAG0_BYTE FLAG0.BYTE
//------------------------------------------------
#define FLAG_Receiver_IDCheck FLAG0.BIT.Bit0
#define FLAG_Signal_DATA_OK FLAG0.BIT.Bit1
#define FLAG_APP_RX FLAG0.BIT.Bit2
#define FLAG_IDCheck_OK FLAG0.BIT.Bit3
#define FLAG_ID_Erase_Login FLAG0.BIT.Bit4
#define FLAG_ID_Erase_Login_PCS FLAG0.BIT.Bit5
#define FLAG_ID_Login FLAG0.BIT.Bit6
#define FLAG_ID_Login_OK FLAG0.BIT.Bit7
//************************************************
extern volatile uFLAG FLAG1;
#define FLAG1_BYTE FLAG1.BYTE
//------------------------------------------------
#define FLAG_Receiver_BEEP FLAG1.BIT.Bit0
#define FLAG_ID_Login_EXIT FLAG1.BIT.Bit1
#define FLAG_ID_Login_OK_bank FLAG1.BIT.Bit2
#define FG_beep_on FLAG1.BIT.Bit3
#define FG_beep_off FLAG1.BIT.Bit4
#define FG_allow_out FLAG1.BIT.Bit5
#define FG_NOT_allow_out FLAG1.BIT.Bit6
#define FG_10ms FLAG1.BIT.Bit7
//************************************************
extern volatile uFLAG FLAG2;
#define FLAG2_BYTE FLAG2.BYTE
//------------------------------------------------
#define FLAG_ID_SCX1801_Login FLAG2.BIT.Bit0   //用于与集中�?�信机�?�信的ID，只有一个ID
#define FG_ID_SCX1801_Login_BEEP FLAG2.BIT.Bit1
#define FLAG_APP_TX_fromUART FLAG2.BIT.Bit2
#define FLAG_APP_TX_fromOUT FLAG2.BIT.Bit3
#define FLAG_APP_TX FLAG2.BIT.Bit4
#define FLAG_APP_RXstart FLAG2.BIT.Bit5
#define FLAG_APP_TX_once FLAG2.BIT.Bit6
#define FLAG_Key_TP3 FLAG2.BIT.Bit7

//************************************************
extern volatile uFLAG FLAG3;
#define FLAG3_BYTE FLAG3.BYTE
//------------------------------------------------
#define Flag_ERROR_Read FLAG3.BIT.Bit0
//#define  FLAG3.BIT.Bit1
#define Flag_shutter_stopping FLAG3.BIT.Bit2
#define FLAG_APP_TX_fromUART_err_read FLAG3.BIT.Bit3
//#define  FLAG3.BIT.Bit4
//#define  FLAG3.BIT.Bit5
//#define  FLAG3.BIT.Bit6
//#define  FLAG3.BIT.Bit7

//************************************************


extern u16 X_COUNT;
extern u16 X_ERR; //记录错误的个�?
extern u8  X_ERR_CNT;
extern u16 X_ERRTimer;

extern u8 SIO_cnt;
extern u8 SIO_buff[16];
extern u8 SIO_DATA[16];
extern u8 Tx_Rx_mode;

extern u8 TIME_10ms;
extern u16 TIMER1s;
extern u16 TIMER300ms;
extern u16 TIMER18ms;
extern u16 TIMER250ms_STOP;
extern u16 TIME_auto_out;
extern u16 TIME_auto_close;
extern u16 time_3sec;
extern u32 ID_Receiver_DATA[256]; //写入EEPROM ID缓存
extern u32 ID_SCX1801_DATA;
extern u16 ID_DATA_PCS;
extern u32 DATA_Packet_ID;
extern u8 DATA_Packet_Control;
extern u8 DATA_Packet_Contro_buf; //2015.3.24�?�?
extern u32 ID_Receiver_Login;
extern u8 TIME_EMC;

extern u16 INquiry;
extern u16 TIME_Receiver_Login_restrict;
extern u8 COUNT_Receiver_Login;
extern u16 TIME_Receiver_Login;
extern u16 TIME_Login_EXIT_rest;
extern u16 TIME_Receiver_Login_led;
extern u16 TIME_ID_SCX1801_Login ;


extern u8 TIME_OUT_OPEN_CLOSE;
extern u16 TIME_Receiver_LED_OUT;
extern u16 TIME_Login_EXIT_Button;
extern u16 Manual_override_TIMER;
extern u16 time_Login_exit_256;
extern u16 TIME_Fine_Calibration; //绐勫甫涓嬩腑棰戞护娉㈠櫒100KHz绮炬�?

extern u8 Count_key_SW3;
extern u8 Display_key_SW3;

extern u16 TIME_TestNo91;
extern u16 TIME_power_led;
extern u8 write_flash_buffer[8];


extern ADF70XX_REG_T ROM_adf7030_value[16];
extern const ADF70XX_REG_T Default_adf7030_value[16];
/**********LCD******************/
extern uFLAG FlagLCDUpdate;

#define LCDUpdateIDFlag FlagLCDUpdate.BIT.Bit0
#define LCDUpdateRISSFlag FlagLCDUpdate.BIT.Bit1
#define LCDFirstDisplayFlag FlagLCDUpdate.BIT.Bit2
/**********RSSI*********************/
extern short RAM_RSSI_AVG;
extern long RAM_RSSI_SUM;
extern u8 RSSI_Read_Counter;
extern u8 Flag_RSSI_Read_Timer;

#define SPI_SEND_BUFF_LONG 300
#define SPI_REV_BUFF_LONG 40

extern const u8 ADF7030Cfg[];
extern const u8 ADF7030Cfg_load[];
extern const u8 ADF7030Cfg_4dot8k[];
extern const u8 *ADF7030Cfg_pointer;

u32 CFG_SIZE(void);

#define OPEN_LONG 12
//extern u8 CONST_TXPACKET_DATA_20000AF0[OPEN_LONG];
extern u8 CONST_TXPACKET_DATA_20000AF0[28];


extern u32 GENERIC_PKT_TEST_MODES0_32bit_20000548;
extern const u8 TEST_MODES0_para[5];
extern u32 RADIO_DIG_TX_CFG0_32bit_20000304;
#define AFCCONFIG_LONG 4
extern const u8 CONST_AFC_Configuration_400041F8[4];
extern u16 PA_POWER_OUT[2][18];
extern u32 PROFILE_CH_FREQ_32bit_200002EC;
extern u32 PROFILE_RADIO_AFC_CFG1_32bit_2000031C;
extern u32 PROFILE_RADIO_DATA_RATE_32bit_200002FC;
extern u32 PROFILE_GENERIC_PKT_FRAME_CFG1_32bit_20000500;
extern const u32 PROFILE_CH1_FREQ_32bit_429HighSpeed;
extern const u32 PROFILE_CH2_FREQ_32bit_429HighSpeed;
extern const u32 PROFILE_CH3_FREQ_32bit_429LowSpeed;
extern const u32 PROFILE_CH4_FREQ_32bit_429LowSpeed;
extern u8 Channels;



void SPI_INIT(void);
u32 ADF7030_GET_MISC_FW(void); //??MISC_FW?????
void ADF7030_WRITING_PROFILE_FROM_POWERON(void);
void ADF7030_TRANSMITTING_FROM_POWEROFF(void);
void ADF7030_RECEIVING_FROM_POWEROFF(void);
void SCAN_RECEIVE_PACKET(void);
u32 ConfigurationLen(void);





extern Wireless_Body Struct_DATA_Packet_Contro,Struct_DATA_Packet_Contro_buf;
extern Wireless_Body Uart_Struct_DATA_Packet_Contro,Last_Uart_Struct_DATA_Packet_Contro;
extern u8 Struct_DATA_Packet_Contro_fno;
extern u16 TIMER_Semi_open;
extern u16 TIMER_Semi_close;
extern u8 FLAG__Semi_open_T;
extern u8 FLAG__Semi_close_T;
extern u16 TIME_APP_TX_fromOUT;
extern u8 Time_APP_blank_TX;
extern u16 Time_APP_RXstart;

extern u8 TIME_TX_RSSI_Scan;
extern u8 Flag_TX_ID_load;
extern u8 TIME_ERROR_Read_once_again;
extern  unsigned char Send_err_com[7] ;
extern u8 Time_error_read_gap;
extern u16 Time_error_read_timeout;
extern u8 ERROR_Read_sendTX_count;
extern u8 ERROR_Read_sendTX_packet;
extern u8 Flag_ERROR_Read_once_again;
extern u16 Time_rf_init;
extern u16 timeout_cnt;
extern u8 retx_cnt;
extern u16 time_cnt;
//-----------------------------------------------------------------------------------------------
typedef union{
    u8 un_var;
    struct{
        u8 un_var_bit0 : 1;
        u8 un_var_bit1 : 1;
        u8 un_var_bit2 : 1;
        u8 un_var_bit3 : 1;
        u8 un_var_bit4 : 1;
        u8 un_var_bit5 : 1;
        u8 un_var_bit6 : 1;
        u8 un_var_bit7 : 1;
    };
}Mark_Un;

extern Mark_Un mark0_stu;

#define key_sta     mark0_stu.un_var_bit0
#define Key1_press  1
#define Key2_press  2
#define Key3_press  3
#define Flag_FREQ_Scan   mark0_stu.un_var_bit1
#define Flag_rx_pream    mark0_stu.un_var_bit2
#define Flag_set_freq    mark0_stu.un_var_bit3
#define Flag_rx_data     mark0_stu.un_var_bit4
#define Flag_tx_en       mark0_stu.un_var_bit5
#define Flag_TxDone      mark0_stu.un_var_bit6
#define Flag_RxDone      mark0_stu.un_var_bit7

extern Mark_Un mark1_stu;
#define Flag_test_mode   mark1_stu.un_var_bit0
#define Flag_test_fm     mark1_stu.un_var_bit1
#define Flag_test_rssi   mark1_stu.un_var_bit2
#define Flag_test_pc     mark1_stu.un_var_bit3
#define Flag_PROFILE_RxLowSpeed_TYPE     mark1_stu.un_var_bit4

#define Tx_No_Status           0xD0  /* ć˛Ąćçść */
#define Tx_Open_Status         0xD1  /* ĺźçść */
#define Tx_Close_Status        0xD2  /* é­çść */
#define Tx_Open_Action_Status  0xD3  /* ĺźĺ¨ä˝ä¸­çść */
#define Tx_Close_Action_Status 0xD4  /* é­ĺ¨ä˝ä¸­çść */
#define Tx_Open_Action_Auto    0xD5  /* ĺźĺ¨ä˝ä¸­çść(čŞĺ¨ĺäżĄćś) */
#define Tx_Close_Action_Auto   0xD6  /* é­ĺ¨ä˝ä¸­çść(čŞĺ¨ĺäżĄćś) */
#define Tx_Abnormal_Status     0xD7  /* ĺźĺ¸¸1çść */
#define Tx_Setting_Status      0xD8  /* ćśé´čŽžĺŽĺčé¸Łĺ¨ON/OFFčŽžĺŽćś,éäżĄ */
#define Tx_Open_StatusNG       0xD9  /* ĺźçść(ć ć),čŞĺ¨äżĄĺˇć ćççść */
#define Tx_Close_StatusNG      0xDA  /* é­çść(ć ć),čŞĺ¨äżĄĺˇć ćççść */
#define Tx_Open_Action_StatusNG     0xDB  /* ĺźĺ¨ä˝ä¸­(ć ć),čŞĺ¨äżĄĺˇć ćççść */
#define Tx_Close_Action_StatusNG    0xDC  /* é­ĺ¨ä˝ä¸­(ć ć),čŞĺ¨äżĄĺˇć ćççść */
#define Tx_Abnormal_StatusNG        0xDD

#endif
