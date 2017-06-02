#ifndef _DATA_TRANSFER_H_
#define _DATA_TRANSFER_H_
#include "stm32f10x.h"
#include "usart.h"


extern u8 Data_Check,Send_Status,Send_Senser,Send_RCData,Send_Version,Send_Model,Send_Power,Send_GpsData,Send_Offset,Send_PID1,Send_PID2,Send_PID3,Send_MotoPwm;

void Data_Receive_Anl(u8 *data_buf,u8 num);
void Data_Exchange(void);

void Data_Send_Status(void);	
void Data_Send_Senser(void);	
void Data_Send_RCData(void);	
void Data_Send_OFFSET(void);	
void Data_Send_PID1(void);
void Data_Send_PID2(void);
void Data_Send_PID3(void);
void Data_Send_PID4(void);
void Data_Send_PID5(void);
void Data_Send_PID6(void);
void Data_Send_MotoPWM(void);
void Data_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver);
void Data_Send_Votage(float votage, float current);//·É»úµç³ØµçÑ¹;
void Data_Send_Check(u16 check);

void Data_Send_User(u16 data);

void NRF_Send_Test(void);

#endif
