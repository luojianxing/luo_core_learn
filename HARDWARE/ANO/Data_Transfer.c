#include "data_transfer.h"
#include "stm32f10x.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

u8 Data_Check,Send_Status,Send_Senser,Send_RCData,Send_Power,Send_Version,Send_Model,Send_Offset,
   Send_PID1,Send_PID2,Send_PID3,Send_PID4,Send_PID5,Send_PID6,Send_MotoPwm;
u8 data_to_send[50];

void Data_Receive_Anl(u8 *data_buf,u8 num)//在串口3中断调用
{
	vs16 rc_value_temp;
	u8 sum = 0;u8 i;
	
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
/////////////////////////////////////////////////////////////////////////////////////
	
//	if(*(data_buf+2)==0X02)//ACK
//	{
//		if(*(data_buf+4)==0X01)
//		{
//			Send_PID1 = 1;
//			Send_PID2 = 1;
//			Send_PID3 = 0;
//			Send_PID4 = 0;
//			Send_PID5 = 0;
//			Send_PID6 = 0;
//		}
//		if(*(data_buf+4)==0X02)
//			Send_Offset = 1;
//        if(*(data_buf+4)==0XA0)//读取下位机版本
//        {
//            Send_Version=1;
//        }
//	}
//	if(*(data_buf+2)==0X10)								//PID1
//	{
//			PID_ROL.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_ROL.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/10000;
//			PID_ROL.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/50000;
//        
//			PID_PIT.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//			PID_PIT.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/10000;
//			PID_PIT.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/10000;
//        
//			PID_YAW.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
//			PID_YAW.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/10000;
//			PID_YAW.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/10000;
//			Data_Send_Check(sum);
//	}
	
//	if(*(data_buf+2)==0X16)								//OFFSET
//	{
//			AngleOffset_Rol = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
//			AngleOffset_Pit = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
//	}
}
void Data_Exchange(void)
{
	
    if(Send_Version)
    {
      Send_Version=0;
      Data_Send_Version(3,  //uint8HardwareType硬件种类
                        100,//uint16 HardwareVER*100硬件版本‘}=【
                        101,//uint16 SoftwareVER*100软件版本
                        100,//uint16 ProtocolVER*100协议版本
                        0); //uint16 BootloaderVER*100
    }
//    else if(Send_Model)
//    {
//      Send_Model=0;
//    }
    
//     Nvic_Init(USART2_ENABLE,USART3_ENABLE,TIM3_ENABLE,TIM4_DISENABLE,MPU6050_ENABLE);
}

void Data_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)//未调用,飞机版本信息
{
	u8 _cnt=0;u8 sum = 0,i=0;
	data_to_send[_cnt++]=0xAA;//帧头
	data_to_send[_cnt++]=0xAA;//帧头
	data_to_send[_cnt++]=0x00;//功能字
	data_to_send[_cnt++]=0;   //LEN 
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);//硬件version
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);//软件version
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);//协议version
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);//程序装载version
	data_to_send[_cnt++]=BYTE0(bootloader_ver);

	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;//校验位
	
	Uart1_Put_Buf(data_to_send,_cnt);
}
void Data_Send_User(u16 data)
{
	u8 _cnt=0;u8 sum = 0,i=0;
	vs16 _temp;
	
	_temp = data;
	
	
	data_to_send[_cnt++]=0xAA;//帧头
	data_to_send[_cnt++]=0xAA;//帧头
	data_to_send[_cnt++]=0xF1;//功能字
	data_to_send[_cnt++]=0;   //LEN 
	
	data_to_send[_cnt++]=BYTE1(_temp);//硬件version
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
//	data_to_send[_cnt++]=BYTE1(software_ver);//软件version
//	data_to_send[_cnt++]=BYTE0(software_ver);
//	data_to_send[_cnt++]=BYTE1(protocol_ver);//协议version
//	data_to_send[_cnt++]=BYTE0(protocol_ver);
//	data_to_send[_cnt++]=BYTE1(bootloader_ver);//程序装载version
//	data_to_send[_cnt++]=BYTE0(bootloader_ver);

	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;//校验位
	
	Uart1_Put_Buf(data_to_send,_cnt);
	
//	printf("\r\n");
	
}


//void Data_Send_PID4(void)
//{
//	u8 _cnt=0;u8 sum = 0;u8 i=0;
//    vs16 _temp;
//    
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x13;
//	data_to_send[_cnt++]=0;
//	
//	
//	_temp = PID_PID_5.P * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PID_5.I * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PID_5.D * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PID_6.P * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PID_6.I * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PID_6.D * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PID_7.P * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PID_7.I * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PID_7.D * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	
//	data_to_send[3] = _cnt-4;
//	
//	
//	for(i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	
//	data_to_send[_cnt++]=sum;
//	
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,_cnt);
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
//}
