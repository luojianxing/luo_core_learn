#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "adc.h"
#include "data_transfer.h"
#include "PID-V2.h"

u8 data[] = {0x01,0x02,0x03};
   	
 int main(void)
 { 
	u16 adcx;
	delay_init();	    	 //��ʱ������ʼ��	  
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ9600
//	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
	 NVIC_Configuration();
// 	Adc_Init();		  		//ADC��ʼ��	    
    
	while(1)
	{
//		 Data_Send_Version(3,300,100,400,0);//δ����,�ɻ��汾��Ϣ
//		 Data_Send_User(1000);
		
		  PID_main();
		  delay_ms(1);
	}											    
}	
