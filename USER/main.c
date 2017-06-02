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
	delay_init();	    	 //延时函数初始化	  
	uart_init(115200);	 	//串口初始化为9600
//	LED_Init();		  		//初始化与LED连接的硬件接口
	 NVIC_Configuration();
// 	Adc_Init();		  		//ADC初始化	    
    
	while(1)
	{
//		 Data_Send_Version(3,300,100,400,0);//未调用,飞机版本信息
//		 Data_Send_User(1000);
		
		  PID_main();
		  delay_ms(1);
	}											    
}	
