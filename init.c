//��ʼ������
#include "init.h"
#include <hidef.h>      /* common defines and macros */
#include <MC9S12XS128.h>     /* derivative information */

//*******************************����ģ��ĳ�ʼ������ *************************
void initGPIO(void){         //ͨ��IO�ڳ�ʼ��
    DDRA = 0x00;             //A�����룬������Ӧ�ý�����ͷ�������ź�
    DDRB = 0xFF;             //B�����
    DDRE = 0b01101100;            //E��2.3.5.6  ָʾ�����
	
	  LED1_DARK = 1;
    LED2_DARK = 1;
    LED3_DARK = 1;
    LED4_DARK = 1;
	
	DDRT = 0x00;           //T��0.1.2.3�ӵĲ��뿪��
	
  PIEJ_PIEJ6 = 1;                     
	PIEJ_PIEJ7 = 1;       //...............              
	PPSJ=0xFF;       //�����ش����ж�
	
  DDRP = 0xFF;          //������Ļ�ֻҪ�õ�PTP_PTP0��ΪIO��PP1��ΪPWM�����������ת����  ˫����õ�PTP_PTP2 ��PP3
  
}
void initPLL(void){                 //���໷��ʼ����������Ƶ�ʵ�����80M
	CLKSEL = 0x00;                 //��ֹ���໷��ʱ�����ⲿ�����ṩ������Ƶ��=�ⲿ����/2
    PLLCTL_CME = 1;         
    PLLCTL_SCME = 0;        
    CRGINT_SCMIE = 1;      
    PLLCTL_PLLON = 1;              //�����໷
    SYNR = 0xc9;           
    REFDV = 0x81;                  // pllclock=fvco=2*osc*(1+SYNR)/(1+REFDV)=80MHz; (osc=16MHz)
    POSTDIV = 0x00;                  
                                       
    _asm(nop);                     //BUS CLOCK=80M
    _asm(nop);
    while(CRGFLG_LOCK != 1) ;      // �ȴ����໷��ʼ�����
    CLKSEL_PLLSEL = 1;             // ʹ�����໷              
}
void initPWM(void){ 
       
     
    PWME = 0x00;                     
    PWMPOL = 0xFF;                      
    PWMPRCLK = 0x22;                    //CLOCK A��Bʱ�ӷ�Ƶ����ѡ��������ķ�Ƶ 20M
    PWMSCLA = 5;                        //CLOCK SA��CLOCK Aʮ��Ƶ��2M
    PWMSCLB = 5;                        //CLOCK SB��CLOCK Bʮ��Ƶ��2M
    PWMCTL = 0xF0;                      //01������23������45������67����
    PWMCLK = 0xFF;                      //PWMʼ��ѡ��ѡ��CLOCK SA SBΪPWMʱ��                   
    
    PWMPER01 = 1000;                    //��һ�����Ƶ��Ϊ1k
    PWMDTY01 = 0;
     
    PWMPER23 = 1000;                    //�ڶ������Ƶ��Ϊ1k
    PWMDTY23 = 0;                    
    
     
    PWME_PWME1 = 1;                     // ���PWM����ʼ���
    PWME_PWME3 = 1;
    PWME_PWME5 = 1;
    PWME_PWME7 = 1;   

}
void initAD(void)
{
    ATD0CTL1=0b00100000;//   10 λ���� 
    ATD0CTL2=0b01000000;//   ��ֹ�ⲿ����,��־λ��������,�жϽ�ֹ
    ATD0CTL3=0b10100000;/*   7;�Ҷ����޷���;
                             6~3:ת�����г���Ϊ4;
                             No FIFOģʽ,Freezeģʽ�¼���ת��?  */
    ATD0CTL4=0b00000111;//   4AD��������,ATDClock=[BusClock*0.5]/[PRS+1]  ; PRS=15, divider=32
    ATD0CTL5=0b00110000;//   ����ͨ����ֹ,��ͨ������,ɨ��ģʽ��������,��ʼΪ AN0
    ATD0DIEN=0b00000000;//   ��ֹ�������� 

    return;
}
void initRTI(void) {                      // ʵʱ�жϳ�ʼ��                     
    RTICTL = 0x83;                            //0.25ms�ж�
    CRGINT = 0x80;                            // ��ʵʱ�ж�
} 

void IOC_Init(void)
{   
    TCNT =0x00;
    TSCR1=0x80;//ʱ������
    TSCR2=0x07;//div by 128
    
    PACTL=0x50;//PT7 PIN,PACN32 16BIT,�����ؼ���,NOT INTERRUPT
    TIOS =0x00;//ÿһλ��Ӧͨ����: 0���벶׽,1����Ƚ�,�˴�PT5Ӧ�þ������벶׽��
//����EDGxB��EDGxAһ��������Ӧͨ�����벶׽���ԣ���Ӧ�Ĺ������£�
    /*��0 0��Ϊ��ֹ���벶׽��
��0 1��Ϊ�����ز�׽
��1 0��Ϊ�½��ز�׽
��1 1��Ϊ�����ػ��½��ز�׽   */
    TCTL3=0x44;//PT7��PT5��������Ч,             
    TIE  =0x20;//ÿһλ��Ӧ��Ӧͨ���ж�����,0��ʾ��ֹ�жϣ��˴�PT5�����벶׽��   
	
    TFLG1=0x20;//���жϱ�־λC5F
}

/***************     ��ʱ������1ms     **********************************/   
void initPIT(void)   //��ʱ�жϳ�ʼ������ 1MS��ʱ�ж����� 
{ 
    PITCFLMT_PITE=0; //��ʱ�ж�ͨ����     
    PITCE_PCE0=1;    //��ʱ��ͨ�� 0ʹ��  ��ʱ1ms��
    PITMUX_PMUX0=1;  //��ʱ��ͨ�� 0ѡ��8λ������1
    PITMTLD1=80-1;   //8λ������0��ֵ�趨��80��Ƶ���� 80MHzBusClock�£�Ϊ 1MHz���� 1us. 
    PITLD0=1000-1;   //16λ��������ֵ�趨��1000*1us=1ms 
    PITINTE_PINTE0=1;//���жϣ���ʱ���ж�ͨ�� 0�ж�ʹ��   
    PITCFLMT_PITE=1; //��ʱ��ͨ��ʹ�� 
    return;
}

/*******************   �г��жϳ�ʼ������  ���ж������ش���  ���ж��½��ش���*********/
void TIM_Init(void) 
{
TIOS =0x00;        //��ʱ��ͨ��0��1 Ϊ���벶׽
TSCR1=0x80;        //��ʱ��ʹ��
TCTL4=0x09;        //ͨ��0 ��׽������ͨ��1 ��׽�½���
TIE=0x03;          //ͨ��0��1 �ж�ʹ��
TFLG1=0xFF;        //���жϱ�־λ
}

/*********************   ����1��ʼ������ ������115200  ����SCI0 *************************/
void SCI0_Init()
{
SCI0BDL = (byte)((24000000 /* OSC freq /2*/) / 115200 /* baud rate */ / 16 /*factor*/);
SCI0CR1 = 0X00;                                      /*normal,no parity*/
SCI0CR2 = 0X0C;                                      /*RIE=1,TE=1,RE=1, */
}