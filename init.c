//初始化程序
#include "init.h"
#include <hidef.h>      /* common defines and macros */
#include <MC9S12XS128.h>     /* derivative information */

//*******************************各个模块的初始化函数 *************************
void initGPIO(void){         //通用IO口初始化
    DDRA = 0x00;             //A口输入，理论上应该接摄像头的输入信号
    DDRB = 0xFF;             //B口输出
    DDRE = 0b01101100;            //E口2.3.5.6  指示灯输出
	
	  LED1_DARK = 1;
    LED2_DARK = 1;
    LED3_DARK = 1;
    LED4_DARK = 1;
	
	DDRT = 0x00;           //T口0.1.2.3接的拨码开关
	
  PIEJ_PIEJ6 = 1;                     
	PIEJ_PIEJ7 = 1;       //...............              
	PPSJ=0xFF;       //上升沿触发中断
	
  DDRP = 0xFF;          //单电机的话只要用到PTP_PTP0作为IO，PP1作为PWM输出进行正反转控制  双电机用到PTP_PTP2 和PP3
  
}
void initPLL(void){                 //锁相环初始化，将总线频率调整到80M
	CLKSEL = 0x00;                 //禁止锁相环，时钟有外部晶振提供，总线频率=外部晶振/2
    PLLCTL_CME = 1;         
    PLLCTL_SCME = 0;        
    CRGINT_SCMIE = 1;      
    PLLCTL_PLLON = 1;              //打开锁相环
    SYNR = 0xc9;           
    REFDV = 0x81;                  // pllclock=fvco=2*osc*(1+SYNR)/(1+REFDV)=80MHz; (osc=16MHz)
    POSTDIV = 0x00;                  
                                       
    _asm(nop);                     //BUS CLOCK=80M
    _asm(nop);
    while(CRGFLG_LOCK != 1) ;      // 等待锁相环初始化完成
    CLKSEL_PLLSEL = 1;             // 使用锁相环              
}
void initPWM(void){ 
       
     
    PWME = 0x00;                     
    PWMPOL = 0xFF;                      
    PWMPRCLK = 0x22;                    //CLOCK A，B时钟分频，均选择从总线四分频 20M
    PWMSCLA = 5;                        //CLOCK SA从CLOCK A十分频，2M
    PWMSCLB = 5;                        //CLOCK SB从CLOCK B十分频，2M
    PWMCTL = 0xF0;                      //01级联，23级联，45级联，67级联
    PWMCLK = 0xFF;                      //PWM始终选择，选择CLOCK SA SB为PWM时钟                   
    
    PWMPER01 = 1000;                    //第一个电机频率为1k
    PWMDTY01 = 0;
     
    PWMPER23 = 1000;                    //第二个电机频率为1k
    PWMDTY23 = 0;                    
    
     
    PWME_PWME1 = 1;                     // 电机PWM波开始输出
    PWME_PWME3 = 1;
    PWME_PWME5 = 1;
    PWME_PWME7 = 1;   

}
void initAD(void)
{
    ATD0CTL1=0b00100000;//   10 位精度 
    ATD0CTL2=0b01000000;//   禁止外部触发,标志位快速清零,中断禁止
    ATD0CTL3=0b10100000;/*   7;右对齐无符号;
                             6~3:转换序列长度为4;
                             No FIFO模式,Freeze模式下继续转换?  */
    ATD0CTL4=0b00000111;//   4AD采样周期,ATDClock=[BusClock*0.5]/[PRS+1]  ; PRS=15, divider=32
    ATD0CTL5=0b00110000;//   特殊通道禁止,多通道采样,扫描模式连续采样,开始为 AN0
    ATD0DIEN=0b00000000;//   禁止数字输入 

    return;
}
void initRTI(void) {                      // 实时中断初始化                     
    RTICTL = 0x83;                            //0.25ms中断
    CRGINT = 0x80;                            // 打开实时中断
} 

void IOC_Init(void)
{   
    TCNT =0x00;
    TSCR1=0x80;//时钟允许
    TSCR2=0x07;//div by 128
    
    PACTL=0x50;//PT7 PIN,PACN32 16BIT,上升沿计数,NOT INTERRUPT
    TIOS =0x00;//每一位对应通道的: 0输入捕捉,1输出比较,此处PT5应该就是输入捕捉了
//其中EDGxB和EDGxA一起来设相应通道输入捕捉极性，对应的功能如下：
    /*（0 0）为禁止输入捕捉。
（0 1）为上升沿捕捉
（1 0）为下降沿捕捉
（1 1）为上升沿或下降沿捕捉   */
    TCTL3=0x44;//PT7和PT5上升沿有效,             
    TIE  =0x20;//每一位对应相应通道中断允许,0表示禁止中断，此处PT5的输入捕捉打开   
	
    TFLG1=0x20;//清中断标志位C5F
}

/***************     定时器函数1ms     **********************************/   
void initPIT(void)   //定时中断初始化函数 1MS定时中断设置 
{ 
    PITCFLMT_PITE=0; //定时中断通道关     
    PITCE_PCE0=1;    //定时器通道 0使能  定时1ms用
    PITMUX_PMUX0=1;  //定时器通道 0选择8位计数器1
    PITMTLD1=80-1;   //8位计数器0初值设定。80分频，在 80MHzBusClock下，为 1MHz。即 1us. 
    PITLD0=1000-1;   //16位计数器初值设定。1000*1us=1ms 
    PITINTE_PINTE0=1;//开中断，定时器中断通道 0中断使能   
    PITCFLMT_PITE=1; //定时器通道使能 
    return;
}

/*******************   行场中断初始化函数  行中断上升沿触发  场中断下降沿触发*********/
void TIM_Init(void) 
{
TIOS =0x00;        //定时器通道0，1 为输入捕捉
TSCR1=0x80;        //定时器使能
TCTL4=0x09;        //通道0 捕捉上升沿通道1 捕捉下降沿
TIE=0x03;          //通道0，1 中断使能
TFLG1=0xFF;        //清中断标志位
}

/*********************   串口1初始化函数 波特率115200  串口SCI0 *************************/
void SCI0_Init()
{
SCI0BDL = (byte)((24000000 /* OSC freq /2*/) / 115200 /* baud rate */ / 16 /*factor*/);
SCI0CR1 = 0X00;                                      /*normal,no parity*/
SCI0CR2 = 0X0C;                                      /*RIE=1,TE=1,RE=1, */
}