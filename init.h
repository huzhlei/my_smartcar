#ifndef _INIT_H
#define _INIT_H


#define LED1_DARK       PORTE_PE2
#define LED2_DARK       PORTE_PE3
#define LED3_DARK       PORTE_PE5
#define LED4_DARK       PORTE_PE6


#define PWM_DEAD     0 
#define DTYMAX   300      //PWMå ç©ºæ¯”æœ€å¤§å€¼

#define ROW     40          //¶¨ÒåÍ¼Ïñ²É¼¯ĞĞÊı£º40ĞĞ
#define COLUMN  120         //¶¨ÒåÍ¼Ïñ²É¼¯ÁĞÊı£º120ÁĞ

#define WHITE 0
#define BLACK 1

#define car_center    92             //³µÄ£ÖĞĞÄÖµ
#define border 80                   //ÈüµÀºÚ°×ÅĞ¶Ï±ß½çÖµ

typedef unsigned char       uint8_t;
typedef unsigned int        uint16_t;
typedef unsigned char       uchar;

  void initPLL(void);
  void initRTI(void);
  void initGPIO(void);
  void IOC_Init(void);        
  void initPWM(void);
  void initAD(void);
  void initPIT(void);
  void TIM_Init(void);
  void SCI0_Init(); 


#endif