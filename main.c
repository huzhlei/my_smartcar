#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "init.h"
#include "math.h"

/////////////////////////////////////////////////////////////////////
//�ó���PLL 160M��BUS 80M
/////////////////////////////////////////////////////////////////////
#pragma CODE_SEG DEFAULT

int i,j,k; 
uint a;

unsigned char TimerFlag20ms = 0;
             
int leftpulse = 0;
int L_BLACK,R_BLACK;       //���߰��ߵ�λ��
int Line_center = 0;       //���������ߵ�λ��
int flag = 0;

int start_time=0;
int kaiguan=0;


//****************************** ����ͷ�ɼ����ݲ��� **************************
#define row_num 30                  //ͼ��ɼ�����
#define col_num 184                 //ͼ��ɼ����� 196

#define yuzhi 50                    //�����ڰײ�ֵ 


///////////////////����ͷͼ��ɼ�/////////////////
uchar image[row_num][col_num];      //ͼ��洢����
uchar image_v_num=0;                //�Ѳɼ���������ͷ�ĵ�ǰ��
uchar v_counter=0;                  //�Ѷ�ȡ������
uchar ccd_state=0;
unsigned int c=0;
unsigned int All_VSYN_Count=0;     //����ͷ�ɼ��ĳ���
//--------------����ͷҪ�ɼ�����----------------------------
const int selectRow[]= 
{
50,   55,   60,   65,   70,   75,   80,   85,   90,   95,   
100,  105,  110,  115,  120,  125,  130,  135,  140,  145, 
150,  155,  160,  165,  170,  175,  180,  185,  190,  195,
//200,  204,  208,  212,  216,  220,  224,  228,  232,  236,
};
//////////////////////������ȡ////////////////////
uchar center_line[row_num];         //�е����� ,����0��ʾû���ҵ�����
uchar Lx[row_num];                    //�����������ĵ��к�,û���ҵ���ֵΪcol_num
uchar Rx[row_num];                    //�����������ĵ��к�,û���ҵ���ֵΪ0
uchar BW_DELTA=40;                //�ڰ��ߵĲ�ֵ
uchar isLeftEdge=0,isRightEdge=0; //�Ƿ񿴵����ұ߽� 
uchar L_LeftEdge=col_num,L_RightEdge=0; //�Ƿ񿴵����ұ߽�         
uchar Distance[row_num]=              //���߾������ĵľ���
{ 
80,78,76,74,72,70,67,65,63,60,
58,55,53,50,47,45,42,39,35,33,
 };  
uchar Distance1[row_num]= 
{
110,109,108,107,106,105,104,103,102,101,
100,99,98,97,96,95,94,93,92,90,
}  ;

//****************************** ������� **************************
#define steer_center 2900 
#define steer_left 2500             //���������ֵ
#define steer_right 3300            //����ҹ����ֵ


int steer_out=steer_center;         //������������
int pre_steer_out=0;                //ǰһ�εĶ��������
float steer_p_near=3;                 //�������Pֵ
float steer_p_mid=5;                  //�м���Pֵ
float steer_p_far=4;                  //Զ�����Pֵ
int near_piancha;                   //���������복ģ����ƫ��
int mid_piancha;                    //�м�����복ģ����ƫ��
int far_piancha;                    //Զ�������복ģ����ƫ��
int allpiancha;

int LRightEdge,LLeftEdge;
unsigned char C_Start,dot;      //��ʼ����ս��

int allpiancha;

float SteerKd=0.3;
int SteerE0=0;
int SteerE1=0;
uint Count_Num=0;

int yanshi=3000;



//****************************** ����ܳ�ʼ������  *************************

unsigned char Led[] = {0xFB,0xF7,0xDF,0xBF};                        //PE2.3.5.6����ʾ
unsigned char LedCode[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};    //�������ʾ
unsigned char LedData[]={0,0,0,0};
unsigned char LedNum = 0;

int time = 0;
int count = 0;
int nCntTime = 0;

//****************************** AD�ɼ��ĳ�ʼ������ *************************
word AD_CAIJI[2]={0,0};         //�ɼ���AD��ֵ��ʱ�������
word AD_QIUHE[2]={0,0};         //�����ʱ�������
word AD_JUNZHI[2]={0,0};        //ƽ��ֵ
volatile byte PIT_CNT=0;        //�жϼ���
volatile int  ANG=0;            //�Ƕ�
volatile int  GYRO=0;           //���ٶ�

volatile int  AD_number=20;     //������� 

volatile int DTY1=0;            //��ʱ����
volatile int DTY2=0;            //��ʱ����

//***********************��������ʼ������ *********************************
unsigned char EventCount=0,SpeedControlPeriod=0,DirectionControlPeriod=0,InputVoltageCount=0,SpeedControlCount=0,DirectionControlCount=0,BMQ_count=0;

long MotorOut=0;

long LeftMotorPulseSigma=0,RightMotorPulseSigma=0,InputVoltageSigma[5]=0;

/*************************�ٶ�PID*************************************/
unsigned int Sudu_PID_P =2.2,Sudu_PID_I=0,Sudu_PID_D = 0;
signed int SpeedLeft_now=0,SpeedRight_now=0,Speed_want=100;

//signed int speed[6]={140,20,140,20,140,18} ;
signed int speed[6]={20,20,140,20,140,18}; 
int Speed_Break_Flag1=0,Speed_Break_Flag2=0,Speed_Break_Count=0;
long Speed_P=0,Speed_I=0,Speed_D=0,Speed_error=0,Speed_error_old=0,Speed_error_error=0;
float Speed=0,SpeedControlOutOld=0,fValue=0,CarSpeed=0,CarSpeed1=0,CarSpeed2=0;
float SpeedControlOut=0;
int Speed_left=0,Speed_right=0;
int RightMotorOut = 0,LeftMotorOut = 0;
    
unsigned int g_fSpeedControlIntegral = 0;
/*************************����PID******************************/
signed int DirectionControlOut;

/**********************�������˲�*****************************************/
float gyro=0,acceler=0,Vref=2.5,jiaodu_jifen=0,jiaodu_error=0; //2.048
float angle=0, angle_dot=0;
long Tg=3;
unsigned int fangdabeishu =3;//15
float C_0 = 1;
float Q_angle=0.001;
float Q_gyro=0.003;
float R_angle=0.5;
float dt=0.006;
float P[2][2] = {{ 1, 0 },
{ 0, 1 }
};
float Pdot[4] ={0,0,0,0};
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
/************************ֱ��PID************************************/
//unsigned int ACCE_CENTER=820,GYRO_CENTER=580;
//unsigned int ACCE_CENTER=244,GYRO_CENTER=440;

unsigned int ACCE_CENTER=244,GYRO_CENTER=444; 
 //unsigned int ACCE_CENTER=250,GYRO_CENTER=427;
//unsigned int ZHILI_PID_P = 570,ZHILI_PID_D=33, ZHILI_PID_I = 0;
unsigned int ZHILI_PID_P= 750,ZHILI_PID_D=30, ZHILI_PID_I = 0;
//unsigned int ZHILI_PID_P = 800,ZHILI_PID_D=40,ZHILI_PID_I = 0;
int PWM_DEADL=80;
int PWM_DEADR=67;
float ZHILI_PD=0,ZHILI_I=0,ZHILI=0,PIANJIAO=0;


//***************************************** �ٶȲɼ� *****************************************
void GetLeftMotorPulse(void)
{
  SpeedLeft_now=leftpulse;
  leftpulse=0;
  if(LeftMotorOut<0)  SpeedLeft_now = -SpeedLeft_now;
  LeftMotorPulseSigma+=SpeedLeft_now;
}
void GetRightMotorPulse(void)
{
  SpeedRight_now=PACNT;
  PACNT=0;
  if(RightMotorOut<0)  SpeedRight_now = -SpeedRight_now;
  RightMotorPulseSigma+=SpeedRight_now;
}


/*****************************************�ٶ�PID����***************************************/
void SpeedControl(void)
{
  CarSpeed=(LeftMotorPulseSigma+RightMotorPulseSigma)/2 ;
  LeftMotorPulseSigma=0;
  RightMotorPulseSigma=0; 
  if(start_time < 26000) 
  {
     Speed_want=40;
     //DirectionControlOut=0;     
    
  } 
  else if(start_time>26000 || start_time==26000)
  {
    start_time=27000;
    kaiguan=PTT & 0x0F;
    if( kaiguan == 1)
     Speed_want = 80; 
    } 
    else if(kaiguan == 4)
     Speed_want = 100;
    else if(kaiguan == 5)
     Speed_want = 120;
     else if(kaiguan == 8)
     Speed_want = 140;    
     else if(kaiguan ==12) 
     Speed_want = 160;   
   
  }
  Speed_error=Speed_want-CarSpeed;
  Speed_P=Speed_error*Sudu_PID_P;
  Speed_I =Speed_error*Sudu_PID_I;
  g_fSpeedControlIntegral +=Speed_I;
  
  SpeedControlOutOld=Speed;
  Speed=Speed_P+g_fSpeedControlIntegral;
}
void SpeedControlOutput(void)
{
    fValue = Speed - SpeedControlOutOld;
    SpeedControlOut = fValue * (SpeedControlPeriod + 1) / 100 + SpeedControlOutOld;
}


//************************* AD�ɼ����� ******************************
void fAD_CAIJI(unsigned int *AD_val)    //AD�ɼ�����ͨ��
{
	  while(!ATD0STAT2_CCF0);//0ͨ��ת�����ǰ�ȴ����ɼ����ٶ�
	  *AD_val=ATD0DR0;
	  AD_val++;
	  while(!ATD0STAT2_CCF1);//1ͨ��ת�����ǰ�ȴ����ɼ��Ƕ�
	  *AD_val=ATD0DR1;  	 
	  return;
}

void fAD_QIUHE(void) 
{
    word i;
    
    
    for(i=0;i<AD_number;i++) 
      {
        fAD_CAIJI(AD_CAIJI);                                            
        AD_QIUHE[0]+=AD_CAIJI[0];     //��ȡAD0ͨ��ֵ��20�κ�
        AD_QIUHE[1]+=AD_CAIJI[1];     //��ȡAD1ͨ��ֵ��20�κ�
      }
    return;   
}   

void fAD_QIUJUNZHI()                  //��ƽ��ֵ
{ 
    AD_JUNZHI[0]=(word)AD_QIUHE[0]/AD_number;
    AD_JUNZHI[1]=(word)AD_QIUHE[1]/AD_number;
    AD_QIUHE[0]=0;                     
    AD_QIUHE[1]=0;
    return;    
}
void Kalman_Filter(float angle_m,float gyro_m) //gyro_m:gyro_measure
{
  angle+=(gyro_m-q_bias) * dt;//�������
  angle_err = angle_m - angle;//zk-�������
  Pdot[0]=Q_angle - P[0][1] - P[1][0];// Pk-' ����������Э�����΢��
  Pdot[1]=- P[1][1];
  Pdot[2]=- P[1][1];
  Pdot[3]=Q_gyro;
  P[0][0] += Pdot[0] * dt;// Pk- ����������Э����΢�ֵĻ��� = ����������Э����
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;//����������Э����
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err;//�������
  q_bias += K_1 * angle_err;//�������
  angle_dot = gyro_m-q_bias;
}

void AD_calculate(void)
{
  acceler=AD_JUNZHI[1];
  gyro=AD_JUNZHI[0];
  gyro=(GYRO_CENTER-gyro)*Vref/(1024*0.00067*fangdabeishu);
  acceler=(acceler-ACCE_CENTER )*Vref/(1024*0.8)*60;
  Kalman_Filter(acceler,gyro); //�������˲�  

}



void zhili_kongzhi(void)
{
  ZHILI_PD=(0-angle-(PIANJIAO/100))*ZHILI_PID_P+(0-angle_dot)*ZHILI_PID_D;
  ZHILI_I +=(0-angle-(PIANJIAO/100))*ZHILI_PID_I;
  if(ZHILI_I>900) ZHILI_I=900;
  if(ZHILI_I<-900) ZHILI_I=-900;
  ZHILI=ZHILI_PD/10+ZHILI_I/10;
}


//************************* ����ͷ���ݲɼ��ʹ����� ******************************
int abs(int x){
  if(x>=0) return x;
  else return (-x);
}

int lvbo(unsigned int a,unsigned int b,unsigned int c){   
  unsigned int x=0;
  if(b==0){
    return (a+c)/2;
  }
  if(a>b){x=b;b=a;a=x;}
  if(b>c){x=c;c=b;b=x;}
  if(a>b){x=b;b=a;a=x;}
  return b ;
}

void Rightline(void)   //ɨ��ɼ�ͼ���е�����
{
  for(i=0;i<row_num;i++) {     
    for(j=car_center;j<col_num;j++){
      if((image[i][j]-image[i][j+2])>yuzhi || (image[i][j]-image[i][j+3])>yuzhi){
      //�ڰ��������ؿ��ܻ���ģ��ƫ��
        if(image[i][j+3]<border){
          Rx[i]=(j+2);
          break;
        }
      }
    if(j==(col_num-1)) Rx[i]=col_num;        //û������������ 
    }
  }
}//Rightline()�������� 
          
void Leftline(void)  //ɨ��ɼ�ͼ���е�����
{
  for(i=0;i<row_num;i++) {     
    for(j=car_center;j>=0;j--){
      if((image[i][j]-image[i][j-2])>yuzhi || (image[i][j]-image[i][j-3])>yuzhi){
      //�ڰ��������ؿ��ܻ���ģ��ƫ��      
        if(image[i][j-3]<border){
          Lx[i]=(j-2);
          break;
        }
      }
      if(j==0) Lx[i]=0;        //û������������ 
    }
  }
}//Leftline()�������� 

void CenterLine(void) {
  int temp=0;
  for(i=row_num-1;i>=0;i--) 
  {
      center_line[i]=(Lx[i]+Rx[i])/2;
  }
  for(i=row_num-2;i>=0;i--) 
    {temp=lvbo(center_line[i-1],center_line[i],center_line[i+1]); 
      center_line[i]=temp;}
  //�˲�֮��Ϊ-1�ģ�˵�������ܵ���   
}


void steer_PD(void) //���PD����
{ 
  if(center_line[16]!=0 && center_line[16]<120){
  mid_piancha= car_center-(int)center_line[14];
  }
 if(center_line[6]!=0 && center_line[8]<120){
    far_piancha= car_center-(int)center_line[8];
  } 
  
  /*if(center_line[24]!=0 && center_line[22]<120){
    near_piancha= car_center-(int)center_line[22];
  }*/
  near_piancha=0;
  allpiancha=steer_p_near*(float)(near_piancha)+steer_p_mid*(float)(mid_piancha)+steer_p_far*(float)(far_piancha);          
  SteerE0=allpiancha; 
  steer_out=SteerE0+SteerKd*(SteerE0-SteerE1);  //��ƫ�ң��������Ҵ�steer_outӦ����
  SteerE1=SteerE0;   
  DirectionControlOut= 0.15*steer_out;   
}


void final_control(void){
       Leftline(); 
       Rightline();
       CenterLine();
       steer_PD(); 
} 

/****************************************����PID*********************************************/

/*
void DirectionControlOutput(void)
{
  float Value;
  Value = allpiancha;
  DirectionControlOut = Value * DirectionControl_p;
}
*/


//************************* ������տ���  ********************
    
void PWM_out()
{


	  
	  if (LeftMotorOut > 0 || LeftMotorOut == 0)
		{
		      PTP_PTP2=0;
		     	PWMDTY23 = LeftMotorOut + PWM_DEADL;
		      if (PWMDTY23 > DTYMAX)   PWMDTY23 = DTYMAX; 	
		      if (RightMotorOut > 0 || RightMotorOut == 0)
		     {
      		  PTP_PTP0=0;
      		  PWMPOL = 0x0f;
      			PWMDTY01 = RightMotorOut + PWM_DEADR;   
      			if (PWMDTY01 > DTYMAX) 	PWMDTY01 = DTYMAX; 
      		}
      		else
	        {
      		  PTP_PTP0=1;
      		  PWMPOL = 0x0c;
      			PWMDTY01 = -RightMotorOut + PWM_DEADR;    
      			if (PWMDTY01 > DTYMAX)  PWMDTY01 = DTYMAX;
      				
      		}
		}	 
		else
		{
    		  PTP_PTP2=1;
    			PWMDTY23 = -LeftMotorOut + PWM_DEADL;  
    			if (PWMDTY23 > DTYMAX)	   PWMDTY23 = DTYMAX;   
    			
    			if (RightMotorOut > 0 || RightMotorOut == 0)
		     {
      		  PTP_PTP0=0;
      		  PWMPOL = 0x03;
      			PWMDTY01 = RightMotorOut + PWM_DEADR;   
      			if (PWMDTY01 > DTYMAX) 	PWMDTY01 = DTYMAX; 
      		}
      		else
	        {
      		  PTP_PTP0=1;
      		  PWMPOL = 0x00;
      			PWMDTY01 = -RightMotorOut + PWM_DEADR;    
      			if (PWMDTY01 > DTYMAX)  PWMDTY01 = DTYMAX;
      				
      		}
		}
		
}

void MotorOutput(void)
{

  //DirectionControlOut=0;
  //SpeedControlOut=0;
  if(DirectionControlOut>0) {
    
  LeftMotorOut =(int)(ZHILI - SpeedControlOut + DirectionControlOut);
  RightMotorOut =(int)(ZHILI - SpeedControlOut- 4*DirectionControlOut);

  }
   else if(DirectionControlOut<0 || DirectionControlOut==0) {
    
  LeftMotorOut =(int)(ZHILI - SpeedControlOut + 4*DirectionControlOut);
  RightMotorOut =(int)(ZHILI - SpeedControlOut - DirectionControlOut);
  }

  
   PWM_out();
   
}

void delay(int t)
{
   int ii,jj;
   if (t<1) t=1;
   for(ii=0;ii<t;ii++)
      //for(jj=0;jj<3338;jj++);    //busclk:40MHz--1ms
     //for(jj=0;jj<2670;jj++);    //busclk:32MHz--1ms
     for(jj=0;jj<6675;jj++);    //busclk:80MHz--1ms         
}
void main(void) {
  /* put your own code here */
  
  DisableInterrupts;
  initPLL();
  initRTI();
  initGPIO();
  IOC_Init();        
  initPWM();
  initAD();
  initPIT();
  PTS = 0x00;

  EnableInterrupts;
     
  for(;;) {
  
        if(ccd_state==1) 
        {
          final_control();
          ccd_state=0;
         }
          
    _FEED_COP(); /* feeds the dog */
  } /* loop forever */
  /* please make sure that you never leave main */
}
#pragma CODE_SEG __NEAR_SEG NON_BANKED
void interrupt 7 RTI_INT(void) {
  time++;
  if(time >=500){            //����ܼ���
      a++;
      time=0;
      count ++;
      LedData[0] = a/1000;
      LedData[1] = a/100%10;
      LedData[2] = a/10%10;
      LedData[3] = a%10;
  }

  PTS = ~(0x01 << LedNum) ;

  PORTB = LedCode[LedData[LedNum]];
  
  LedNum++;
  if(LedNum >= 4) LedNum = 0;
  CRGFLG |= 0x80;
}

#pragma CODE_SEG DEFAULT
#pragma CODE_SEG __NEAR_SEG NON_BANKED 
//1ms�ж�
void interrupt 66 PIT0_ISR(void) 
{         
    PITTF_PTF0=1;           //���жϱ�־λ 
    PIT_CNT++;
    start_time++;
    SpeedControlPeriod++;
    SpeedControlOutput();
   
    if(PIT_CNT==1) 
    {      
       fAD_QIUHE() ;        //ȡ20��AD����ĺ�        
    }
    else if(PIT_CNT==2) {
      
       fAD_QIUJUNZHI();     //20��AD�����ƽ��ֵ
       AD_calculate();
       zhili_kongzhi();
       MotorOutput();
    }
   
    else if(PIT_CNT==3) {
     SpeedControlCount ++;
     if(SpeedControlCount >= 20)
     {
       SpeedControl();
       SpeedControlCount = 0;
       SpeedControlPeriod = 0;
     } 
    }
    else if(PIT_CNT==4) {
      DirectionControlCount ++;
      TimerFlag20ms++;
      if(DirectionControlCount >= 2)
      {
        //saidao();
        //DirectionControlOutput();
        DirectionControlCount = 0;
        DirectionControlPeriod = 0;
      }
      
    }
   
    else if(PIT_CNT>=5)
    {            
        PIT_CNT=0;
        BMQ_count++;        
       
        if(BMQ_count == 2) 
        {
          BMQ_count = 0;
          GetLeftMotorPulse();
          GetRightMotorPulse();          
        }
       
    }   
}

interrupt 24 void V_SYNC(void)
{     //�г��жϷ����Ӻ���
  //--------------------------------
  if (PIFJ_PIFJ6)                   //���ж�
  { 
    All_VSYN_Count++;                       //4 VSync
    image_v_num=0;                  ////�Ѳɼ���������ͷ�ĵ�ǰ��������
    if (All_VSYN_Count%2 == 1)    // �泡�ɼ�ͼ��
 	  {
 	    //ccd_state=0;
 	    PIEJ_PIEJ7 = 1;                      // �������ж�
 	  }
 	  else                                     // ż��������
 	  {
 	    ccd_state=1;  
 	    PIEJ_PIEJ7 = 0;                      //�ر����ж�
 	  }
    v_counter=0;                    //�Ѷ�ȡ������,�ɼ�������
    c=0; 
    PIFJ_PIFJ6=1;
  } 
  else if (PIFJ_PIFJ7)              //���ж�
  {
   //way2-��������ͷ---------------------------------
   image_v_num++;                     //��¼���ֶ��ٴ����ж�,��������
   if(v_counter<row_num) 
   {
     c=0;
     if(image_v_num==selectRow[v_counter]) 
     {
       //�˴��ɼ���ʱ�����ε����ɼ���ͼ������ҶԳ���
       while (c<col_num)
       {    //17
         image[v_counter][c++]=PORTA;
       }
       v_counter++; 
      }//��¼ʵ�ʼ�¼����
      PIFJ_PIFJ7=1;
    }
    PIFJ_PIFJ7=1;
  }
}

//***********************  OC5�ж�,��������źŵ������� ************************

void interrupt 13 TC5_INT(void){
//��MC9S12XS128.h�л�ȡ��Ӧ�жϺţ�#define VectorNumber_Vtimch5            13U
    leftpulse++;
    TFLG1=0x20;//���жϱ�־λ
}

#pragma CODE_SEG DEFAULT

//*********************************** ��ʱ���� ************************




/* CPU delay 200ns at 80M bus clock */
void Cpu_Delay200ns() 
    { __asm(nop); __asm(nop); __asm(nop); __asm(nop);
      __asm(nop); __asm(nop); __asm(nop); __asm(nop);
      __asm(nop); __asm(nop); __asm(nop); __asm(nop);
      __asm(nop); __asm(nop); __asm(nop); __asm(nop);
    }

void SamplingDelay()
    { 
        Cpu_Delay200ns();
    }

