#include <IfxCpu.h>
#include <LQ_ADC.h>
#include <LQ_CCU6.h>
#include <LQ_STM.h>
#include <Main.h>
#include <Platform_Types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_MotorServo.h>
#include <LQ_GPIO_LED.h>
#include <LQ_Inductor.h>
#include <LQ_GPT12_ENC.h>
#include <LQ_MotorServo.h>

void InductorInit (void)
{
    ADC_InitConfig(ADC0, 80000);
    ADC_InitConfig(ADC1, 80000);
//    ADC_InitConfig(ADC2, 80000);
//    ADC_InitConfig(ADC3, 80000);
    ADC_InitConfig(ADC4, 80000);
    ADC_InitConfig(ADC5, 80000);
//    ADC_InitConfig(ADC6, 80000);
//    ADC_InitConfig(ADC7, 80000);
}

sint32 TempAngle = 0;
sint32 TempMotor = 0;
int Circle_Sign=0;

extern uint32 LastAngle;
extern short motorduty;
extern sint32 MagnetValue_Init;
extern sint32 MagnetInfo[MagnetInfo_len];
extern sint32 RelaValue[6];
extern sint32 ADCValue[6];
extern sint32 ADCValue_Init[6];


void InductorCtrl (void)
{
    if(MagnetInfo[0]<beta1*MagnetValue_Init)//当前磁场强度若较弱则进行倒车
    {
        TempAngle=Servo_Center_Mid-(LastAngle-Servo_Center_Mid);//对称打角
        TempMotor=-motorduty;//直流电机反向
        while(MagnetInfo[0]<beta2*MagnetValue_Init)//执行倒车直到判定出弱磁场
        {
            MotorCtrl(TempMotor);
            ServoCtrl(TempAngle);
        }
    }
    else if(RelaValue[1]<25||RelaValue[4]<25)//若有方向磁场较弱
    {
        if(RelaValue[0]+RelaValue[1]>RelaValue[4]+RelaValue[5])//向磁场较强方向打角，用于直角弯
        {
            LastAngle=Servo_Left_Min;
        }
        else
        {
            LastAngle=Servo_Right_Max;
        }
        TempMotor=0.8*motorduty;
    }
    else if (RelaValue[1]<35||RelaValue[4]<35)//某一方向磁场强度较弱，微调角度
    {
        LastAngle = Servo_Center_Mid - 8*(RelaValue[1] - RelaValue[4]);
        TempMotor = 0.9*motorduty;
    }
    else//直行时调整车辆姿态
    {
        LastAngle = Servo_Center_Mid - 3*(RelaValue[1] - RelaValue[4]);
        TempMotor = motorduty;
    }
}
sint32 V1=0;
sint32 V2=0;
int Enter_Circle=0;
void CircleDetect (void)
{
        if(Circle_Sign==1&&Enter_Circle==0)//若进入强磁场且入圈允许标志为0则执行入圈
        {
            ServoCtrl(Servo_Center_Mid-(LastAngle-Servo_Center_Mid));
            TempMotor=0.8*motorduty;//减速
            MotorCtrl(TempMotor);

            int i=0;
            while(1)//检测磁场开始衰减的地方，也就是磁场强度最大处
            {
                delayms(1);
                if(MagnetInfo[0]<MagnetInfo[1])
                {
                    i=i+2;
                }
                else i--;
                if(i>10) break;
            }

            V1=100*ADC_Read(ADC1)/ADCValue_Init[1];//计算从进入强磁场区域到磁场最大处两边电感变化倍数
            V2=100*ADC_Read(ADC4)/ADCValue_Init[4];
            if(V2>V1)
            {
                 LastAngle=Servo_Center_Mid+Servo_Delta/2+90;//右打角
            }
            else
            {
                 LastAngle=Servo_Center_Mid-Servo_Delta/2-50;//左打角
            }

            Enter_Circle=1;//防止再次进入环岛
            ServoCtrl(LastAngle);
            while(MagnetInfo[0]>1.05*MagnetValue_Init);//出强磁场区域打角结束
            Circle_Sign=0;
        }
        else if(Circle_Sign==1&&Enter_Circle==1)//出磁场时
        {
            TempMotor=0.8*motorduty;//减速
            MotorCtrl(TempMotor);
            int i=0;
            while(1)
            {
                delayms(1);
                if(MagnetInfo[0]<MagnetInfo[1])
                {
                    i=i+2;
                }
                else i--;
                if(i>14) break;//检测磁场最大处
            }
            ServoCtrl(Servo_Center_Mid);//直行直到出强磁场区域
            while(MagnetInfo[0]>1.05*MagnetValue_Init);
            Circle_Sign=0;
        }
}
