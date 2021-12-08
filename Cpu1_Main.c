#include <include.h>//引入头文件
#include <IfxCpu.h>
#include <IfxScuCcu.h>
#include <IfxScuWdt.h>
#include <IfxStm.h>
#include <IfxStm_reg.h>
#include <LQ_CCU6.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_GPIO_LED.h>
#include <LQ_MotorServo.h>
#include <LQ_SOFTI2C.h>
#include <LQ_UART.h>
#include <LQ_Inductor.h>
#include <Main.h>
#include <Platform_Types.h>

volatile uint8 cpu1Flage5ms = 0;
volatile uint8 cpu1Flage50ms = 0;

#define MagnetInfo_len 10//定义磁场强度数组（用于检测磁场强度变化）
sint32 MagnetInfo[MagnetInfo_len];
sint32 MagnetValue;//定义磁场强度当前值

#define NOfData 10//思路电感强度数组（用于均值滤波）
sint32 ADC_Data0[NOfData];//左一电感
sint32 ADC_Data1[NOfData];//左二电感
sint32 ADC_Data4[NOfData];//右二电感
sint32 ADC_Data5[NOfData];//右一电感

void Fresh_hist(sint32 hist[],sint32 value)//该函数将新测到数据放入数组最前端，也就是堆存储方式
{
    for(int i=MagnetInfo_len-1;i>0;i--)
    {
        hist[i]=hist[i-1];
    }
    hist[0]=value;
}

sint32 Get_Mean(sint32 ADC_Data[])//获取到数组的均值，也就是均值滤波，防止舵机抽搐
{
    sint32 sum=0;
    for(int i=0;i<NOfData;i++){
        sum+=ADC_Data[i]/NOfData;
    }
    return sum;
}

void ADC_DataInit(sint32 ADC_Data[],ADC_Channel_t t)//初始化ADC数组内容
{
    for(int i=0;i<NOfData;i++){
        ADC_Data[i]=ADC_Read(t);
    }

}

void MagnetCollect(void)//收集和刷新磁场强度数组
{
    MagnetValue=ADC_Read(ADC0)+ADC_Read(ADC1)+ADC_Read(ADC4)+ADC_Read(ADC5);
    Fresh_hist(MagnetInfo,MagnetValue);

}

void MagnetInfo_Init(void)//初始化磁场强度数组，防止其初始值随机化
{
    for(int i=0;i<MagnetInfo_len;i++)
    {
        MagnetCollect();
    }

}

int Counter=0;
sint32 ADCValue[6];
sint32 ADCValue_Init[6];
sint32 RelaValue[6];
sint32 AD_Max[6] = {100,100 , 100, 100, 100, 100};
sint32 AD_Min[6] = {20, 20, 20, 20, 20, 20};

void InfoCollect()//数据收集汇总函数
{
    MagnetCollect();
    ADC_Data0[Counter]=ADC_Read(ADC0);
    ADC_Data1[Counter]=ADC_Read(ADC1);
    ADC_Data4[Counter]=ADC_Read(ADC4);
    ADC_Data5[Counter]=ADC_Read(ADC5);
    Counter++;
    if(Counter==NOfData) Counter=0;
    ADCValue[0]=Get_Mean(ADC_Data0);//均值滤波后数据放入ADCValue数组中
    ADCValue[1]=Get_Mean(ADC_Data1);
    ADCValue[4]=Get_Mean(ADC_Data4);
    ADCValue[5]=Get_Mean(ADC_Data5);

    if (ADCValue[0] < AD_Min[0])//获取滤波后电感值的最大值和最小值
        AD_Min[0] = ADCValue[0];
    else if (ADCValue[0] > AD_Max[0])
        AD_Max[0] = ADCValue[0];
    if (ADCValue[1] < AD_Min[1])
        AD_Min[1] = ADCValue[1];
    else if (ADCValue[1] > AD_Max[1])
        AD_Max[1] = ADCValue[1];
    if (ADCValue[4] < AD_Min[4])
        AD_Min[4] = ADCValue[4];
    else if (ADCValue[4] > AD_Max[4])
        AD_Max[4] = ADCValue[4];
    if (ADCValue[5] < AD_Min[5])
        AD_Min[5] = ADCValue[5];
    else if (ADCValue[5] > AD_Max[5])
        AD_Max[5] = ADCValue[5];
    //计算归一化值（方便分段比例控制）
    RelaValue[0] = (ADCValue[0] - AD_Min[0]) * 100 / (AD_Max[0] - AD_Min[0]);
    RelaValue[1] = (ADCValue[1] - AD_Min[1]) * 100 / (AD_Max[1] - AD_Min[1]);
    RelaValue[4] = (ADCValue[4] - AD_Min[4]) * 100 / (AD_Max[4] - AD_Min[4]);
    RelaValue[5] = (ADCValue[5] - AD_Min[5]) * 100 / (AD_Max[5] - AD_Min[5]);

}

extern int Circle_Sign;
extern int Enter_Circle;
sint32 MagnetValue_Init;
extern uint32 LastAngle;
extern sint32 V1;
extern sint32 V2;

void Oled_Show(void)//OLED显示函数
{
    char txt[16] = "X:";
    sprintf(txt, "L1: %3d    %4d", (int)RelaValue[0],(int)ADCValue[0]);
    OLED_P6x8Str(0,0,txt);
    sprintf(txt, "L2: %3d    %4d", (int)RelaValue[1],(int)ADCValue[1]);
    OLED_P6x8Str(0,1,txt);
    sprintf(txt, "R2: %3d    %4d", (int)RelaValue[4],(int)ADCValue[4]);
    OLED_P6x8Str(0,2,txt);
    sprintf(txt, "R1: %3d    %4d", (int)RelaValue[5],(int)ADCValue[5]);
    OLED_P6x8Str(0,3,txt);
    sprintf(txt, "M: %4d   %4d", (short)MagnetInfo[0],(int)MagnetValue_Init);
    OLED_P6x8Str(0,4,txt);
    sprintf(txt, "Circle_Sign: %1d  %1d", Circle_Sign,Enter_Circle);
    OLED_P6x8Str(0,5,txt);
    sprintf(txt, "Ang:%4d Sp:%4d",(int)LastAngle,(int)ECPULSE1);
    OLED_P6x8Str(0,6,txt);
    sprintf(txt, "%4d   %4d", (int)(V1),(int)(V2));
    OLED_P6x8Str(0,7,txt);

}

int core1_main (void)//CPU1主函数入口
{
    IfxCpu_enableInterrupts();

    IfxScuWdt_disableCpuWatchdog (IfxScuWdt_getCpuWatchdogPassword());

    while(!IfxCpu_acquireMutex(&mutexCpu0InitIsOk));

    mutexCpu0TFTIsOk=0;

	CCU6_InitConfig(CCU61, CCU6_Channel0, 50000);


	ADC_DataInit(ADC_Data0,ADC0);//初始化
	ADC_DataInit(ADC_Data1,ADC1);
	ADC_DataInit(ADC_Data4,ADC4);
	ADC_DataInit(ADC_Data5,ADC5);
	ADCValue_Init[0]=Get_Mean(ADC_Data0);
	ADCValue_Init[1]=Get_Mean(ADC_Data1);
	ADCValue_Init[4]=Get_Mean(ADC_Data4);
	ADCValue_Init[5]=Get_Mean(ADC_Data5);
	MagnetInfo_Init();
	MagnetValue_Init=Get_Mean(MagnetInfo);

	sint32 Dist=0;
    while(1)
    {
        Oled_Show();
        InfoCollect();
        if((MagnetInfo[0]>beta3*MagnetValue_Init))//此为检测小车是否进入强磁场区域
        {
            if(Circle_Sign==0)
            {
                Circle_Sign=1;
                Dist=0;
            }
        }
        Dist+=ECPULSE1;
        if(Dist>1500000) Enter_Circle=0;//若小车走过一定路程，则使再次进环岛标志位清零
    }
}
