#include <include.h>//����ͷ�ļ�
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

#define MagnetInfo_len 10//����ų�ǿ�����飨���ڼ��ų�ǿ�ȱ仯��
sint32 MagnetInfo[MagnetInfo_len];
sint32 MagnetValue;//����ų�ǿ�ȵ�ǰֵ

#define NOfData 10//˼·���ǿ�����飨���ھ�ֵ�˲���
sint32 ADC_Data0[NOfData];//��һ���
sint32 ADC_Data1[NOfData];//������
sint32 ADC_Data4[NOfData];//�Ҷ����
sint32 ADC_Data5[NOfData];//��һ���

void Fresh_hist(sint32 hist[],sint32 value)//�ú������²⵽���ݷ���������ǰ�ˣ�Ҳ���ǶѴ洢��ʽ
{
    for(int i=MagnetInfo_len-1;i>0;i--)
    {
        hist[i]=hist[i-1];
    }
    hist[0]=value;
}

sint32 Get_Mean(sint32 ADC_Data[])//��ȡ������ľ�ֵ��Ҳ���Ǿ�ֵ�˲�����ֹ����鴤
{
    sint32 sum=0;
    for(int i=0;i<NOfData;i++){
        sum+=ADC_Data[i]/NOfData;
    }
    return sum;
}

void ADC_DataInit(sint32 ADC_Data[],ADC_Channel_t t)//��ʼ��ADC��������
{
    for(int i=0;i<NOfData;i++){
        ADC_Data[i]=ADC_Read(t);
    }

}

void MagnetCollect(void)//�ռ���ˢ�´ų�ǿ������
{
    MagnetValue=ADC_Read(ADC0)+ADC_Read(ADC1)+ADC_Read(ADC4)+ADC_Read(ADC5);
    Fresh_hist(MagnetInfo,MagnetValue);

}

void MagnetInfo_Init(void)//��ʼ���ų�ǿ�����飬��ֹ���ʼֵ�����
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

void InfoCollect()//�����ռ����ܺ���
{
    MagnetCollect();
    ADC_Data0[Counter]=ADC_Read(ADC0);
    ADC_Data1[Counter]=ADC_Read(ADC1);
    ADC_Data4[Counter]=ADC_Read(ADC4);
    ADC_Data5[Counter]=ADC_Read(ADC5);
    Counter++;
    if(Counter==NOfData) Counter=0;
    ADCValue[0]=Get_Mean(ADC_Data0);//��ֵ�˲������ݷ���ADCValue������
    ADCValue[1]=Get_Mean(ADC_Data1);
    ADCValue[4]=Get_Mean(ADC_Data4);
    ADCValue[5]=Get_Mean(ADC_Data5);

    if (ADCValue[0] < AD_Min[0])//��ȡ�˲�����ֵ�����ֵ����Сֵ
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
    //�����һ��ֵ������ֶα������ƣ�
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

void Oled_Show(void)//OLED��ʾ����
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

int core1_main (void)//CPU1���������
{
    IfxCpu_enableInterrupts();

    IfxScuWdt_disableCpuWatchdog (IfxScuWdt_getCpuWatchdogPassword());

    while(!IfxCpu_acquireMutex(&mutexCpu0InitIsOk));

    mutexCpu0TFTIsOk=0;

	CCU6_InitConfig(CCU61, CCU6_Channel0, 50000);


	ADC_DataInit(ADC_Data0,ADC0);//��ʼ��
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
        if((MagnetInfo[0]>beta3*MagnetValue_Init))//��Ϊ���С���Ƿ����ǿ�ų�����
        {
            if(Circle_Sign==0)
            {
                Circle_Sign=1;
                Dist=0;
            }
        }
        Dist+=ECPULSE1;
        if(Dist>1500000) Enter_Circle=0;//��С���߹�һ��·�̣���ʹ�ٴν�������־λ����
    }
}
