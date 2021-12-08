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

App_Cpu0 g_AppCpu0;
IfxCpu_mutexLock mutexCpu0InitIsOk = 1;
volatile char mutexCpu0TFTIsOk=0;

short motorduty=2500;
uint32 LastAngle=Servo_Center_Mid;
extern sint32 TempMotor;
int core0_main (void)
{
	IfxCpu_disableInterrupts();

	IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
	IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

	g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
	g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
	g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
	g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);

	OLED_Init();//��ʼ��OLED����
	GPIO_KEY_Init();//��ʼ�������ж�
	GPIO_LED_Init();//��ʼ��LED������

	UART_InitConfig(UART0_RX_P14_1,UART0_TX_P14_0, 9600);

	IfxCpu_enableInterrupts();

	IfxCpu_releaseMutex(&mutexCpu0InitIsOk);

	mutexCpu0TFTIsOk=0;

    InductorInit(); //��ʼ����Ŵ�����ADC����
    MotorInit();//��ʼ���������PWM�������
    ServoInit();//��ʼ�������������
    EncInit();//��ʼ��������

    while(1){
        if (KEY_Read(KEY0) == 0) motorduty=2200;//����ģʽ
        if (KEY_Read(KEY2) == 0) motorduty=2800;//����ģʽ
        if (KEY_Read(KEY1) == 0) motorduty=2500;//һ���ٶ�
        CircleDetect();//��Ȧ���ƺ���
        InductorCtrl();//�����ǿ���
        MotorCtrl(TempMotor);
        ServoCtrl(LastAngle);
    }
}
