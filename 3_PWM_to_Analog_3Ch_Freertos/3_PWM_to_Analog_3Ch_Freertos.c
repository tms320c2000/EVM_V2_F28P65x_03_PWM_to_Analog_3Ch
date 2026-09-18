// 파일이름	:	3_PWM_to_Analog_3Ch_Freertos.c
// 대상장치	:	TMS320F28X EVM V2, TMS320F28P65x(F28P650DK9 산업용 / F28P659DK8-Q1 차량용) 모듈
// 파일버전	:	1.00
// 갱신이력	:	2026-09-17, 버전 1.00
// 예제설명	:

//************************************************************************************************************************************************************************
//
// 본 예제는 TMS320F28P65x 모듈이 탑재된 TMS320F28X 개발보드(EVM) V2를 대상으로 하고 있으며,
// TMS320F28P65x 칩의 ePWM 회로를 통해 개발보드의 PWM-to-Analog 출력 회로를 구동합니다.
//
// TMS320F28X 개발보드 V2의 '(3) PWM-to-Analog 출력 3채널' 회로는 ePWM 신호를 2차 저역통과필터에
// 통과시켜서 듀티(Duty)에 비례하는 아날로그 전압(0.0V~3.3V)으로 바꿔주는 회로입니다.
//
// 본 예제는 FreeRTOS 커널 위에서 두 개의 정적(Static, 힙 미사용) 태스크가 협력하여 동작합니다:
//   1. PWM_Task (우선순위 1, 5ms 주기): 240포인트 사인 룩업 테이블을 120도 위상차로 전진시켜
//      3상 사인파(SPWM) 듀티를 갱신합니다.
//   2. ADC_Task (우선순위 2, 2ms 주기 = 500Hz): 필터 출력 전압을 ADCINA2/A3/A4로 되읽어
//      채널당 1024워드 크기의 원형 버퍼(RAMGS0/1/2)에 실시간으로 저장합니다.
//
// a. 배선 안내 (B Side 핀-헤더와 개발보드 PWM-to-Analog 4채널 회로 연결):
//
//		>> 1채널(U상): GPIO 213번(EPWM8A)  (B Side 91번 핀) ──▶ PWM.IN1
//		>> 2채널(V상): GPIO 211번(EPWM14A) (B Side 95번 핀) ──▶ PWM.IN2
//		>> 3채널(W상): GPIO 209번(EPWM12A) (B Side 99번 핀) ──▶ PWM.IN3
//
//		>> 1채널 피드백: A.OUT1 ──▶ B Side 107번 핀 (ADCINA2 / AIO229)
//		>> 2채널 피드백: A.OUT2 ──▶ B Side 105번 핀 (ADCINA3 / AIO230)
//		>> 3채널 피드백: A.OUT3 ──▶ B Side 103번 핀 (ADCINA4 / AIO231)
//
//	 (Bitfield 예제, DriverLib 예제와 완전히 동일한 점퍼 배선으로 공용할 수 있습니다.)
//
// 예제 폴더의 CCS Expressions 창에 아래 변수를 등록해두면 동작을 실시간으로 확인할 수 있습니다.
// --> dutyCh1, dutyCh2, dutyCh3, rampCount, adcCh1Result, adcCh2Result, adcCh3Result
//
// 본 예제는 TI DriverLib API + FreeRTOS 정적 할당(configSUPPORT_DYNAMIC_ALLOCATION=0)만으로
// 제작되었습니다. 같은 동작을 비트필드로 구현한 3_PWM_to_Analog_3Ch_Bitfield.c,
// 베어메탈 DriverLib으로 구현한 3_PWM_to_Analog_3Ch_Driverlib.c와 함께 세 가지 방식을
// 비교해 보실 수 있습니다.
//
//************************************************************************************************************************************************************************


// 헤더 파일들
#include "driverlib.h"		// TI 제공 Driver API Library 헤더파일 (driverlib)
#include "device.h"
#include "FreeRTOS.h"
#include "task.h"

// 전처리 구문 정의
#define	PWM_TBPRD	2000U	// TBCLK=SYSCLK(200MHz) 기준 PWM 주파수 100kHz
#define	ADC_ACQPS	39U		// SOC 샘플/홀드 창(윈도우) 크기
#define	STACK_SIZE	256U	// 태스크 스택 크기, 워드 단위 (C28x는 1워드=16비트이므로 512바이트)

#define	ADC_BUFFER_LEN	1024U	// 채널당 ADC 샘플 원형 버퍼 길이(워드)

// 3상 사인파(SPWM) 테이블 파라미터
#define SINE_TABLE_SIZE			240U	// 240포인트 사인 룩업 테이블 (5ms 갱신 시 1주기 = 1.2초, 0.833Hz)
#define SINE_PHASE_STEP_120		80U		// 240 / 3 = 80스텝 (120도 위상차, V상)
#define SINE_PHASE_STEP_240		160U	// 240 * 2 / 3 = 160스텝 (240도 위상차, W상)

// 240포인트 사인 룩업 테이블 (듀티: 최소 100(5%), 중심 1000(50%), 최대 1900(95%))
static const uint16_t sineTable[SINE_TABLE_SIZE] = {
    1000, 1024, 1047, 1071, 1094, 1117, 1141, 1164, 1187, 1210,
    1233, 1256, 1278, 1300, 1323, 1344, 1366, 1387, 1409, 1429,
    1450, 1470, 1490, 1510, 1529, 1548, 1566, 1585, 1602, 1620,
    1636, 1653, 1669, 1684, 1699, 1714, 1728, 1742, 1755, 1767,
    1779, 1791, 1802, 1812, 1822, 1831, 1840, 1848, 1856, 1863,
    1869, 1875, 1880, 1885, 1889, 1892, 1895, 1897, 1899, 1900,
    1900, 1900, 1899, 1897, 1895, 1892, 1889, 1885, 1880, 1875,
    1869, 1863, 1856, 1848, 1840, 1831, 1822, 1812, 1802, 1791,
    1779, 1767, 1755, 1742, 1728, 1714, 1699, 1684, 1669, 1653,
    1636, 1620, 1602, 1585, 1566, 1548, 1529, 1510, 1490, 1470,
    1450, 1429, 1409, 1387, 1366, 1344, 1323, 1300, 1278, 1256,
    1233, 1210, 1187, 1164, 1141, 1117, 1094, 1071, 1047, 1024,
    1000,  976,  953,  929,  906,  883,  859,  836,  813,  790,
     767,  744,  722,  700,  677,  656,  634,  613,  591,  571,
     550,  530,  510,  490,  471,  452,  434,  415,  398,  380,
     364,  347,  331,  316,  301,  286,  272,  258,  245,  233,
     221,  209,  198,  188,  178,  169,  160,  152,  144,  137,
     131,  125,  120,  115,  111,  108,  105,  103,  101,  100,
     100,  100,  101,  103,  105,  108,  111,  115,  120,  125,
     131,  137,  144,  152,  160,  169,  178,  188,  198,  209,
     221,  233,  245,  258,  272,  286,  301,  316,  331,  347,
     364,  380,  398,  415,  434,  452,  471,  490,  510,  530,
     550,  571,  591,  613,  634,  656,  677,  700,  722,  744,
     767,  790,  813,  836,  859,  883,  906,  929,  953,  976
};

// 함수 원형 선언
void initPwmGpio(void);
void initEPWMChannel(uint32_t base);
void initAdcFeedback(void);
void PWM_Task(void *pvParameters);
void ADC_Task(void *pvParameters);


// 전역 변수 선언
volatile uint16_t	dutyCh1;						// 1채널(U상, 0도) 현재 듀티(100~1900)
volatile uint16_t	dutyCh2;						// 2채널(V상, +120도) 현재 듀티
volatile uint16_t	dutyCh3;						// 3채널(W상, +240도) 현재 듀티
volatile uint32_t	rampCount = 0U;					// 듀티 갱신 루프 실행 횟수 계수용 S/W 카운터
static uint16_t		sineIndex = 0U;					// 사인 룩업 테이블 현재 인덱스(0~239)

volatile uint16_t	adcCh1Result = 0U;				// ADCINA2로 되읽은 1채널 아날로그 출력 최신 샘플(0~4095)
volatile uint16_t	adcCh2Result = 0U;				// ADCINA3로 되읽은 2채널 아날로그 출력 최신 샘플
volatile uint16_t	adcCh3Result = 0U;				// ADCINA4로 되읽은 3채널 아날로그 출력 최신 샘플

// 1024워드 원형 버퍼는 .bss(2KB) 오버플로우를 방지하기 위해 RAMGS0/1/2(각 8KB)에 분산 배치합니다.
#pragma DATA_SECTION(adcCh1Buffer, "ramgs0")
volatile uint16_t	adcCh1Buffer[ADC_BUFFER_LEN];	// 1채널 ADC 샘플 원형 버퍼
#pragma DATA_SECTION(adcCh2Buffer, "ramgs1")
volatile uint16_t	adcCh2Buffer[ADC_BUFFER_LEN];	// 2채널 ADC 샘플 원형 버퍼
#pragma DATA_SECTION(adcCh3Buffer, "ramgs2")
volatile uint16_t	adcCh3Buffer[ADC_BUFFER_LEN];	// 3채널 ADC 샘플 원형 버퍼
volatile uint32_t	adcBufferIndex = 0UL;			// 다음에 쓸 원형 버퍼 인덱스

// FreeRTOS 태스크용 정적 메모리 선언 (힙 미사용, 정적 할당)
static StaticTask_t pwmTaskBuffer;
static StackType_t  pwmTaskStack[STACK_SIZE];
#pragma DATA_SECTION(pwmTaskStack, ".freertosStaticStack")
#pragma DATA_ALIGN(pwmTaskStack, portBYTE_ALIGNMENT)

static StaticTask_t adcTaskBuffer;
static StackType_t  adcTaskStack[STACK_SIZE];
#pragma DATA_SECTION(adcTaskStack, ".freertosStaticStack")
#pragma DATA_ALIGN(adcTaskStack, portBYTE_ALIGNMENT)

static StaticTask_t idleTaskBuffer;
static StackType_t  idleTaskStack[STACK_SIZE];
#pragma DATA_SECTION(idleTaskStack, ".freertosStaticStack")
#pragma DATA_ALIGN(idleTaskStack, portBYTE_ALIGNMENT)


// 메인 함수
void main(void)
{

//	1. 전역 인터럽트 스위치 OFF, CPU 인터럽트 벡터 비-활성화 및 플래그(Flag) 비트 클리어
//	   (Interrupt_initModule() 함수가 한 번에 담당합니다)


//	2. 시스템 초기화 - Device_init( ) 함수 호출
//	* 왓치독 타이머 비-활성화
//	* CPU 클럭 주파수 설정 (PLL, 200MHz)
//	* 주변회로 클럭 공급 설정
	Device_init();

//	* 범용 입출력 포트(GPIO) 핀 락 해제, 내부 풀업 활성화 - Device_initGPIO( ) 함수 호출
	Device_initGPIO();


//	3. 주변회로 인터럽트 확장회로 초기화 - Interrupt_initModule( ) 함수 호출
	Interrupt_initModule();


//	4. 주변회로 인터럽트 벡터 확장 및 복사 실행 - Interrupt_initVectorTable( ) 함수 호출
	Interrupt_initVectorTable();


//	5. 인터럽트 벡터와 인터럽트 서비스 루틴 재-연결


//	6. 주변회로 초기화 - PWM-to-Analog 3채널용 GPIO 핀 설정 및 ePWM 3채널 설정
	initPwmGpio();

	SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

	dutyCh1 = sineTable[0];
	dutyCh2 = sineTable[SINE_PHASE_STEP_120];
	dutyCh3 = sineTable[SINE_PHASE_STEP_240];

	initEPWMChannel(EPWM8_BASE);
	initEPWMChannel(EPWM14_BASE);
	initEPWMChannel(EPWM12_BASE);

	EPWM_setCounterCompareValue(EPWM8_BASE, EPWM_COUNTER_COMPARE_A, dutyCh1);
	EPWM_setCounterCompareValue(EPWM14_BASE, EPWM_COUNTER_COMPARE_A, dutyCh2);
	EPWM_setCounterCompareValue(EPWM12_BASE, EPWM_COUNTER_COMPARE_A, dutyCh3);

	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

	// ADC-A 3채널(ADCINA2/A3/A4) 초기화 - PWM-to-Analog 출력 되먹임 확인용
	initAdcFeedback();


//	7. 전역 변수 및 S/W 모듈 초기화
	rampCount = 0U;
	sineIndex = 0U;


//	8. 실시간 디버깅 활성화, 전역 인터럽트 스위치 ON
	ERTM;	// Debug Enable Mask 비트 설정
	EINT;	// 전역 인터럽트 스위치 ON (/INTM ON)


//	9. FreeRTOS 태스크 생성 및 스케줄러 시작
//	   - PWM_Task (우선순위 1, 5ms 주기): 3상 사인파 듀티 갱신
//	   - ADC_Task (우선순위 2, 2ms 주기): 500Hz ADC 변환 및 원형 버퍼 저장
	xTaskCreateStatic(PWM_Task,				// 태스크 함수
	                  "PWM Task",			// 이름(디버깅용)
	                  STACK_SIZE,			// 스택 크기(워드)
	                  NULL,					// 파라미터 없음
	                  tskIDLE_PRIORITY + 1,	// 우선순위 1
	                  pwmTaskStack,
	                  &pwmTaskBuffer);

	xTaskCreateStatic(ADC_Task,				// 태스크 함수
	                  "ADC Task",			// 이름(디버깅용)
	                  STACK_SIZE,			// 스택 크기(워드)
	                  NULL,					// 파라미터 없음
	                  tskIDLE_PRIORITY + 2,	// 우선순위 2 (지터 최소화)
	                  adcTaskStack,
	                  &adcTaskBuffer);

	vTaskStartScheduler();		// 이 아래로는 절대 돌아오지 않음

	for(;;)
	{
	    // 여기 도달하면 스케줄러 시작 실패 (메모리 부족 등)
	}
}

//	10. 인터럽트 서비스 루틴 및 기타 함수들

//
// PWM_Task - 5ms마다 240포인트 3상 사인파 듀티 갱신 (1주기 = 1.200초, 0.833Hz)
//
void PWM_Task(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        dutyCh1 = sineTable[sineIndex];
        dutyCh2 = sineTable[(sineIndex + SINE_PHASE_STEP_120) % SINE_TABLE_SIZE];
        dutyCh3 = sineTable[(sineIndex + SINE_PHASE_STEP_240) % SINE_TABLE_SIZE];

        EPWM_setCounterCompareValue(EPWM8_BASE, EPWM_COUNTER_COMPARE_A, dutyCh1);
        EPWM_setCounterCompareValue(EPWM14_BASE, EPWM_COUNTER_COMPARE_A, dutyCh2);
        EPWM_setCounterCompareValue(EPWM12_BASE, EPWM_COUNTER_COMPARE_A, dutyCh3);

        sineIndex++;
        if(sineIndex >= SINE_TABLE_SIZE)
        {
            sineIndex = 0U;
        }

        rampCount++;

        // vTaskDelayUntil을 사용하여 누적 오차 없이 정확히 5ms(200Hz) 간격으로 실행
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

//
// ADC_Task - 2ms마다(500Hz) ADCINA2/A3/A4 변환을 트리거하여 1024워드 원형 버퍼에 저장
//
void ADC_Task(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
        ADC_forceMultipleSOC(ADCA_BASE, (ADC_FORCE_SOC0 | ADC_FORCE_SOC1 | ADC_FORCE_SOC2));

        uint16_t timeout = 0xFFFF;
        while((ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false) && (timeout > 0))
        {
            timeout--;
        }

        adcCh1Result = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
        adcCh2Result = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
        adcCh3Result = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2);

        adcCh1Buffer[adcBufferIndex] = adcCh1Result;
        adcCh2Buffer[adcBufferIndex] = adcCh2Result;
        adcCh3Buffer[adcBufferIndex] = adcCh3Result;

        adcBufferIndex++;
        if(adcBufferIndex >= ADC_BUFFER_LEN)
        {
            adcBufferIndex = 0UL;
        }

        // vTaskDelayUntil을 사용하여 지터 없이 정밀한 2ms(500Hz) 샘플 레이트 유지
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
    }
}

//
// initPwmGpio - GPIO213/211/209를 각각 EPWM8A/EPWM14A/EPWM12A 기능으로 전환 (B Side 핀-헤더)
//
void initPwmGpio(void)
{
    //
    // F28P65x의 GPIO213(A10), GPIO211(A8), GPIO209(A6)는 아날로그 겸용 핀(AIO)입니다.
    // 칩 리셋 시 GPGAMSEL 비트가 1(아날로그 모드)로 초기화되어 디지털 입출력 버퍼가 분리됩니다.
    // ePWM 디지털 구동 신호가 물리 핀(B-Side 91, 95, 99번)으로 정상 출력되려면
    // 반드시 아날로그 모드를 해제(디지털 전환: GPGAMSEL=0, AGPIOCTRLG=0)해야 합니다.
    //
    GPIO_setAnalogMode(213U, GPIO_ANALOG_DISABLED);
    GPIO_setPadConfig(213U, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_213_EPWM8_A);

    GPIO_setAnalogMode(211U, GPIO_ANALOG_DISABLED);
    GPIO_setPadConfig(211U, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_211_EPWM14_A);

    GPIO_setAnalogMode(209U, GPIO_ANALOG_DISABLED);
    GPIO_setPadConfig(209U, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_209_EPWM12_A);
}

//
// initEPWMChannel - TBPRD=2000, 업카운트, ePWMxA에서 0%~100% 듀티 출력 가능한
// 기본 Action Qualifier 구성 (TBCTR=0에서 High, TBCTR=CMPA에서 Low)
//
void initEPWMChannel(uint32_t base)
{
    EPWM_setEmulationMode(base, EPWM_EMULATION_FREE_RUN);
    EPWM_setPeriodLoadMode(base, EPWM_PERIOD_SHADOW_LOAD);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);
    EPWM_setTimeBasePeriod(base, PWM_TBPRD);
    EPWM_setTimeBaseCounter(base, 0U);

    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH,
                                   EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW,
                                   EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
}

//
// initAdcFeedback - ADC-A를 12bit 단일종단 모드로 초기화하고 ADCINA2/A3/A4를 설정합니다.
//
void initAdcFeedback(void)
{
    // AIO229(A2, B Side 107번), AIO230(A3, B Side 105번), AIO231(A4, B Side 103번) 아날로그 모드 활성화
    GPIO_setAnalogMode(229U, GPIO_ANALOG_ENABLED);
    GPIO_setAnalogMode(230U, GPIO_ANALOG_ENABLED);
    GPIO_setAnalogMode(231U, GPIO_ANALOG_ENABLED);

    // 내부 3.3V 기준전압 활성화 (Bitfield의 ANAREFASEL=0, ANAREFA2P5SEL=0과 동일)
    ASysCtl_setAnalogReferenceInternal(ASYSCTL_VREFHIA);
    ASysCtl_setAnalogReference1P65(ASYSCTL_VREFHIA);	// 3.3V / 1.65V 모드 (bit8=0)

    // ADC-A 클록 분주(50MHz, 200MHz SYSCLK 기준 /4) 및 12bit 단일종단 모드 설정
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(5000U);	// ADC 코어 및 내부 VREF 안정화 대기시간 (5ms)

    // SOC 채널 배정 (소프트웨어 강제 트리거 모드)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN2, ADC_ACQPS);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN3, ADC_ACQPS);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN4, ADC_ACQPS);

    // SOC2(마지막 채널) 변환 완료 시 ADCINT1 플래그 발생 (단발성 모드)
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2);
    ADC_disableContinuousMode(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}

//
// vApplicationStackOverflowHook - FreeRTOS가 스택오버플로우를 감지하면 호출
//
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    for(;;) { }
}

//
// vApplicationGetIdleTaskMemory - configSUPPORT_STATIC_ALLOCATION=1 이면 Idle 태스크
// 메모리도 애플리케이션이 직접 제공해야 함(힙을 안 쓰므로)
//
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    configSTACK_DEPTH_TYPE *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &idleTaskBuffer;
    *ppxIdleTaskStackBuffer = idleTaskStack;
    *pulIdleTaskStackSize = STACK_SIZE;
}

// 파일 끝.
