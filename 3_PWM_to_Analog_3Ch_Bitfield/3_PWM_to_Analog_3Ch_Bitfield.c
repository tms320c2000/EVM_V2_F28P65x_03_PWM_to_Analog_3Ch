// 파일이름	:	3_PWM_to_Analog_3Ch_Bitfield.c
// 대상장치	:	TMS320F28X EVM V2, TMS320F28P65x(F28P650DK9 산업용 / F28P659DK8-Q1 차량용) 모듈
// 파일버전	:	1.00
// 갱신이력	:	2026-09-17, 버전 1.00
// 예제설명	:

//************************************************************************************************************************************************************************
//
// 본 예제는 TMS320F28P65x 모듈이 탑재된 TMS320F28X 개발보드(EVM) V2를 대상으로 하고 있으며,
// TMS320F28P65x 칩의 ePWM 회로를 통해 개발보드의 PWM-to-Analog 출력 회로를 구동합니다.
//
// TMS320F28X 개발보드 V2의 '(3) PWM-to-Analog 출력 3채널' 회로는 ePWM 신호를 저역통과필터에
// 통과시켜서 듀티(Duty)에 비례하는 아날로그 전압(0.0V~3.3V)으로 바꿔주는 회로입니다.
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
//	 (DriverLib 예제, FreeRTOS 예제와 완전히 동일한 점퍼 배선으로 공용할 수 있습니다.)
//
// 예제 폴더의 CCS Expressions 창에 아래 변수를 등록해두면 동작을 실시간으로 확인할 수 있습니다.
// --> dutyCh1, dutyCh2, dutyCh3, rampCount, adcCh1Result, adcCh2Result, adcCh3Result
//
// 본 예제는 28X 칩 MMR(Memory Mapped Register)을 직접 조작하는 TI의 Bit-Field Approach
// 만으로 제작되었습니다 (Driverlib 미사용, driverlib.lib 자체가 이 프로젝트에는 없습니다).
// 같은 동작을 DriverLib API로 구현한 3_PWM_to_Analog_3Ch_Driverlib.c, FreeRTOS 태스크로
// 구현한 3_PWM_to_Analog_3Ch_Freertos.c와 함께 세 가지 방식을 비교해 보실 수 있습니다.
//
// b.	ADCINA2/A3/A4 3개 채널로 PWM-to-Analog 출력을 되먹임(feedback) 받아, 각 채널이
//		만든 아날로그 전압을 칩이 직접 되읽어서 확인합니다.
//
//		(위 3개 핀은 F28P65x에서 AIO229/AIO230/AIO231로 멀티플렉싱되어 있으므로,
//		initAdcFeedback()에서 GPHAMSEL과 AGPIOCTRLH를 아날로그 모드로 반드시
//		설정해야 합니다.)
//
//		ADC 샘플링은 1채널 PWM(EPWM8)의 주기 인터럽트(매 10usec, 100kHz)를 기준
//		클럭으로 씁니다. 인터럽트마다 소프트웨어 카운터로 ADC_SAMPLE_DECIMATION번에
//		1번만 실제 ADC 변환을 하므로(2msec에 1번, 500Hz), 메인 루프의 "5msec에 한
//		번 값 읽기" 방식보다 훨씬 촘촘하고 시간이 정확한 샘플을 얻습니다. 매 샘플을
//		adcCh1Buffer/adcCh2Buffer/adcCh3Buffer(각 1024워드) 원형 버퍼에 순서대로
//		채워 넣고, 끝까지 차면 처음(인덱스 0)으로 돌아가 계속 덮어씁니다 - 1024개
//		샘플 * 2msec = 약 2.05초 분량이라 1.2초 주기의 3상 사인파 약 1.7주기 전체가
//		항상 버퍼 안에 들어 있습니다.
//
// 예제 폴더의 CCS **Tools > Graph > Single Time**로 아래처럼 설정하면 3상 사인파 전체
// 모양을 실시간으로 볼 수 있습니다 (세 채널 모두 같은 방식, Start Address만 다름):
//		Acquisition Buffer Size = 1024, DSP Data Type = 16-bit unsigned,
//		Sampling Rate (Hz) = 500, Start Address = adcCh1Buffer (또는 adcCh2Buffer/adcCh3Buffer)
// CCS Expressions 창에 adcCh1Result/adcCh2Result/adcCh3Result(최신 샘플 1개, 0~4095)도
// 함께 등록해두면 그래프 없이도 값 변화를 바로 확인할 수 있습니다.
//
//************************************************************************************************************************************************************************


// 헤더 파일들
#include "f28x_project.h"		// TI 제공 칩-지원 헤더 통합 Include 용 헤더파일 (bit-field)

// 전처리 구문 정의
#define	PWM_TBPRD	2000U	// TBCLK=SYSCLK(200MHz) 기준 PWM 주파수 100kHz
#define	ADC_ACQPS	39U		// SOC 샘플/홀드 창(윈도우) 크기 - 6_Potentiometer_1Ch_ADC 예제와 동일 값

#define	ADC_BUFFER_LEN			1024U	// 채널당 ADC 샘플 원형 버퍼 길이(워드)
#define	ADC_SAMPLE_DECIMATION	200U	// EPWM8 인터럽트(100kHz, 10usec) 200번 중 1번만 실제 ADC 변환
										// -> 유효 샘플레이트 500Hz(2msec) -> 버퍼 전체 = 약 2.05초 분량

// 3상 사인파(SPWM) 테이블 파라미터
#define SINE_TABLE_SIZE			240U	// 240포인트 사인 룩업 테이블 (5ms 갱신 시 1주기 = 1.2초, 0.833Hz)
#define SINE_PHASE_STEP_120		80U		// 240 / 3 = 80스텝 (120도 위상차, V상)
#define SINE_PHASE_STEP_240		160U	// 240 * 2 / 3 = 160스텝 (240도 위상차, W상)

// 240포인트 사인 룩업 테이블 (듀티: 최소 100(5%), 중심 1000(50%), 최대 1900(95%))
static const Uint16 sineTable[SINE_TABLE_SIZE] = {
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
void initEPWMChannel(volatile struct EPWM_REGS *pwm);
void initAdcFeedback(void);
void initAdcSampleInterrupt(void);
interrupt void epwm8_isr(void);


// 전역 변수 선언
volatile Uint16	dutyCh1;						// 1채널(U상, 0도) 현재 듀티(100~1900)
volatile Uint16	dutyCh2;						// 2채널(V상, +120도) 현재 듀티
volatile Uint16	dutyCh3;						// 3채널(W상, +240도) 현재 듀티
volatile Uint32	rampCount;						// 듀티 갱신 루프 실행 횟수 계수용 S/W 카운터
static Uint16	sineIndex = 0U;					// 사인 룩업 테이블 현재 인덱스(0~239)

volatile Uint16	adcCh1Result;					// ADCINA2로 되읽은 1채널 아날로그 출력 최신 샘플(0~4095, 12bit)
volatile Uint16	adcCh2Result;					// ADCINA3로 되읽은 2채널 아날로그 출력 최신 샘플
volatile Uint16	adcCh3Result;					// ADCINA4로 되읽은 3채널 아날로그 출력 최신 샘플

// 1024워드짜리 버퍼 3개(6KB)는 기본 .bss 영역(28p65x_generic_ram_lnk_cpu1.cmd의 RAMLS5,
// 2KB=1024워드 1개도 겨우 들어가는 크기)에 다 안 들어갑니다. 링커 cmd 파일이 SDFM용으로
// 비워둔 RAMGS0/1/2(각 8KB, NOINIT 섹션 "ramgs0"/"ramgs1"/"ramgs2")로 채널별로 하나씩
// 옮겨서 배치합니다 - "program will not fit into available memory ... section .bss" 링크
// 에러의 수정입니다.
#pragma DATA_SECTION(adcCh1Buffer, "ramgs0")
volatile Uint16	adcCh1Buffer[ADC_BUFFER_LEN];	// 1채널 ADC 샘플 원형 버퍼 - CCS Graph에서 이 배열을 가리킴
#pragma DATA_SECTION(adcCh2Buffer, "ramgs1")
volatile Uint16	adcCh2Buffer[ADC_BUFFER_LEN];	// 2채널 ADC 샘플 원형 버퍼
#pragma DATA_SECTION(adcCh3Buffer, "ramgs2")
volatile Uint16	adcCh3Buffer[ADC_BUFFER_LEN];	// 3채널 ADC 샘플 원형 버퍼
volatile Uint32	adcBufferIndex = 0UL;			// 다음에 쓸 버퍼 위치(ADC_BUFFER_LEN에 닿으면 0으로 순환)

static Uint16 adcDecimationCounter = 0U;		// epwm8_isr 호출 횟수를 세어 ADC_SAMPLE_DECIMATION번마다 1번만 샘플링


// 메인 함수
void main(void)
{

//	1. 전역 인터럽트 스위치 OFF, CPU 인터럽트 벡터 비-활성화 및 플래그(Flag) 비트 클리어
	DINT;			// 전역 인터럽트 스위치 OFF (/INTM OFF)
	IER = 0x0000;	// CPU 인터럽트 벡터 비-활성화
	IFR = 0x0000;	// CPU 인터럽트 플래그 클리어


//	2. 시스템 초기화 - InitSysCtrl( ) 함수 호출 (f28p65x_sysctrl.c)
//	* 왓치독 타이머 비-활성화
//	* CPU 클럭 주파수 설정 (PLL)
//	* 주변회로 클럭 공급 설정
	InitSysCtrl();

//	* 범용 입출력 포트(GPIO) 설정 - InitGpio( ) 함수 호출 후, GPIO213/211/209를
//	  각각 EPWM8A/EPWM14A/EPWM12A 기능(Mux=2)으로 전환 (B Side 91/95/99번 핀 -
//	  ADC 되먹임 핀(B Side 103/105/107번)과 같은 커넥터라 점퍼선이 짧습니다)
	InitGpio();
	GPIO_SetupPinMux(213U, GPIO_MUX_CPU1, 2);	// GPIO213 -> EPWM8A  (1채널, B Side 91번 핀)
	GPIO_SetupPinMux(211U, GPIO_MUX_CPU1, 2);	// GPIO211 -> EPWM14A (2채널, B Side 95번 핀)
	GPIO_SetupPinMux(209U, GPIO_MUX_CPU1, 2);	// GPIO209 -> EPWM12A (3채널, B Side 99번 핀)


//	3. 주변회로 인터럽트 확장회로 초기화 - InitPieCtrl( ) 함수 호출 (f28p65x_piectrl.c)
	InitPieCtrl();


//	4. 주변회로 인터럽트 벡터 확장 및 복사 실행 - InitPieVectTable( ) 함수 호출 (f28p65x_pievect.c)
	InitPieVectTable();


//	5. 인터럽트 벡터와 인터럽트 서비스 루틴 재-연결, 인터럽트 벡터 활성화
//	   (EPWM8_INT -> epwm8_isr 연결은 initAdcSampleInterrupt()에서 6번 단계 중에 합니다 -
//	   EPwm8Regs의 TBCTR/TBPRD가 먼저 설정돼 있어야 하기 때문입니다)


//	6. 주변회로 초기화 - ePWM 3채널 설정
//	   (TBCLK 동기화를 잠깐 정지한 뒤 3채널을 설정하고 다시 동기화를 재개합니다 —
//	   DriverLib 버전의 SysCtl_disablePeripheral/enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC)와
//	   동일한 의도로, 클래식 방식에서는 아래처럼 레지스터 비트를 직접 클리어/셋합니다)
	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;	// TBCLK 동기화 정지

	dutyCh1 = sineTable[0];
	dutyCh2 = sineTable[SINE_PHASE_STEP_120];
	dutyCh3 = sineTable[SINE_PHASE_STEP_240];

	initEPWMChannel(&EPwm8Regs);
	initEPWMChannel(&EPwm14Regs);
	initEPWMChannel(&EPwm12Regs);

	EPwm8Regs.CMPA.bit.CMPA = dutyCh1;
	EPwm14Regs.CMPA.bit.CMPA = dutyCh2;
	EPwm12Regs.CMPA.bit.CMPA = dutyCh3;

	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;	// TBCLK 동기화 재개
	EDIS;

//	   ADC-A 3채널(ADCINA2/A3/A4) 초기화 - PWM-to-Analog 출력 되먹임 확인용
	initAdcFeedback();

//	   EPWM8 주기 인터럽트(100kHz)를 ADC 오버샘플링 기준 클럭으로 배선 - initEPWMChannel()로
//	   EPwm8Regs의 TBCTR/TBPRD가 이미 구성된 뒤에 호출해야 합니다
	initAdcSampleInterrupt();


//	7. 전역 변수 및 S/W 모듈 초기화
	rampCount = 0UL;
	sineIndex = 0U;


//	8. 실시간 디버깅 활성화, 전역 인터럽트 스위치 ON
	ERTM;	// Debug Enable Mask 비트 설정 (실시간 디버깅이 가능하도록 ST1 레지스터의 /DBGM 비트를 0으로 클리어)
	EINT;	// 전역 인터럽트 스위치 ON (/INTM ON)


//	9. Idle(Background) Loop - 240포인트 3상 사인파 듀티 갱신 (120도 위상차)
	for(;;)
	{
		dutyCh1 = sineTable[sineIndex];
		dutyCh2 = sineTable[(sineIndex + SINE_PHASE_STEP_120) % SINE_TABLE_SIZE];
		dutyCh3 = sineTable[(sineIndex + SINE_PHASE_STEP_240) % SINE_TABLE_SIZE];

		EPwm8Regs.CMPA.bit.CMPA  = dutyCh1;
		EPwm14Regs.CMPA.bit.CMPA = dutyCh2;
		EPwm12Regs.CMPA.bit.CMPA = dutyCh3;

		sineIndex++;
		if(sineIndex >= SINE_TABLE_SIZE)
		{
			sineIndex = 0U;
		}

		// ADC 샘플링은 epwm8_isr()가 500Hz(2msec 주기)로 백그라운드에서 원형 버퍼를 계속 채웁니다.

		rampCount++;

		DELAY_US(5000);	// 5msec 지연 (240스텝 * 5ms = 1.200초 주기, 0.833Hz 3상 사인파)
	}
}

//	10. 인터럽트 서비스 루틴 및 기타 함수들
//	    (epwm8_isr가 이 예제의 유일한 인터럽트 서비스 루틴입니다 - ADC 오버샘플링용)

//
// initEPWMChannel - TBPRD=2000, 업카운트, ePWMxA에서 0%~100% 듀티 출력 가능한
// 기본 Action Qualifier 구성 (TBCTR=0에서 High, TBCTR=CMPA에서 Low)
// (DriverLib 버전의 initEPWMChannel()과 정확히 같은 레지스터 값을 만듭니다 —
// EPWM_CLOCK_DIVIDER_1/EPWM_HSCLOCK_DIVIDER_1은 각각 값 0에 대응합니다)
//
void initEPWMChannel(volatile struct EPWM_REGS *pwm)
{
	EALLOW;

	pwm->TBCTL.bit.FREE_SOFT = 3;		// 에뮬레이션 정지 중에도 계속 동작 (Free Run)
	pwm->TBCTL.bit.PRDLD = 1;			// TBPRD 쉐도우 모드 (Shadow Load)
	pwm->TBCTL.bit.PHSEN = 0;			// 위상 로드 비활성화 (동기화 사용 안 함)
	pwm->TBCTL.bit.CLKDIV = 0;			// TBCLK = EPWMCLK / 1
	pwm->TBCTL.bit.HSPCLKDIV = 0;		// TBCLK = EPWMCLK / 1 (HSPCLKDIV도 /1)
	pwm->TBCTL.bit.CTRMODE = 0;		// Up-count 모드 (0에서 TBPRD까지 증가 후 0으로 리셋)

	pwm->TBPRD = PWM_TBPRD;			// 타이머 주기 레지스터 설정 (100kHz)
	pwm->TBCTR = 0U;					// 타이머 카운터 초기화
	pwm->TBPHS.bit.TBPHS = 0U;			// 위상 레지스터 초기화

	pwm->AQCTLA.bit.ZRO = 2;			// TBCTR=0일 때 EPWMxA를 High로 (Set)
	pwm->AQCTLA.bit.CAU = 1;			// TBCTR=CMPA(상승 중)일 때 EPWMxA를 Low로 (Clear)

	EDIS;
}

//
// initAdcFeedback - ADC-A를 12bit/단일종단(Single-Ended) 모드로 켜고, SOC0/1/2를
// 각각 ADCINA2/A3/A4에 배정합니다. TRIGSEL=0(소프트웨어 강제 트리거 전용)으로 두어
// EPWM 트리거 없이 epwm8_isr()가 소프트웨어로 직접 변환을 강제(Force)하는 방식으로
// 씁니다 (6_Potentiometer_1Ch_ADC.c의 InitAdcModules()와 같은 원리이나, 그 예제는
// EPWM2를 전용 트리거로 새로 만드는 반면, 본 예제는 별도 SOC 트리거용 EPWM 없이
// 소프트웨어 강제 트리거 + 폴링 방식을 대신 사용합니다)
//
// RESOLUTION/SIGNALMODE를 레지스터에 직접 쓰는 대신 반드시 AdcSetMode()를
// 통해서 설정해야 합니다 - AdcSetMode()는 그 두 비트만 쓰는 게 아니라 CalAdcINL()로
// 공장 출하 시 OTP에 저장된 선형성(INL) trim 값을 ADCINLTRIM1/2/4/5에 로드하고
// ADCCTL2.bit.OFFTRIMMODE를 "개별 채널별 오프셋 trim 사용"으로 설정합니다. 이 trim을
// 건너뛰면 실제 입력 전압과 무관하게 변환 결과가 크게 어긋날 수 있습니다 - PWM 필터
// 출력이든 가변저항(Potentiometer)이든 무엇을 연결해도 adcCh1Result가 4095로 고정되던
// 증상의 원인이었습니다(device/f28p65x_adc.c를 C2000Ware에서 그대로 복사해서 추가함).
//
void initAdcFeedback(void)
{
	EALLOW;

	//
	// F28P65x A2(AIO229, B Side 107번), A3(AIO230, B Side 105번), A4(AIO231, B Side 103번)
	// 아날로그 모드 활성화: 디지털 입력 버퍼를 분리하고 아날로그 서브시스템 스위치를 연결합니다.
	//
	GpioCtrlRegs.GPHAMSEL.bit.GPIO229 = 1;
	GpioCtrlRegs.GPHAMSEL.bit.GPIO230 = 1;
	GpioCtrlRegs.GPHAMSEL.bit.GPIO231 = 1;

	AnalogSubsysRegs.AGPIOCTRLH.bit.GPIO229 = 1;
	AnalogSubsysRegs.AGPIOCTRLH.bit.GPIO230 = 1;
	AnalogSubsysRegs.AGPIOCTRLH.bit.GPIO231 = 1;

	//
	// 내부 기준전압(VREF) 활성화 (ADCA 내부 1.65V/3.3V 기준전압 버퍼 공급)
	//
	AnalogSubsysRegs.ANAREFCTL.bit.ANAREFASEL = 0;    // 0: 내부 VREF 선택
	AnalogSubsysRegs.ANAREFCTL.bit.ANAREFA2P5SEL = 0;  // 0: 3.3V / 1.65V 모드

	AdcaRegs.ADCCTL2.bit.PRESCALE = 6;		// ADCCLK = SYSCLK/4 (50MHz, 200MHz SYSCLK 기준 권장값)
	AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);	// 12bit, 단일종단 + OTP trim 로드

	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;		// 변환 결과가 레지스터에 반영된 뒤 인터럽트/플래그 발생
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;			// ADC 아날로그 회로 파워-업
	DELAY_US(5000);								// ADC 코어 및 내부 VREF 밴드갭 안정화 대기시간 (5ms)

	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;			// SOC0 -> ADCINA2 (1채널 되먹임, B Side 107번 핀)
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = ADC_ACQPS;
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0;		// 0 = 소프트웨어 강제 트리거 전용(SW Only)

	AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;			// SOC1 -> ADCINA3 (2채널 되먹임, B Side 105번 핀)
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = ADC_ACQPS;
	AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0;

	AdcaRegs.ADCSOC2CTL.bit.CHSEL = 4;			// SOC2 -> ADCINA4 (3채널 되먹임, B Side 103번 핀)
	AdcaRegs.ADCSOC2CTL.bit.ACQPS = ADC_ACQPS;
	AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 0;

	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2;		// ADCINT1 = SOC2(마지막 채널) 변환 완료 시점에 발생
	AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0;		// 연속 모드 비활성화
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;		// ADCINT1 플래그 생성 활성화
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;		// 시작 전 플래그 클리어

	EDIS;
}

//
// initAdcSampleInterrupt - 1채널 PWM(EPWM8)의 주기 인터럽트(TBCTR=0 시점, 10usec마다,
// 100kHz)를 ADC 오버샘플링 기준 클럭으로 배선합니다. epwm8_isr()가 이 인터럽트를 받아서
// ADC_SAMPLE_DECIMATION번에 1번만 실제로 ADC를 변환합니다. initEPWMChannel(&EPwm8Regs)로
// TBCTR/TBPRD가 이미 설정된 뒤에 호출해야 합니다(ETSEL은 그 값을 기준으로 동작함).
//
void initAdcSampleInterrupt(void)
{
	EALLOW;
	EPwm8Regs.ETSEL.bit.INTSEL = 1;	// 1 = TBCTR=0 시점(매 주기 시작)에서 인터럽트 발생
	EPwm8Regs.ETSEL.bit.INTEN = 1;		// EPWM8 인터럽트 활성화
	EPwm8Regs.ETPS.bit.INTPRD = 1;		// 1 = 매 이벤트마다(하드웨어 추가 분주 없음 - 나머지는 SW에서 분주)

	PieVectTable.EPWM8_INT = &epwm8_isr;	// PIE 벡터 3.8에 ISR 연결
	EDIS;

	PieCtrlRegs.PIEIER3.bit.INTx8 = 1;	// PIE 그룹3.8(EPWM8_INT) 활성화
	IER |= M_INT3;						// CPU 인터럽트 3번 활성화
}

//
// epwm8_isr - EPWM8 주기 인터럽트(10usec마다). ADC_SAMPLE_DECIMATION번(기본 200번, 즉
// 2msec)에 1번만 SOC0/1/2를 소프트웨어로 강제 트리거하고, ADCINTFLG.ADCINT1(SOC2 - 마지막
// 채널 - 변환 완료 플래그)이 설 때까지 짧게 폴링한 뒤 결과 3개를 최신값 변수와 원형 버퍼에
// 함께 저장합니다. SOC0/1/2는 우선순위 순서(라운드로빈)로 순차 변환되므로, 마지막 SOC2가
// 끝났다는 것은 SOC0/SOC1도 이미 끝났다는 뜻입니다.
//
// (주의: ADCCTL1.bit.ADCBSY를 강제 트리거 직후 바로 폴링하면 안 됩니다 - SOC를 force한
// 뒤 busy 플래그가 실제로 올라가기까지 몇 사이클 지연이 있어서, 그 사이에 while 조건이
// 이미 0을 읽고 빠져나가 변환이 끝나기도 전에 스테일(stale)한 ADCRESULT 값을 읽어버리는
// 레이스가 있었습니다(adcCh1Result가 4095로 고정되던 원인이었습니다). ADCINTFLG는
// INTPULSEPOS=1과 함께 "결과 레지스터에 반영된 뒤"에만 서므로 이 레이스가 없습니다.)
//
// FLASH 메모리에서 코드 실행 시, 인터럽트 서비스 루틴을 램 영역으로 복사해서 고속 실행하도록 함
#ifdef _FLASH
	#pragma CODE_SECTION(epwm8_isr, ".TI.ramfunc");
#endif
interrupt void epwm8_isr(void)
{
	EPwm8Regs.ETCLR.bit.INT = 1;	// EPWM8 자체 인터럽트 플래그 클리어(다음 주기에 또 뜨도록)

	adcDecimationCounter++;
	if(adcDecimationCounter >= ADC_SAMPLE_DECIMATION)
	{
		adcDecimationCounter = 0U;

		AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	// 이전 샘플의 플래그 클리어(선행 클리어라 레이스 없음)
		AdcaRegs.ADCSOCFRC1.all = 0x0007;		// SOC0/1/2 동시 강제 트리거 (bit0~2 = SOC0/SOC1/SOC2)

		Uint16 timeout = 0xFFFF;
		while((AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0) && (timeout > 0))
		{
			timeout--;
			// SOC2(마지막 채널) 변환이 끝날 때까지 대기 (ACQPS+12bit 변환 - 수usec 이내)
		}

		adcCh1Result = AdcaResultRegs.ADCRESULT0;
		adcCh2Result = AdcaResultRegs.ADCRESULT1;
		adcCh3Result = AdcaResultRegs.ADCRESULT2;

		adcCh1Buffer[adcBufferIndex] = adcCh1Result;
		adcCh2Buffer[adcBufferIndex] = adcCh2Result;
		adcCh3Buffer[adcBufferIndex] = adcCh3Result;

		adcBufferIndex++;
		if(adcBufferIndex >= ADC_BUFFER_LEN)
		{
			adcBufferIndex = 0UL;	// 원형 버퍼 - 끝까지 차면 처음으로 돌아가 계속 덮어씀(라이브 그래프용)
		}
	}

	PieCtrlRegs.PIEACK.bit.ACK3 = 1;	// PIE 그룹3 Acknowledge(같은 그룹의 다음 인터럽트를 받으려면 필요)
}

// 파일 끝.
