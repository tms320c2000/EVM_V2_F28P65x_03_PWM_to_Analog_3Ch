# PWM-to-Analog 출력 3채널 테스트 — 비트필드(레지스터 직접 제어) 버전 — TMS320F28P659DK8-Q1

[3_PWM_to_Analog_3Ch_Driverlib](../3_PWM_to_Analog_3Ch_Driverlib/README.md)(DriverLib 버전)와
동일한 동작을 **DriverLib을 전혀 쓰지 않고** TI의 클래식 레지스터 비트필드 구조체
(`EPwm8Regs`, `EPwm14Regs`, `EPwm12Regs`)만으로 구현했습니다. `driverlib.lib` 자체가 이
프로젝트에는 없습니다.

## 이 버전의 핵심
- 시스템 초기화: `InitSysCtrl()` (DriverLib의 `Device_init()`에 대응, 클래식 헬퍼)
- GPIO→EPWM 핀 전환: `GPIO_SetupPinMux(gpio, GPIO_MUX_CPU1, 2)` (클래식 헬퍼, Mux=2가
  EPWM8A/12A/14A를 선택 — DriverLib의 `GPIO_setPinConfig(GPIO_213_EPWM8_A)` 등과 같은 값)
- **ePWM 설정**: `TBCTL`/`TBPRD`/`AQCTLA` 레지스터를 직접 씀 — 이게 DriverLib 버전의
  `EPWM_setTimeBasePeriod()`/`EPWM_setActionQualifierAction()` 등과 정확히 대응되는
  지점입니다.
- **듀티 갱신**: `EPwmxRegs.CMPA.bit.CMPA`에 직접 씀 — DriverLib 버전의
  `EPWM_setCounterCompareValue()`와 대응.
- TBCLK 동기화 정지/재개는 `CpuSysRegs.PCLKCR0.bit.TBCLKSYNC`를 직접 클리어/셋 — DriverLib
  버전의 `SysCtl_disablePeripheral()`/`SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC)`와
  대응.

## 요구 하드웨어
- SyncWorks TMS320F28X 개발보드 V2
- TMS320F28P650DK9 또는 TMS320F28P659DK8-Q1 모듈
- 점퍼 케이블 6개 (PWM 출력 3개 + ADC 되먹임 3개)
- (선택) 오실로스코프 또는 멀티미터

## 배선 (DriverLib, FreeRTOS 예제와 100% 동일)

개발보드 B Side 핀-헤더와 보드의 **(3) PWM-to-Analog 4채널** 블록을 점퍼 와이어로 연결하세요:

| 채널 | PWM 출력 (B Side) | PWM-to-Analog 입력 단자 | PWM-to-Analog LPF 출력 단자 | ADC 되먹임 입력 (B Side) |
|---|---|---|---|---|
| **1채널 (U상)** | **B Side 91번** (GPIO213, EPWM8A) | ──▶ **`PWM.IN1`** | **`A.OUT1`** ──▶ | **B Side 107번** (ADCINA2 / AIO229) |
| **2채널 (V상)** | **B Side 95번** (GPIO211, EPWM14A) | ──▶ **`PWM.IN2`** | **`A.OUT2`** ──▶ | **B Side 105번** (ADCINA3 / AIO230) |
| **3채널 (W상)** | **B Side 99번** (GPIO209, EPWM12A) | ──▶ **`PWM.IN3`** | **`A.OUT3`** ──▶ | **B Side 103번** (ADCINA4 / AIO231) |

> **안내**: 보드 실크 인쇄 명칭은 매뉴얼 75페이지 [그림 10-3]에서 확인되었습니다:
> - 입력: `PWM.IN1` ~ `PWM.IN4`
> - 출력: `A.OUT1` ~ `A.OUT4`, `GND`
> - F28P65x에서 ADCINA2/A3/A4는 AIO229/AIO230/AIO231로 멀티플렉싱되어 있어,
>   `initAdcFeedback()`에서 `GPHAMSEL` 및 `AGPIOCTRLH`를 설정해야 신호가 유입됩니다.

![F28P65x 모듈 - 개발보드 V2 (3) PWM-to-Analog 출력 3채널 배선도](../3_PWM_to_Analog_3Ch_Driverlib/f28xevm_v2_pwm_to_analog_3ch.png)

## 소프트웨어 버전
CCS 21.x / **SysConfig·C2000Ware 라이브러리 링크 없음** (products="C2000WARE"만 사용) /
CGT 22.6.3.LTS. `device/common_include`, `device/headers_include`에 필요한 클래식 헤더를
전부 복사해 자기완결형으로 만들었습니다.

## Import → Build → Flash → Run
1. CCS에서 `CCS/3_PWM_to_Analog_3Ch_Bitfield.projectspec`를 Import
2. Build (CPU1_RAM 또는 CPU1_FLASH)
3. Debug 연결 후 Flash/Run — `.ccxml`은 XDS2xx USB 디버그 프로브 설정을 사용합니다
   (DK8-Q1 최초 연결 시 확인 필요, Driverlib 버전과 동일한 주의사항)

## 정상 동작 확인
3상 사인파(SPWM, 120도 위상차) 출력이 보이면 정상입니다. `dutyCh1`/`dutyCh2`/`dutyCh3`/
`rampCount`도 동일하게 CCS Expressions에서 확인 가능합니다.

## ADC 피드백 확인

PWM-to-Analog 회로가 실제로 만들어낸 아날로그 전압을 눈으로만 볼 게 아니라 칩이
직접 되읽어서(ADC) 확인할 수 있도록, ADCINA2/A3/A4 3채널로 출력을 되먹임(feedback)
받는 기능을 추가했습니다. 배선은 위 "배선" 절과 동일합니다.

### 동작 방식

1채널 PWM(EPWM8)의 주기 인터럽트(`epwm8_isr`, 매 10usec/100kHz)를 ADC 오버샘플링
기준 클럭으로 씁니다. 인터럽트마다 소프트웨어 카운터를 세다가 `ADC_SAMPLE_DECIMATION`
(기본 200)번에 1번만 `ADCSOCFRC1`로 SOC0/1/2를 동시에 소프트웨어 강제 트리거하고,
`ADCINTFLG.bit.ADCINT1`(SOC2 - 마지막 채널 - 변환 완료 플래그)이 설 때까지 짧게
폴링한 뒤 결과 3개를 읽습니다. 즉 유효 샘플레이트는 500Hz(2msec 간격)입니다.

읽은 값은 `adcCh1Result`/`adcCh2Result`/`adcCh3Result`(최신 1개 값)뿐 아니라
`adcCh1Buffer`/`adcCh2Buffer`/`adcCh3Buffer`(각 1024워드 원형 버퍼)에도 순서대로
쌓입니다. 1024개 * 2msec ≈ 2.05초 분량이라 1.2초 주기의 3상 사인파 약 1.7주기 전체가 항상
버퍼 안에 들어 있고, 버퍼가 다 차면 처음(인덱스 0)으로 돌아가 계속 덮어씁니다 —
CCS Graph를 연결해두면 계속 갱신되는 3상 교차 사인파 "라이브 스코프"를 볼 수 있습니다.

세 채널의 1024워드 버퍼(6KB)는 기본 `.bss` 영역(`RAMLS5`, 2KB)에 다 들어가지 않아,
`#pragma DATA_SECTION`으로 채널별로 `RAMGS0`/`RAMGS1`/`RAMGS2`(각 8KB, 이 cmd 파일이
원래 SDFM용으로 비워둔 미사용 블록)에 나눠 배치했습니다.

ADC 변환 완료 대기는 `ADCCTL1.bit.ADCBSY` 대신 `ADCINTFLG`를 폴링합니다 — `ADCBSY`는
SOC를 강제(force)한 직후 바로 폴링하면 busy 플래그가 실제로 올라가기까지 몇 사이클
지연이 있어 레이스 컨디션이 생기지만(TI 공식 `adc_ex1_soc_software.c`,
`6_Potentiometer_1Ch_ADC.c`도 같은 이유로 `ADCINTFLG` 방식을 씀), `ADCINTFLG`는
`INTPULSEPOS=1`과 함께 결과 레지스터에 반영된 뒤에만 서므로 이 문제가 없습니다.

ADC 초기화는 `RESOLUTION`/`SIGNALMODE` 레지스터를 직접 쓰지 않고 반드시 `AdcSetMode()`
(`device/f28p65x_adc.c`, C2000Ware 제공)를 거칩니다 — 이 함수는 그 두 비트뿐 아니라
`CalAdcINL()`로 공장 출하 시 OTP에 저장된 선형성(INL) trim 값을 로드하고
`ADCCTL2.bit.OFFTRIMMODE`를 채널별 오프셋 trim 사용으로 설정합니다. 이 trim이 없으면
실제 입력 전압과 무관하게 변환 결과가 풀스케일(4095) 근처로 고정됩니다.

F28P65x에서 A2/A3/A4는 AIO229/230/231이므로, `initAdcFeedback()`에서 `GPHAMSEL`과
`AGPIOCTRLH`로 아날로그 스위치를 열어야 외부 신호가 ADC 코어로 유입됩니다.
`AnalogSubsysRegs.ANAREFCTL`로 내부 VREF(1.65V/3.3V)를 공급하고, `PRESCALE`은
SYSCLK/4(50MHz)를 사용하며, `epwm8_isr` 폴링에는 타임아웃 보호가 걸려 있습니다.

PWM 듀티 변조는 240포인트 사인 룩업 테이블(듀티 100~1900, 중심 1000)로 1채널(U상, 0°),
2채널(V상, +120°), 3채널(W상, +240°)의 3상 사인파(SPWM)를 만듭니다(1주기 = 240 * 5ms =
1.200초, 0.833Hz). LPF 통과 후 실제 3상 아날로그 전압이 출력되어 ADCINA2/A3/A4로
되읽을 때 3개 위상이 교차하는 사인파가 표시됩니다.

### 확인 방법

CCS **Tools > Graph > Single Time**로 아래처럼 설정하면 세 채널 모두 3상 사인파 전체
모양을 실시간으로 볼 수 있습니다(Start Address만 채널별로 다르게):

| 설정 | 값 |
|---|---|
| Acquisition Buffer Size | 1024 |
| DSP Data Type | 16-bit unsigned |
| Sampling Rate (Hz) | 500 |
| Start Address | `adcCh1Buffer` (또는 `adcCh2Buffer` / `adcCh3Buffer`) |

CCS Expressions 창에 `adcCh1Result`/`adcCh2Result`/`adcCh3Result`(최신 샘플 1개,
0~4095)도 함께 등록해두면 그래프 없이도 값이 바뀌는지 바로 확인할 수 있습니다.

## 세 가지 버전 비교
| 버전 | 폴더 | 핵심 차이 |
|---|---|---|
| DriverLib | [3_PWM_to_Analog_3Ch_Driverlib](../3_PWM_to_Analog_3Ch_Driverlib/) | TI 표준 HAL 함수 호출 |
| **비트필드(이 폴더)** | 3_PWM_to_Analog_3Ch_Bitfield | 레지스터 구조체 직접 조작, DriverLib 미사용 |
| FreeRTOS | [3_PWM_to_Analog_3Ch_Freertos](../3_PWM_to_Analog_3Ch_Freertos/) | DriverLib + 태스크 스케줄링, 정적 할당 |

### 메모리 실측 비교 (2026-09-18, CCS 21.x / C:\ti\ccs2100, CPU1_RAM 빌드, `.map` 기준)

세 프로젝트 모두 `C:\Users\vosam\workspace_ccstheia`에서 실제로 Import → Build까지 성공한
뒤의 `.map` 파일(TI 링커 Grand Total, 16비트 워드 단위)을 기준으로 한 실측치입니다 —
추정이 아닙니다.

| 버전 | code | ro data | rw data | 합계 |
|---|---:|---:|---:|---:|
| DriverLib | 4,619 | 1,341 | 4,111 | **10,071** |
| **비트필드(이 폴더)** | 4,205 | 714 | 6,683 | **11,602** |
| FreeRTOS | 6,463 | 1,563 | 4,623 | **12,649** |

- 세 버전 모두 rw data의 대부분(3,072워드 = 3채널 × 1024워드)은 `RAMGS0`/`RAMGS1`/`RAMGS2`에
  나눠 배치한 ADC 원형 버퍼입니다.
- **이 비트필드 버전이 code는 DriverLib보다 작지만(4,205 vs 4,619) rw data는 훨씬 큽니다
  (6,683 vs 4,111)** — 클래식 헤더(`f28p65x_globalvariabledefs.obj`)가 `EPwm8Regs`/
  `AdcaRegs` 등 모든 페리페럴 레지스터 구조체를 파일 전역으로 선언하면서 rw data에
  3,333워드를 추가로 잡아먹기 때문입니다. DriverLib은 이런 전역 레지스터 구조체 선언이
  없어 이 부담이 없습니다.

## 관련 링크
- 상품 페이지: https://tms320f28x.co.kr/goods/goods_view.php?goodsNo=200903127
- 게시판 글: https://tms320f28x.co.kr/board/view.php?bdId=tms320f28xevmv2&sno=104
- 유튜브 영상: (게시 후 URL 추가 예정)
