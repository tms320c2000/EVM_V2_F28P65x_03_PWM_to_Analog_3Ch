# PWM-to-Analog 출력 3채널 테스트 — DriverLib 버전 — TMS320F28P659DK8-Q1

TMS320F28P65x 개발보드 V2의 회로블록 **(3) PWM-to-Analog 출력 3채널**을 순수 DriverLib
(SysConfig 미사용)로 구동합니다. 240포인트 사인 룩업 테이블을 이용해 120도 위상차를 가진
**3상 사인파(SPWM)**를 발생시키며, 필터 후단에서 출력되는 3상 아날로그 전압을
ADCINA2/A3/A4로 되읽어 채널당 1024워드 크기의 원형 버퍼에 실시간 저장합니다.

## 요구 하드웨어
- SyncWorks TMS320F28X 개발보드 V2
- TMS320F28P650DK9 또는 TMS320F28P659DK8-Q1 모듈
- 점퍼 케이블 6개 (PWM 출력 3개 + ADC 되먹임 3개)
- (선택) 오실로스코프 또는 멀티미터

## 배선 (Bitfield, FreeRTOS 예제와 100% 동일)

개발보드 B Side 핀-헤더와 보드의 **(3) PWM-to-Analog 4채널** 블록을 점퍼 와이어로 연결하세요:

| 채널 | PWM 출력 (B Side) | PWM-to-Analog 입력 단자 | PWM-to-Analog LPF 출력 단자 | ADC 되먹임 입력 (B Side) |
|---|---|---|---|---|
| **1채널 (U상)** | **B Side 91번** (GPIO213, EPWM8A) | ──▶ **`PWM.IN1`** | **`A.OUT1`** ──▶ | **B Side 107번** (ADCINA2 / AIO229) |
| **2채널 (V상)** | **B Side 95번** (GPIO211, EPWM14A) | ──▶ **`PWM.IN2`** | **`A.OUT2`** ──▶ | **B Side 105번** (ADCINA3 / AIO230) |
| **3채널 (W상)** | **B Side 99번** (GPIO209, EPWM12A) | ──▶ **`PWM.IN3`** | **`A.OUT3`** ──▶ | **B Side 103번** (ADCINA4 / AIO231) |

> **안내 (AIO 핀 주의사항)**:
> - 보드 실크 인쇄 명칭은 매뉴얼 75페이지 [그림 10-3]에서 확인되었습니다:
>   - 입력: `PWM.IN1` ~ `PWM.IN4`
>   - 출력: `A.OUT1` ~ `A.OUT4`, `GND`
> - **PWM 출력 핀(GPIO213/211/209)**: F28P65x의 Port G AIO 핀으로, 칩 리셋 시 `GPGAMSEL` 비트가 1(아날로그 모드)로 기본 설정되어 디지털 출력 드라이버가 하드웨어적으로 차단됩니다. 반드시 `GPIO_setAnalogMode(pin, GPIO_ANALOG_DISABLED)`를 호출해 디지털 출력으로 전환해야 ePWM 파형이 물리 핀으로 정상 출력됩니다.
> - **ADC 되먹임 핀(ADCINA2/A3/A4)**: AIO229/AIO230/AIO231로 멀티플렉싱되어 있어, `GPIO_setAnalogMode(pin, GPIO_ANALOG_ENABLED)`를 호출해 아날로그 서브시스템 스위치를 연결해야 전압이 정상 측정됩니다.

![F28P65x 모듈 - 개발보드 V2 (3) PWM-to-Analog 출력 3채널 배선도](f28xevm_v2_pwm_to_analog_3ch.png)

## 소프트웨어 구성
- CCS: 21.x (Theia 기반)
- **SysConfig 미사용** — `products="C2000WARE"`만 사용
- C2000Ware: 26.00.00.00 — `device.h`/`device.c`/`driverlib.h`/driverlib 헤더 전체/
  `driverlib.lib`를 이 폴더 안에 전부 복사해 두어 C2000Ware 설치 경로에 의존하지 않습니다.
- Code Generation Tools: 22.6.3.LTS

## 동작 방식

1. **3상 사인파(SPWM) 발생 (120도 위상차)**:
   - 240포인트 사인 룩업 테이블(듀티 100~1900, 중심 1000)을 5ms마다 1스텝씩 전진시킵니다.
   - 1주기는 240 × 5ms = 1.200초 (0.833Hz)입니다.
   - 1채널(U상, 0°), 2채널(V상, +120° = 80스텝 오프셋), 3채널(W상, +240° = 160스텝 오프셋)의
     위상차로 갱신되어 모터/인버터 구동과 동일한 3상 교차 사인파가 생성됩니다.
2. **DriverLib ADC 초기화 (`initAdcFeedback`)**:
   - `GPIO_setAnalogMode()`를 호출하여 AIO229/230/231 핀을 아날로그 입력 모드로 설정합니다.
   - `ASysCtl_setAnalogReferenceInternal(ASYSCTL_VREFHIA)`로 3.3V 내부 기준전압을 공급합니다.
   - `ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0)`로 50MHz ADCCLK를 설정합니다.
   - SOC0/1/2를 ADCINA2/A3/A4에 배정하고, SOC2 완료 시 `ADCINT1` 플래그가 발생하도록 설정합니다.
3. **EPWM8 주기 인터럽트 기반 오버샘플링 (`epwm8ISR`)**:
   - EPWM8의 주기 인터럽트(100kHz, 매 10usec)를 카운트하여 200회마다 1회(500Hz, 2ms 간격)
     ADC 변환을 소프트웨어 강제 트리거(`ADC_forceMultipleSOC`)합니다.
   - 변환 결과는 채널당 1024워드 크기의 원형 버퍼(`adcCh1Buffer`, `adcCh2Buffer`, `adcCh3Buffer`)에
     저장됩니다.
   - 1024샘플 × 2ms = 2.048초 분량이 저장되므로, CCS Graph 창에서 약 1.7주기의 3상 교차 사인파가
     실시간 스코프처럼 계속 흘러가는 모습을 관찰할 수 있습니다.

> **메모리 배치**: 1024워드 × 3채널(6KB) 버퍼는 기본 `.bss`(RAMLS5, 2KB)에 다 들어가지 않으므로,
> `#pragma DATA_SECTION`으로 `RAMGS0`, `RAMGS1`, `RAMGS2`(각 8KB)에 나눠 배치했습니다.

## 정상 동작 확인

### 1. Expressions 창
CCS Expressions 창에 아래 변수들을 등록하여 실시간 값이 3상 사인파 주기에 맞춰 부드럽게
교차하며 변하는지 확인합니다:
- `dutyCh1`, `dutyCh2`, `dutyCh3` (100 ~ 1900)
- `adcCh1Result`, `adcCh2Result`, `adcCh3Result` (0 ~ 4095)
- `rampCount`

### 2. CCS Graph 실시간 파형 확인
CCS **Tools > Graph > Single Time**로 아래처럼 설정하면 각 채널의 3상 사인파 파형을
실시간으로 볼 수 있습니다:

| 설정 항목 | 설정 값 |
|---|---|
| Acquisition Buffer Size | 1024 |
| DSP Data Type | 16-bit unsigned |
| Sampling Rate (Hz) | 500 |
| Start Address | `adcCh1Buffer` (또는 `adcCh2Buffer` / `adcCh3Buffer`) |
| Display Data Size | 1024 |

Continuous Refresh를 켜면 3개 채널의 아름다운 사인파 교차 파형을 확인할 수 있습니다.

## 세 가지 버전 비교
같은 PWM-to-Analog 3채널 출력을 세 가지 방식으로 구현해 비교합니다.

| 버전 | 폴더 | 핵심 차이 |
|---|---|---|
| **DriverLib(이 폴더)** | 3_PWM_to_Analog_3Ch_Driverlib | TI 표준 HAL 함수 호출, EPWM8 ISR 샘플링 |
| 비트필드 | [3_PWM_to_Analog_3Ch_Bitfield](../3_PWM_to_Analog_3Ch_Bitfield/) | 레지스터 구조체 직접 조작, EPWM8 ISR 샘플링 |
| FreeRTOS | [3_PWM_to_Analog_3Ch_Freertos](../3_PWM_to_Analog_3Ch_Freertos/) | DriverLib + FreeRTOS 태스크 스케줄링(정적 할당) |

### 메모리 실측 비교 (2026-09-18, CCS 21.x / C:\ti\ccs2100, CPU1_RAM 빌드, `.map` 기준)

세 프로젝트 모두 `C:\Users\vosam\workspace_ccstheia`에서 실제로 Import → Build까지 성공한
뒤의 `.map` 파일(TI 링커 Grand Total, 16비트 워드 단위)을 기준으로 한 실측치입니다 —
추정이 아닙니다.

| 버전 | code | ro data | rw data | 합계 |
|---|---:|---:|---:|---:|
| **DriverLib(이 폴더)** | 4,619 | 1,341 | 4,111 | **10,071** |
| 비트필드 | 4,205 | 714 | 6,683 | **11,602** |
| FreeRTOS | 6,463 | 1,563 | 4,623 | **12,649** |

- 세 버전 모두 rw data의 대부분(3,072워드 = 3채널 × 1024워드)은 `RAMGS0`/`RAMGS1`/`RAMGS2`에
  나눠 배치한 ADC 원형 버퍼입니다 — 실제 응용 코드가 쓰는 rw data는 이보다 훨씬 작습니다.
- **비트필드가 code는 DriverLib보다 작지만(4,205 vs 4,619) rw data는 훨씬 큽니다(6,683 vs
  4,111)** — 클래식 헤더(`f28p65x_globalvariabledefs.obj`)가 `EPwm8Regs`/`AdcaRegs` 등
  모든 페리페럴 레지스터 구조체를 파일 전역으로 선언하면서 rw data에 3,333워드를 추가로
  잡아먹기 때문입니다. DriverLib은 이런 전역 레지스터 구조체 선언이 없어 이 부담이 없습니다.
- **FreeRTOS는 DriverLib 대비 code +1,844워드**가 순수 커널 오버헤드(스케줄러, 태스크
  전환)이고, rw data는 정적 태스크 스택 3개(각 256워드) 중 일부가 겹쳐 배치되어 순증가는
  +512워드에 그칩니다.

## 관련 링크
- 상품 페이지: https://tms320f28x.co.kr/goods/goods_view.php?goodsNo=200903127
- 보드 핀헤더 정의표: https://www.tms320f28x.co.kr/data/goods/swgoods/4064/TMS320F28X_EVM_V2_PinHeader_Reference_F28P650DK9_Module.pdf
