# PWM-to-Analog 출력 3채널 테스트 — FreeRTOS 멀티태스킹 버전 — TMS320F28P659DK8-Q1

TMS320F28P65x 개발보드 V2의 회로블록 **(3) PWM-to-Analog 출력 3채널**을 FreeRTOS 멀티태스킹
(정적 할당, 힙 미사용)과 DriverLib API로 구동합니다. 240포인트 사인 룩업 테이블을 이용해 120도
위상차를 가진 **3상 사인파(SPWM)**를 발생시키며, 독립된 ADC 전용 태스크가 필터 후단 아날로그
전압을 되읽어 1024워드 원형 버퍼에 실시간 저장합니다.

## 이 버전의 핵심 — FreeRTOS 멀티태스킹 아키텍처

1. **태스크 1: `PWM_Task` (우선순위: `tskIDLE_PRIORITY + 1`, 주기: 5ms)**
   - 240포인트 사인 룩업 테이블(듀티 100~1900, 중심 1000)을 5ms마다 1스텝씩 전진시킵니다.
   - 1주기 = 240 × 5ms = 1.200초 (0.833Hz) 3상 교차 사인파를 발생시킵니다.
   - 1채널(U상, 0°), 2채널(V상, +120° = 80스텝 오프셋), 3채널(W상, +240° = 160스텝 오프셋).
2. **태스크 2: `ADC_Task` (우선순위: `tskIDLE_PRIORITY + 2`, 주기: 2ms = 500Hz)**
   - `PWM_Task`보다 높은 우선순위로 동작하여 샘플링 지터(Jitter)를 최소화합니다.
   - 2ms마다 `ADC_forceMultipleSOC()`를 호출해 3개 채널(ADCINA2/A3/A4)을 동시 변환하고,
     완료 플래그를 확인한 뒤 원형 버퍼(`adcCh1Buffer`, `adcCh2Buffer`, `adcCh3Buffer`)에 저장합니다.
   - 1024샘플 × 2ms = 2.048초 분량(약 1.7주기)의 3상 교차 파형이 버퍼에 상시 유지됩니다.
3. **완전한 정적 메모리 할당 (Zero Heap)**:
   - `configSUPPORT_DYNAMIC_ALLOCATION=0`, `configSUPPORT_STATIC_ALLOCATION=1` 설정으로
     힙 메모리를 일체 사용하지 않습니다.
   - 태스크 스택(`STACK_SIZE = 256`워드 = 512바이트)과 TCB를 모두 정적 전역 변수로 할당하여
     `.freertosStaticStack` 섹션에 안전하게 배치했습니다.

## 요구 하드웨어 / 배선 (Bitfield, DriverLib 예제와 100% 동일)

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

![F28P65x 모듈 - 개발보드 V2 (3) PWM-to-Analog 출력 3채널 배선도](../3_PWM_to_Analog_3Ch_Driverlib/f28xevm_v2_pwm_to_analog_3ch.png)

## 소프트웨어 구성
- CCS: 21.x (Theia 기반)
- **SysConfig 미사용** — `products="C2000WARE"`만 사용
- C2000Ware: 26.00.00.00 driverlib(로컬 복사)
- **FreeRTOS 커널 소스 전체를 `FreeRTOS/` 폴더에 로컬 복사** (TI kernel/FreeRTOS 배포본 기준)
- Code Generation Tools: 22.6.3.LTS

## Import → Build → Flash → Run
1. CCS에서 `CCS/3_PWM_to_Analog_3Ch_Freertos.projectspec`를 Import
2. Build (CPU1_RAM 또는 CPU1_FLASH) — FreeRTOS 전용 링커 cmd(`28p65x_freertos_*_lnk_cpu1.cmd`) 사용.
   - 1024워드 원형 버퍼 3개는 `#pragma DATA_SECTION`으로 `RAMGS0`, `RAMGS1`, `RAMGS2`(각 8KB)에 배치.
3. Debug 연결 후 Flash/Run — `.ccxml`은 XDS2xx USB 디버그 프로브 설정 사용.

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
| DriverLib | [3_PWM_to_Analog_3Ch_Driverlib](../3_PWM_to_Analog_3Ch_Driverlib/) | TI 표준 HAL 함수 호출, EPWM8 ISR 샘플링 |
| 비트필드 | [3_PWM_to_Analog_3Ch_Bitfield](../3_PWM_to_Analog_3Ch_Bitfield/) | 레지스터 구조체 직접 조작, EPWM8 ISR 샘플링 |
| **FreeRTOS(이 폴더)** | 3_PWM_to_Analog_3Ch_Freertos | DriverLib + FreeRTOS 태스크 스케줄링(정적 할당, PWM_Task + ADC_Task) |

### 메모리 실측 비교 (2026-09-18, CCS 21.x / C:\ti\ccs2100, CPU1_RAM 빌드, `.map` 기준)

세 프로젝트 모두 `C:\Users\vosam\workspace_ccstheia`에서 실제로 Import → Build까지 성공한
뒤의 `.map` 파일(TI 링커 Grand Total, 16비트 워드 단위)을 기준으로 한 실측치입니다 —
추정이 아닙니다.

| 버전 | code | ro data | rw data | 합계 |
|---|---:|---:|---:|---:|
| DriverLib | 4,619 | 1,341 | 4,111 | **10,071** |
| 비트필드 | 4,205 | 714 | 6,683 | **11,602** |
| **FreeRTOS(이 폴더)** | 6,463 | 1,563 | 4,623 | **12,649** |

- 세 버전 모두 rw data의 대부분(3,072워드 = 3채널 × 1024워드)은 `RAMGS0`/`RAMGS1`/`RAMGS2`에
  나눠 배치한 ADC 원형 버퍼입니다.
- **이 FreeRTOS 버전은 DriverLib 대비 code +1,844워드**가 순수 커널 오버헤드(스케줄러,
  컨텍스트 스위칭)이고, rw data는 정적 태스크 스택 3개(`pwmTaskStack`/`adcTaskStack`/
  `idleTaskStack`, 각 `STACK_SIZE=256`워드)가 추가되지만 일부가 겹쳐 배치되어 DriverLib
  대비 순증가는 +512워드에 그칩니다.

## 관련 링크
- 상품 페이지: https://tms320f28x.co.kr/goods/goods_view.php?goodsNo=200903127
- 게시판 글: https://tms320f28x.co.kr/board/view.php?bdId=tms320f28xevmv2&sno=104
- 보드 핀헤더 정의표: https://www.tms320f28x.co.kr/data/goods/swgoods/4064/TMS320F28X_EVM_V2_PinHeader_Reference_F28P650DK9_Module.pdf
