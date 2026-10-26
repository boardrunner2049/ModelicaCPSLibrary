# FIRE 최소 multicopter 모델

활성 패키지는 `MULTICOPTER_INTERFACES.md` v0.8과
`261006_quadrotor-fidelity-port-contract.md`의 최초 실행 범위를 구현합니다.
**연속 P0 motor + R0 rotor + 단일 6DOF 강체 + 이상 센서 + 4점 탄성 착륙다리**, SI 단위,
world NED / chassis·body FRD가 기본입니다.

- `Vehicles.Copter.MultirotorPlant`: N개 로터의 hub wrench를 CG에서 합산합니다.
- `Vehicles.Copter.MultirotorWithSensors`: IMU, magnetometer, GNSS, barometer를 추가합니다.
- `Vehicles.Copter.Presets`: QuadX, HexaX, OctoX, CoaxialX8. Coaxial은 배치만 다르며 간섭 공력은 없습니다.
- `Physical.Mechanical.Chassis`: aggregate 또는 정적 부품 합성으로 질량·CG·full inertia를 계산합니다.
- `Adapters.VirtualIO`: PWM·센서 값 변환과 독립적인 SPI 바이트→신호 adapter입니다.
- `Exports.FMU`: 외부 PWM·센서 포트와 진단 출력을 정의하며 P0/P1/P2 plant는 composer가 조립합니다.

`demand`는 **추력 비율이 아닌 목표 회전속도 비율**입니다. 모터 lag는 한 번만 적용됩니다.
기본 센서는 연속 이상 관측식과 bias 0을 사용합니다. 선택적 IMU profile이
샘플링·hold·timestamp·응답 필터를 추가합니다.
4점 착륙다리는 기본 활성화되어 수평 지면에서 기체를 지지합니다.
`landingGear(enabled=false)`를 지정하면 지면을 통과하는 자유낙하 모델이 됩니다.
P1은 ideal voltage-only ESC와 전압 기반 속도 응답을 추가합니다.
P2는 인덕턴스를 포함한 전류·토크·부하 피드백을 추가합니다.
일반 지형·항력·배터리·프로토콜 모델은 활성 경로에 없습니다.

질량 record에는 `mass`, `r_C`, `R_Cj`, `inertia`만 포함합니다. 부품 `componentId`는
TOML과 manifest에 보존하며 composer가 모델 생성 전에 비어 있지 않은 ID의 중복을 검사합니다.
직접 작성하는 Modelica의 `MassProperties(...)` 및 파생 record 생성자에서는 `componentId`
수정자를 제거해야 합니다. Composer를 거치지 않는 경우 부품 중복 계상은 호출자가 확인해야 하며,
질량·회전행렬·관성의 물리적 유효성 검사는 기존 Modelica 함수에서 계속 수행합니다.

## 센서 fidelity (2026-10-26, 개정)

**I0 이상값 → I1 디지털 출력 근사 → I2 교정·통계 오차 → I3 기계·전기 물리 모델**로
구성합니다. 추진계와 독립적으로 `[models.imu]`의 `model`을
`ideal`, `digital`, `errors`, `electromechanical` 중 선택합니다.
[IMU 사용법·수식·한계](Systems/Sensing/IMU/README.md)에 상세 계약을 기록했습니다.

I1은 대역폭·ADC·IIR·decimation·전달 지연을, I2는 교정·온도·시변 bias·잡음을 추가합니다.
I2 오차를 0으로 두면 같은 설정의 I1과 일치합니다. I3는 3축 자이로의 drive/sense 진동,
정전용량 검출, AGC·PLL·복조와 3축 가속도계 proof mass를 구현합니다.
전기부는 carrier를 평균화한 C/V 및 제어 루프 모델이며, transistor 회로나 실제 칩 복제는 아닙니다.
Sense는 개방형이며 force rebalance는 후속입니다.

[독립 음향 vulnerability](Vulnerabilities/Acoustic/README.md)는
`[vulnerabilities.acoustic]`에서 선택합니다. I1/I2는 `equivalent_rate`로 RMS envelope를,
I3는 `modal_force`로 순간 음압을 받아 MEMS에 힘을 전달합니다.
패키지는 유효면적·결합계수·방향 투영만 사용하며 추가 패키지 동역학이나 FEM은 없습니다.

`configs/quad_i1.toml`–`quad_i3.toml` 및 `quad_i2_acoustic.toml`, `quad_i3_acoustic.toml`을
사용할 수 있습니다. 계수는 예시이며 ICM-42688-P 실측 보정값이 아닙니다.
기존 `statistical`, `device`, `acoustic` 선택은 동작을 보존하며 manifest에서 legacy로 구분합니다.
I0 기본식은 유지합니다. [검증 범위](docs/validation.md)는 native/FMI2와 hardware·Rumoca/FMI3·
FastDyn firmware 통합을 구분합니다. H-Flow의 PAA3905E1·AFBR-S50LV85D 및 GNSS 확장은 설계안입니다.

## P0 유지와 P1 선택

P0/P1은 컴포넌트 조합의 이름입니다. 클래스 이름에는 동작을 나타내며,
composer가 ESC·Motor·Rotor 배열을 공통 기체 옆에 직접 생성하고 연결합니다.

```text
P0: PwmInput -> ESC.PassThrough -> Motors.FirstOrderSpeed
                                      -> QuadraticBlade -> MultirotorWithSensors
P1: PwmInput -> ESC.AverageVoltage -> Motors.FirstOrderVoltageToSpeed
                                      -> QuadraticBlade -> MultirotorWithSensors
```

ESC는 `Physical.Electrical.Drives.ESC`, 경험적 Motor 응답은 `Systems.Propulsion.Motors`에 둡니다.
`PassThrough`는 [0,1] 정규화 명령을 그대로 전달합니다. `AverageVoltage`는
`equivalentVoltage=clamp(escCommand,0,1)*dcVoltage`를 계산합니다.
회전수 상태는 Motor에만 하나씩 있습니다.

- `FirstOrderSpeed`: `tau*der(omega)=omegaMax*clamp(demand,0,1)-omega`.
- `FirstOrderVoltageToSpeed`: `tauOmega*der(omega)=Kv*equivalentVoltage-omega`.

공통 `QuadraticBlade`가 회전수에서 hub 힘·모멘트를 계산합니다.
`MultirotorPlant`는 `rotorForce_r[nRotors,3]`, `rotorMoment_r[nRotors,3]`를 입력받고,
로터의 상대 유입속도와 기체 각속도를 출력합니다. 장착 회전과 CG의 `r×F`는 기체가 한 번 적용합니다.
기체 내부에는 ESC·Motor·Rotor가 없으며, `MultirotorWithSensors`는 공통 센서만 추가합니다.

| 조합 | models.esc | models.motor | models.rotor |
|---|---|---|---|
| P0 기본 | `pass_through` | `first_order_speed` | `quadratic_blade` |
| P1 | `average_voltage` | `first_order_voltage_to_speed` | `quadratic_blade` |

입출력 의미가 맞지 않는 ESC/Motor 조합은 생성 전에 거부합니다.
P0 `[motor]`는 `omegaMax/tau/omega_start`, P1은 `Kv/tauOmega/omega_start`입니다.
공력 계수 `kT/kQ`는 `[rotor]`로 분리합니다. 각 필드는 scalar 또는 rotor 수만큼의 배열입니다.
P1에는 `motor.Kv`와 `[power] dcVoltage`가 필수이며, Kv 단위는 **(rad/s)/V**입니다.
rpm/V는 `2*pi/60`을 곱해 변환합니다. 예제의 16 V·62.5 (rad/s)/V는 비교용 수치입니다.
`Kv*Vnom=omegaMax`이고 시정수·초기속도가 같으면 명목 P0/P1 응답은 같습니다.
전류·소비전력·축부하 피드백·역회전·능동 제동은 포함하지 않습니다.
P1은 별도 최대속도 포화가 없으며 전압 0에서 지수적으로 감속합니다.

상위 FastDyn의 `configs/copter462_fire_p0.toml`, `configs/copter462_fire_p1.toml`을 유지합니다.
`--profile plant`는 두 조합 모두 `normalizedCommand`를 받으며 P1에만 기본값이 있는
`dcVoltage` 입력을 추가합니다. `--profile vio`는 외부 PWM·센서 IO를 유지하고 공급 전압을 내부에 둡니다.
`Examples.QuadVoltageSag`는 1초에 공급 전압을 16 V에서 12 V로 낮춥니다.

이행 시 기존 `models.rotor="speed_driven"/"voltage_driven"`을 위 세 선택자로 바꾸고,
`[motor] kT/kQ`를 `[rotor]`로 옮깁니다. 통합 rotor 모델, `ResponseModels`와
`VoltageDrivenMultirotor*`는 제거했으며 `IdealVoltage`는 `ESC.AverageVoltage`로 바뀝니다.
생성 source와 FMU를 재생성해야 합니다.

## P2 전기·기계 결합 모델

`average_unidirectional` ESC, `dynamic_dc_equivalent` motor, `quadratic_blade` rotor를
선택합니다. `configs/quad_p2.toml`, `hex_p2.toml`은 실제 모터에 보정하지 않은 예시입니다.

- `Physical.Electrical.Passives.SeriesRL`: 양방향 범용 R·L 직렬 회로입니다.
- `Physical.Mechanical.Rotational.RigidShaft`: 회전 관성과 점성 마찰을 담당합니다.
- `Systems.Propulsion.Motors.DynamicDCEquivalent`: 두 요소를 MSL `RotationalEMF`로 연결합니다.
- `Systems.Propulsion.HubReaction`: 공력 모멘트와 축 가속 반력을 조합합니다.

`motor.R/L/k/J`, `rotor.J`, `power.dcVoltage`를 명시해야 합니다. `b/i_start/omega_start`는
기본 0입니다. 모터·로터 값은 scalar 또는 로터 수만큼의 배열을 받습니다.
`rotor.J`는 모터의 `JLoad`로 전달하며 블레이드 공력 모델에는 전달하지 않습니다.
`k`는 DC 등가 SI 변환 상수이고, P1의 경험적 `Kv`를 그대로 역수 변환해서 쓰지 않습니다.
전류와 회전속도의 동역학은 `L*der(i)=Vm-R*i-k*omega`,
`(J+JLoad)*der(omega)=k*i-b*omega-loadTorque`입니다.

구동을 중단하면 양의 잔류 전류가 순환하며 감소합니다. 전류가 0에 도달한 뒤 출력단을
개방하고 단자 전압은 역기전력을 따릅니다. 역전류·능동제동·회생은 없습니다.
`esc.coast`를 `motor.currentZero`에 연결하여 실제 권선 전류의 영점 사건만 처리합니다.
구동 명령을 끈 순간 전류를 초기화하지 않습니다. `esc.openCircuitVoltage`에는
`motor.backEMF`를 연결합니다. 이 보조 신호들은 전기 Pin 연결과 함께 사용합니다.

두 export profile에 전류·공급전류·전자기토크·부하토크·입력전력·동손·마찰손실·축동력·저장에너지
배열 진단을 추가합니다. 전력 수지는 자기 에너지와 회전 에너지를 모두 포함합니다.
기체에는 공력 모멘트와 `-spinSign*(J+JLoad)*der(omega)`를 한 번 반영합니다.
기체 회전의 축 동역학 피드백과 자이로 결합은 생략한 고정 하우징 근사입니다.
PWM 리플·3상 정류·PWM 주기 내 불연속 전도·전류 제한·배터리·열 동역학은 포함하지 않습니다.

`Examples.QuadElectromechanicalCoast`에서 호버·차단·재구동을 확인할 수 있습니다.
`python3 tools/probe_p2_export.py`는 실제 quad/hexa FMI2 실행과 적분 간격 수렴을 검사합니다.
P2는 OpenModelica native 및 FMI2에서 검증했습니다. Rumoca/FastDyn FMI3는 현재 컴파일에 실패하며,
제약과 검증 범위는 [validation 문서](docs/validation.md#p2-electromechanical-validation-2026-10-12)에 기록했습니다.
새 파라미터를 사용할 때는 `L/R`에 맞춰 적분 간격을 다시 검토해야 합니다.

## VirtualIO adapter와 FMU export 계층

`Adapters.VirtualIO`의 `PwmInput`, `SensorValues`는 상태 없이 값을 변환하며 차량을 생성하지 않습니다.
독립적인 `SpiTransfer`는 이상적인 8비트 SPI 신호를 제공하며 copter export와
분리되어 있습니다. [SPI 인터페이스 초안](docs/spi_interface_ko.md)을 참고하세요.
P0/P1/P2 조합은 `Exports.FMU.CopterInterface`를 상속합니다.
`PartialCopterInterface`가 센서 포트·단위/지리 좌표 변환과 별도 `truth`/`rotorSpeed`
진단 출력을 정의하고, `CopterInterface`가 공통 PWM adapter를 연결합니다.

Composer가 `geometry.actuatorIndex`로 PWM 채널을 선택하여 `escs.escCommand`에 연결합니다.
기체는 PWM·ESC·Motor 종류와 독립적인 hub 힘·모멘트 경계를 사용합니다. 상수 대기압 101325 Pa·온도 288.15 K도
composer가 plant에 직접 전달하며 export 클래스에는 환경 공급식이 없습니다.

CLI `--profile vio`와 기존 `--profile fastdyn` 별칭은 유지합니다. 외부 PWM·센서 이름과
단위는 같고 `[interface] pwm_min/pwm_max`는 1100/1900 μs가 기본입니다.
독립 `PwmInput`의 기본 범위는 1000/2000 μs이며 원하는 calibration을 명시할 수 있습니다.

기존 `Adapters.VIO`/`Adapters.FastDyn` 경로와 standalone `MultirotorValueAdapter`,
`MultirotorFmu`는 제거했습니다. CopterInterface를 상속하던 외부 source는 `Exports.FMU`로
변경하고 staged source와 FMU를 재생성하세요. `tools/probe_export.py`도 composer로 만든
P0/P1 quad/hexa를 사용해 별도 차량 조립 없이 동일한 export 계약을 검증합니다.

## 실행

```sh
omc tools/simulate_landing_gear.mos
python3 tools/verify.py
FIRE_TEST_OMC=1 python3 -m unittest discover -s tools -p test_composer.py
python3 tools/probe_export.py
cargo run --locked --offline --manifest-path tools/fire-compose/Cargo.toml -- \
  check configs/quad.toml --profile vio
```

OpenModelica와 Modelica 4.0.0이 필요합니다. FMU probe는 Linux FMI 2.0 Co-Simulation을 검사합니다.
`Examples.QuadHover`, `HexaHover`, `OctoHover`, `CoaxialHover`는 초기 회전속도로 중력을 상쇄하는
개방 루프 예제입니다. 제어기는 포함하지 않습니다.

## 4점 착륙다리

OMEdit에서 `package.mo`를 열고 `Examples.QuadGroundCycle`을 6초간 시뮬레이션하거나,
라이브러리 루트에서 위 `omc` 명령을 실행하세요. 발끝 지상고 2 cm에서 낙하·안착한 뒤
2초에 대칭 추력을 가하고, 2.7초에 모터를 끄고 재착륙합니다.
`altitude`, `normalLoad`, `contactCount`, `acceleration_frd[3]`를 확인할 수 있습니다.
결과 CSV는 `build/landing-gear/QuadGroundCycle_res.csv`에 생성됩니다.

`vehicle.landingGear`는 `Physical.Mechanical.Chassis.LandingGear.Parameters` 레코드입니다.

| 파라미터 | 기본값 / 의미 |
|---|---|
| `enabled` | `true`; `false`이면 번역 시 접촉점 제거 |
| `position_C[4,3]` | X/Y = ±0.17 m, Z = +0.10 m; chassis C 기준 FRD 발끝 위치 |
| `groundZ` | 0 m; 수평 지면의 NED Down 좌표 |
| `stiffness`, `damping` | 발 하나당 3000 N/m, 150 N·s/m |
| `tangentialDamping`, `frictionCoefficient` | 25 N·s/m, 0.6; 수직 하중으로 제한되는 점성 마찰 |

Deprecated의 `CompliantPointLeg`, `LandingGearAssembly`, `Contact` 모델을 재사용했습니다.
기체 강체가 스프링 위의 질량을 제공하며 다리는 별도 질량·상태를 추가하지 않습니다.
실제 다리 질량은 aggregate 또는 `additionalParts`에 한 번만 포함하세요.
발끝 위치에서 `chassis.cg_C`를 빼서 CG 기준으로 보정하며, 지면의 바깥쪽 법선은 `{0,0,-1}`입니다.
접촉 시 수직력은 `max(0, k*max(0,-gap) - c*normalVelocity)`입니다.
마찰은 `mu*normalForce`로 제한되며 정지 마찰 구속은 없습니다.
접촉력과 `r × F` 모멘트를 로터·외력과 함께 한 번만 합산합니다.
접촉·이탈 이벤트는 유지하고, 속도 재설정이나 arming 조건은 사용하지 않습니다.

질량 1.5 kg, 발 하나당 강성 3000 N/m인 수평 quad의 정적 침하량은 1.226 mm,
CG 고도는 0.098774 m, FRD 가속도계 Z는 −9.80665 m/s²입니다.
기본 plant의 `p_start={0,0,-1}`은 유지하며, quad/hexa TOML은 기본 CG·장착 위치에서
발끝 지상고가 2 cm가 되도록 `p_start={0,0,-0.12}`를 명시합니다.
CG나 자세를 바꾸면 `p_tip = p_CG + R_wb*(position_C-cg_C)`로 초기 위치를 정하세요.
로터 수와 무관하게 발은 4개입니다.

Composer schema 3의 선택 항목 `[landing_gear]`에서 위 레코드 필드를 설정할 수 있습니다.

```toml
[landing_gear]
enabled = true
groundZ = 0.0
stiffness = 3000.0
damping = 150.0
tangentialDamping = 25.0
frictionCoefficient = 0.6
```

## 좌표계와 설정 이행

`R_ab`는 a에 대한 b의 자세이며 성분 변환은 `v_a = R_ab*v_b`입니다.
`q_wb={w,x,y,z}`는 같은 자세의 Hamilton quaternion입니다.
기본 기체는 기존 East heading을 보존해 `q_start={sqrt(0.5),0,0,sqrt(0.5)}`이며,
`p_start={0,0,-1}`은 지면 원점보다 1 m 위입니다. Identity quaternion은 North heading입니다.
Rotor local +z 추력축과 sensor local 축을 유지하므로 기본 장착 행렬은 `diag(1,-1,-1)`입니다.
Body FRD에 정렬된 IMU가 필요하면 `R_bImu=identity(3)`을 지정합니다.

TOML은 **schema_version=3**만 허용합니다. 기존 schema 1/2, 옛 `[ground]` 설정과 `geometry.nLegs`, sample period,
`first_order` sensor response는 거부합니다. 기존 FastDyn host TOML의 내장 physics table도
같은 이행이 필요합니다. Schema 숫자만 바꾸지 말고 위치·벡터·관성·장착·초기 자세를 함께 변환하세요.
변환식과 포트는 [architecture](docs/architecture.md)에 있습니다.

이전 fixed wing·rover·지형·sampling·필터·통신 경로 및 ENU/FLU 착륙다리 원본은
[deprecated](deprecated/README.md)에 보존합니다. 이전 클래스와 새 NED/FRD 클래스를
같은 namespace로 함께 로드하지 마세요. 이전 검증 기록은 새 모델의 검증 결과가 아닙니다.

[검증 범위](docs/validation.md), [센서 계약](Systems/Sensing/README.md),
[조합 도구](tools/fire-compose/README.md)를 참고하세요.
P1 확장에서는 상위 FastDyn 통합 테스트로 Rumoca FMI 3 packaging과 실제 Co-Simulation stepping을 검증합니다.
RBC 연결과 실제 firmware/비행 검증은 해당 테스트 범위 밖입니다.

`plant`와 `vio` 생성 모델 모두 공통 기체를 `plant` 컴포넌트로 포함합니다. `plant` 프로파일은
환경 입력과 truth·measurements·합산 wrench 진단을 전달하며 내부 hub wrench를 외부 입력으로 노출하지 않습니다.
기체를 수정할 때는 `plant(...)` 아래에 modifier를 지정합니다.

## P3·P4 3상 추진계

P3는 6-step 평균화 인버터, P4는 6개 스위치의 PWM 인버터를 사용합니다. 부동 중성점의 `ThreePhasePM` 모터와 기체 공통 `DCLink`를 공유합니다. 수식·파라미터 정의·coast 정책·configuration·검증 범위는 [추진계 모델링 문서](docs/propulsion_modeling_ko.md)에 정리했습니다. 예제는 `configs/quad_p3.toml`, `quad_p4.toml`, `hex_p3.toml`, `hex_p4.toml`입니다. 회전자 위치를 아는 6-step 모델이며 FOC는 포함하지 않습니다. Rumoca/FastDyn FMI3 실행을 검증한 결과는 아닙니다.
