# EtherCAT Controller

`transped_mujoco`의 Controller 형식(`sensor_callback` → `command` → `send_commands_to_robot`)으로
실제 EtherCAT 모터를 제어하는 프로젝트. MuJoCo `Daemon` 자리에 `GVdrive`(EtherCAT 1 kHz RT thread)가 들어간다.

```text
config/motors.yaml       모터 등록 (slave_index, type, control_mode, 드라이브 p/d_gain)
config/controller.yaml   제어 주기, homing, torque 모드 kp/kd/tau_max, safety
include/Controller/      Controller (INIT -> HOMING -> FINISH)
                         + TrajectoryGenerator: transped 와 같은 cosine 보간 (DoF 가변)
src/main.cpp             GVdrive <-> Controller 연결 루프 (Daemon + sensor/JointControl thread 역할)
include/MotorController/  EtherCAT_MotorControl/motor 복사 (GVdrive)
include/soem/            SOEM (EtherCAT master)
```

## 명령 프로토콜 (관절별, motors.yaml `control_mode`)

| control_mode | 전송 값 | PD 계산 위치 | 게인 |
|---|---|---|---|
| `impedance` | q_ref (+ τ_ff=0) | 드라이브 | motors.yaml `p_gain`/`d_gain` |
| `position` | q_ref | 드라이브 (CSP) | 드라이브 내부 |
| `velocity` | qd_ref | 드라이브 (CSV) | 드라이브 내부 |
| `torque` | kp·(q_ref−q) + kd·(qd_ref−qd), ±tau_max | PC | controller.yaml `torque.kp/kd` |

## 실행

```bash
IFACE=enp4s0 ./run.sh
```

Ctrl+C → FINISH → 드라이브 disable. 추종 오차가 `safety.max_tracking_error`를 넘어도 FINISH.

## 새 동작 추가

`include/Controller/controller.cpp`의 `Controller::command()`에 상태를 추가하고 `ref_q_`, `ref_dq_`를 채우면 된다.
torque 관절의 토크는 `command()` 끝에서 공통으로 계산된다.
