# 🦾 Robot Arm 1-DOF — STM32 NUCLEO-G474RE

ระบบควบคุมแขนกล 1 แกนหมุน (1-DOF) ด้วย Brush DC Motor 24V บน STM32G474RE  
ใช้ S-Curve Trajectory + Cascade PID (position → velocity) + Kalman Filter 4-state  
ควบคุมที่ความถี่ **1 kHz** ผ่าน TIM7 ISR

---

## Hardware

| ชิ้นส่วน | รายละเอียด |
|---|---|
| MCU | STM32 NUCLEO-G474RE (170 MHz) |
| Motor | Brush DC 24 V |
| Motor Driver | Cyton MD20A (DIR + PWM) |
| Encoder | Quadrature, 8192 counts/rev (X4 mode) |
| Power Supply | 24 V |

### การเชื่อมต่อขา

| สัญญาณ | Pin | Timer/Peripheral |
|---|---|---|
| PWM Output | TIM1 CH1 | PA8 |
| Motor DIR | PB4 | GPIO Output |
| Encoder A/B | TIM2 CH1/CH2 | PA0, PA1 |
| Control ISR | TIM7 | 1 kHz interrupt |
| UART Log | LPUART1 | PA2 (TX), PA3 (RX) |
| User Button | PC13 (B1) | EXTI interrupt |

---

## Software Architecture

```
Waypoints [deg]
     │
     ▼
┌─────────────────────────┐
│   S-Curve Generator     │  ← 7-phase trajectory, pure kinematic
│   pos_ref / vel_ref     │    ไม่มี Euler integration → ไม่มี drift
└────────────┬────────────┘
             │ pos_ref, vel_ref
             ▼
┌─────────────────────────┐
│   Outer PID (Position)  │  ← output = vel_cmd [deg/s]
│   Zone switch ≤10 deg   │    fine-zone gain switching ใกล้เป้า
└────────────┬────────────┘
             │ vel_setpoint
             ▼
┌─────────────────────────┐
│   Inner PID (Velocity)  │  ← output = pwm_pid
│ + Velocity Feedforward  │    FF = (KE·ω + B·ω) / eff
└────────────┬────────────┘
             │ pwm_cmd
             ▼
┌─────────────────────────┐
│   Motor Driver MD20A    │  ← DIR pin + TIM1 PWM compare
│   Brush DC 24 V         │
└────────────┬────────────┘
             │ encoder counts
             ▼
┌─────────────────────────┐
│   Kalman Filter 4-state │  ← [θ, ω, i, τ_disturbance]
│   Predict + Update      │    Exact discretization (Van Loan's)
└─────────────────────────┘
             │ pos_deg, vel_deg_s (filtered)
             └──────────────► feedback → Outer PID
```

> ทุก block ทำงานใน `RobotArm_ControlTick()` ที่ถูก call จาก TIM7 ISR @ 1 kHz

---

## File Structure

```
├── main.c / main.h          — Entry point, HAL init, TIM config, UART logging
├── robot_arm.c / .h         — Library หลัก: API, Control Loop, PID, Motor driver
├── scurve_trajectory.c / .h — S-Curve 7-phase trajectory generator
└── kalman.c / .h            — 4-State Discrete Kalman Filter
```

---

## S-Curve Trajectory

เคลื่อนที่ 7 phase เพื่อจำกัด jerk (อัตราเปลี่ยน acceleration):

| Phase | Jerk | คำอธิบาย |
|---|---|---|
| 1 | +J_max | Acceleration เพิ่ม |
| 2 | 0 | Acceleration คงที่ (A_max) |
| 3 | −J_max | Acceleration ลด |
| 4 | 0 | Cruise (ความเร็วคงที่) |
| 5 | −J_max | Deceleration เริ่ม |
| 6 | 0 | Deceleration คงที่ |
| 7 | +J_max | Deceleration ลด → หยุด |

สำหรับระยะสั้นที่ไม่ถึง v_max จะ bisect หา v_peak ที่เหมาะสมอัตโนมัติ

**พารามิเตอร์ปัจจุบัน:**

```c
#define SCURVE_V_MAX_DEG_S    350.0f   // deg/s
#define SCURVE_A_MAX_DEG_S2   3500.0f  // deg/s²
#define SCURVE_J_MAX_DEG_S3   1000.0f  // deg/s³
```

---

## Kalman Filter

State vector: `x = [θ (rad), ω (rad/s), i (A), τ_disturbance (N·m)]`

- **Predict** ทุก tick ด้วย `u_volt` จาก PWM รอบก่อน
- **Update** ด้วย encoder position (แปลงจาก deg → rad)
- Matrix `Ad`, `Bd`, `Qd` คำนวณด้วย Van Loan's Method ที่ Ts = 0.001 s
- State `τ_disturbance` ช่วยรับมือกับ load ที่เปลี่ยนแปลงได้

---

## PID Gains

### Normal Zone (ระยะ > 10 deg)

| Loop | Kp | Ki | Kd |
|---|---|---|---|
| Position (outer) | 1.0 | 0.0001 | 0.0 |
| Velocity (inner) | 2.0 | 0.0001 | 0.0 |

### Fine Zone (ระยะ ≤ 10 deg)

| Loop | Kp | Ki | Kd |
|---|---|---|---|
| Position (outer) | 1.3 | 0.0 | 0.0 |
| Velocity (inner) | 1.4 | 0.0 | 0.0 |

เมื่อสลับ zone จะ reset integral ทั้งสอง loop เพื่อป้องกัน windup

**Done threshold:** `pos_error ≤ 0.3 deg` และ `|vel| ≤ 0.5 deg/s`

---

## API

```c
// เริ่มต้นระบบ (เรียกครั้งเดียวใน main)
void RobotArm_Init(RobotArm_t *arm);

// สั่งเคลื่อนที่ (relative move)
// degrees    : ระยะทาง [deg] > 0
// direction  : +1.0f หรือ -1.0f
void RobotArm_Move(RobotArm_t *arm, float degrees, float direction);

// ตรวจสอบว่าถึงเป้าและหยุดนิ่งแล้ว
bool RobotArm_IsDone(const RobotArm_t *arm);

// หยุดทันที
void RobotArm_Stop(RobotArm_t *arm);

// Control loop body — เรียกจาก TIM7 ISR ทุก 1 ms
void RobotArm_ControlTick(RobotArm_t *arm);
```

### ตัวอย่างการใช้งาน

```c
// ISR — ใน stm32g4xx_it.c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM7)
        RobotArm_ControlTick(&g_arm);
}

// สั่งหมุน 90 องศา ทิศบวก
RobotArm_Move(&g_arm, 90.0f, +1.0f);

// รอจนเสร็จ (polling)
while (!RobotArm_IsDone(&g_arm));
```

---

## UART Logging

ส่งข้อมูลออก LPUART1 ที่ **115200 baud**, **100 Hz** รูปแบบ binary frame:

```
[0xAA][0xBB][pos_deg: 4B float][ref_pos_deg: 4B float][vel_deg_s: 4B float][ref_vel_deg_s: 4B float]
 ──── header ────  ────────────────────── 16 bytes data ──────────────────────────────────────────
```

รวม 18 bytes ต่อ frame ใช้ดู real-time ผ่าน Serial plotter หรือ Python script

---

## Waypoints เริ่มต้น

```c
const float WAYPOINTS_DEG[6] = {10.0f, 45.0f, 60.0f, 90.0f, 180.0f, 360.0f};
```

เรียก `SCurve_RunWaypoints()` เพื่อให้วิ่งผ่านทุก waypoint ตามลำดับโดยอัตโนมัติ

---

## การ Build

1. เปิด project ใน **STM32CubeIDE**
2. ใส่ไฟล์ `robot_arm.c/.h`, `scurve_trajectory.c/.h`, `kalman.c/.h` เข้า project
3. Build & Flash ผ่าน ST-Link บน NUCLEO board
4. กดปุ่ม **B1 (PC13)** เพื่อสั่ง move ตามค่า `tar` และ `d` ที่ตั้งไว้ใน `main.c`

---

## License

สำหรับ FRA233 Lab 4 — FIBO, KMUTT
