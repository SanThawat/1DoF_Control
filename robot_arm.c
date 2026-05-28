/**
 * @file    robot_arm.c
 * @brief   1-DOF Robot Arm Library — Implementation
 *          STM32 NUCLEO-G474RE
 *
 * Control loop (runs every 1 ms in TIM7 ISR):
 *
 *   S-Curve  ──► pos_ref, vel_ref
 *                  │
 *   [Outer PID]  pos_error = pos_ref − pos_meas  → vel_cmd
 *                  │
 *   vel_setpoint = vel_cmd + KFF_V * vel_ref    ← velocity feed-forward
 *                  │
 *   [Inner PID]  vel_error = vel_setpoint − vel_meas  → pwm_cmd
 *                  │
 *   Motor driver  DIR pin + PWM compare
 */

#include "robot_arm.h"
#include "main.h"      /* HAL, htim1, htim2, htim7, GPIO defines */
#include <math.h>
#include <string.h>

/* ─── External HAL handles (defined in main.c) ─────────────────────────── */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;

/* ─── Helpers ───────────────────────────────────────────────────────────── */
#define DEG_TO_COUNTS(d)  ((int32_t)((d) * ARM_PULSES_PER_REV / 360.0f))
#define COUNTS_TO_DEG(c)  ((float)(c) * 360.0f / (float)ARM_PULSES_PER_REV)
#define CLAMPF(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define TIM2_PERIOD       65535

/* ─── Private: PID compute ──────────────────────────────────────────────── */
static float _PID_Compute(PIDState_t *pid,
                           float       error,
                           float       kp,
                           float       ki,
                           float       kd,
                           float       i_limit,
                           float       out_limit,
                           float       dt)
{
    /* P */
    float P = kp * error;

    /* I + anti-windup clamp */
    pid->integral += error * dt;
    pid->integral  = CLAMPF(pid->integral, -i_limit, i_limit);
    float I = ki * pid->integral;

    /* D */
    float D = kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return CLAMPF(P + I + D, -out_limit, out_limit);
}

/* ─── Private: read encoder, update pos & vel ───────────────────────────── */
static void _Encoder_Update(RobotArm_t *arm, float dt)
{
    int32_t raw  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t diff = raw - arm->encoder_last;

    if (diff >  (TIM2_PERIOD / 2)) diff -= (TIM2_PERIOD + 1);
    if (diff < -(TIM2_PERIOD / 2)) diff += (TIM2_PERIOD + 1);

    arm->encoder_last  = raw;
    arm->encoder_raw  += diff;
    arm->pos_deg       = COUNTS_TO_DEG(arm->encoder_raw);

    /* LPF velocity  α=0.1 → smooth,  α=0.3 → เร็วขึ้นแต่ noisy กว่า */
    float vel_raw      = COUNTS_TO_DEG(diff) / dt;
    arm->vel_deg_s     = 0.1f * vel_raw + 0.9f * arm->vel_deg_s;
}

/* ─── Private: set motor PWM + direction ────────────────────────────────── */
static void _Motor_SetPWM(float pwm_cmd)
{
    pwm_cmd = CLAMPF(pwm_cmd, -ARM_PWM_MAX, ARM_PWM_MAX);

    /* Deadband compensation */
    if (pwm_cmd > 1.0f)
        pwm_cmd = ARM_PWM_DEADBAND + pwm_cmd * (ARM_PWM_MAX - ARM_PWM_DEADBAND) / ARM_PWM_MAX;
    else if (pwm_cmd < -1.0f)
        pwm_cmd = -(ARM_PWM_DEADBAND + (-pwm_cmd) * (ARM_PWM_MAX - ARM_PWM_DEADBAND) / ARM_PWM_MAX);
    else
        pwm_cmd = 0.0f;   /* dead zone เล็กๆ รอบ 0 ให้ motor หยุดสนิท */

#if ARM_MOTOR_DIR_INVERT
    pwm_cmd = -pwm_cmd;
#endif

    if (pwm_cmd >= 0.0f) {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
        pwm_cmd = -pwm_cmd;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pwm_cmd);
}

/* ─── Private: reset PID states ─────────────────────────────────────────── */
static void _PID_Reset(PIDState_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

void RobotArm_Init(RobotArm_t *arm)
{
    memset(arm, 0, sizeof(RobotArm_t));

    SCurve_Init(&arm->traj);

    /* Start peripherals */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* Motor off */
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

    /* Snapshot encoder start */
    arm->encoder_last = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

    /* Start 1 kHz control interrupt */
    HAL_TIM_Base_Start_IT(&htim7);

    arm->running = false;
    arm->done    = true;
}

/* ─────────────────────────────────────────────────────────────────────────
 *  RobotArm_Move
 *  degrees   : distance to travel (> 0)
 *  direction : +1.0f or -1.0f
 * ───────────────────────────────────────────────────────────────────────── */
void RobotArm_Move(RobotArm_t *arm, float degrees, float direction)
{
    if (degrees <= 0.0f) return;

    /* Snap direction to ±1 */
    float dir = (direction >= 0.0f) ? 1.0f : -1.0f;

    float q_start = arm->pos_deg;
    float q_end   = q_start + dir * degrees;

    _PID_Reset(&arm->pos_pid);
    _PID_Reset(&arm->vel_pid);

    SCurve_SetSegment(&arm->traj, q_start, q_end);

    arm->running = true;
    arm->done    = false;
}

/* ─────────────────────────────────────────────────────────────────────────
 *  RobotArm_IsDone
 * ───────────────────────────────────────────────────────────────────────── */
bool RobotArm_IsDone(const RobotArm_t *arm)
{
    return arm->done;
}

/* ─────────────────────────────────────────────────────────────────────────
 *  RobotArm_Stop
 * ───────────────────────────────────────────────────────────────────────── */
void RobotArm_Stop(RobotArm_t *arm)
{
    arm->running = false;
    arm->done    = true;
    _PID_Reset(&arm->pos_pid);
    _PID_Reset(&arm->vel_pid);
    _Motor_SetPWM(0.0f);

    /* Freeze trajectory at current position */
    SCurve_SetSegment(&arm->traj, arm->pos_deg, arm->pos_deg);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  RobotArm_ControlTick  — called from TIM7 ISR @ 1 kHz
 *
 *  Data flow:
 *    1. Read encoder  → pos_meas, vel_meas
 *    2. Step S-Curve  → pos_ref, vel_ref
 *    3. Outer PID     → vel_cmd   (pos error → velocity setpoint)
 *    4. Velocity FF   → vel_setpoint = vel_cmd + KFF_V * vel_ref
 *    5. Inner PID     → pwm_cmd   (vel error → PWM)
 *    6. Drive motor
 *    7. Check done
 * ═══════════════════════════════════════════════════════════════════════════ */
void RobotArm_ControlTick(RobotArm_t *arm)
{
    const float dt = TRAJ_DT;   /* 0.001 s */

    /* ── 1. Encoder ─────────────────────────────────────────────────────── */
    _Encoder_Update(arm, dt);

    if (!arm->running) {
        /* Hold: keep motor off, nothing else to do */
        _Motor_SetPWM(0.0f);
        return;
    }

    /* ── 2. S-Curve step ────────────────────────────────────────────────── */
    TrajOutput_t ref = SCurve_Update(&arm->traj);
    arm->ref_pos_deg   = ref.position_deg;
    arm->ref_vel_deg_s = ref.velocity_deg_s;

    /* ── 3. Outer PID — position → velocity command ─────────────────────── */
    float pos_error = arm->ref_pos_deg - arm->pos_deg;
    float vel_cmd   = _PID_Compute(&arm->pos_pid,
                                    pos_error,
                                    ARM_POS_KP,
                                    ARM_POS_KI,
                                    ARM_POS_KD,
                                    ARM_POS_I_LIMIT,
                                    ARM_POS_OUT_LIMIT,
                                    dt);

    /* ── 4. Velocity feed-forward ────────────────────────────────────────── */
    float vel_setpoint = vel_cmd + ARM_KFF_V * arm->ref_vel_deg_s;

    /* ── 5. Inner PID — velocity → PWM ──────────────────────────────────── */
    float vel_error = vel_setpoint - arm->vel_deg_s;
    float pwm_cmd   = _PID_Compute(&arm->vel_pid,
                                    vel_error,
                                    ARM_VEL_KP,
                                    ARM_VEL_KI,
                                    ARM_VEL_KD,
                                    ARM_VEL_I_LIMIT,
                                    ARM_VEL_OUT_LIMIT,
                                    dt);

    /* ── 6. Drive motor ──────────────────────────────────────────────────── */
    _Motor_SetPWM(pwm_cmd);

    /* ── 7. Done check ───────────────────────────────────────────────────── */
    if (SCurve_IsDone(&arm->traj)) {
        float pos_err = fabsf(arm->traj.q_end - arm->pos_deg);
        float vel_mag = fabsf(arm->vel_deg_s);

        if (pos_err <= ARM_DONE_TOL_DEG && vel_mag <= ARM_DONE_TOL_VEL) {
            arm->running = false;
            arm->done    = true;
            _Motor_SetPWM(0.0f);
        }
    }
}
