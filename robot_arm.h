/**
 * @file    robot_arm.h
 * @brief   1-DOF Robot Arm Library
 *          STM32 NUCLEO-G474RE
 *
 * Architecture:
 *   S-Curve Trajectory → Cascade PID (outer=pos, inner=vel) + Velocity FF
 *   All control runs in TIM7 interrupt @ 1 kHz
 *
 * API (caller side):
 *   RobotArm_Init()
 *   RobotArm_Move(float degrees, float direction)   direction = +1.0f / -1.0f
 *   RobotArm_IsDone()
 *   RobotArm_Stop()
 *
 * Dependencies:
 *   scurve_trajectory.h / scurve_trajectory.c
 *   HAL (TIM1, TIM2, TIM7, GPIOA)
 */

#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <stdint.h>
#include <stdbool.h>
#include "scurve_trajectory.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  HARDWARE CONFIG
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ARM_PULSES_PER_REV      8192        /* Encoder counts per revolution   */
#define ARM_TIM_PWM_PERIOD      999         /* TIM1 ARR  (0–999 duty)          */
#define ARM_PWM_MAX             999.0f      /* Max PWM command (±)             */
#define ARM_PWM_DEADBAND   10.0f

/* ═══════════════════════════════════════════════════════════════════════════
 *  OUTER LOOP — Position PID
 *  Output = velocity setpoint [deg/s]
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ARM_POS_KP              1.5f        /* proportional gain               */
#define ARM_POS_KI              0.00f        /* integral gain                   */
#define ARM_POS_KD              0.0f       /* derivative gain                 */
#define ARM_POS_I_LIMIT         50.0f       /* anti-windup clamp [deg/s]       */
#define ARM_POS_OUT_LIMIT       SCURVE_V_MAX_DEG_S  /* clamp to v_max [deg/s] */

/* ═══════════════════════════════════════════════════════════════════════════
 *  INNER LOOP — Velocity PID
 *  Output = PWM command (–999 … +999)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ARM_VEL_KP              2.0f        /* proportional gain               */
#define ARM_VEL_KI              0.001f        /* integral gain                   */
#define ARM_VEL_KD              0.00f        /* derivative gain                 */
#define ARM_VEL_I_LIMIT         200.0f      /* anti-windup clamp [PWM units]   */
#define ARM_VEL_OUT_LIMIT       ARM_PWM_MAX /* clamp PWM output                */

/* ═══════════════════════════════════════════════════════════════════════════
 *  FEEDFORWARD — Velocity
 *  FF = KFF_V * vel_ref  [PWM units / (deg/s)]
 *  Tune so that at v_max the FF alone gives ~60–70% duty
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ARM_KFF_V               2.5f        /* velocity feed-forward gain      */

/* ═══════════════════════════════════════════════════════════════════════════
 *  DONE THRESHOLD
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ARM_DONE_TOL_DEG        5.000f        /* position tolerance to call Done */
#define ARM_DONE_TOL_VEL        5.000f        /* velocity tolerance [deg/s]      */

/* ═══════════════════════════════════════════════════════════════════════════
 *  INTERNAL STRUCTS
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    float integral;
    float prev_error;
} PIDState_t;

typedef struct {
    /* S-Curve trajectory */
    SCurveTraj_t    traj;

    /* Current kinematic state (encoder-derived) */
    float           pos_deg;        /* current position [deg]   */
    float           vel_deg_s;      /* current velocity [deg/s] */
    int32_t         encoder_raw;    /* accumulated encoder count */
    int32_t         encoder_last;   /* last TIM2 counter value  */

    /* PID states */
    PIDState_t      pos_pid;
    PIDState_t      vel_pid;

    /* Trajectory reference (from S-Curve) */
    float           ref_pos_deg;
    float           ref_vel_deg_s;

    /* Status */
    bool            running;
    bool            done;
} RobotArm_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise library. Call once in main() before starting TIM7.
 *         Starts TIM1 (PWM) and TIM2 (encoder) internally.
 */
void RobotArm_Init(RobotArm_t *arm);

/**
 * @brief  Command a relative move.
 * @param  degrees    Magnitude of movement [deg], must be > 0
 * @param  direction  +1.0f = positive encoder direction, -1.0f = reverse
 */
void RobotArm_Move(RobotArm_t *arm, float degrees, float direction);

/**
 * @brief  Returns true when trajectory is complete and arm has settled.
 */
bool RobotArm_IsDone(const RobotArm_t *arm);

/**
 * @brief  Immediately stop motion and hold position.
 */
void RobotArm_Stop(RobotArm_t *arm);

/**
 * @brief  1 kHz control ISR body. Call from TIM7 IRQHandler.
 *         Example:
 *           void TIM7_IRQHandler(void) {
 *               HAL_TIM_IRQHandler(&htim7);
 *           }
 *           void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *               if (htim->Instance == TIM7) RobotArm_ControlTick(&g_arm);
 *           }
 */
void RobotArm_ControlTick(RobotArm_t *arm);

#endif /* ROBOT_ARM_H */
