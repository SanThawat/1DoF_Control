/**
 * @file    scurve_trajectory.h
 * @brief   S-Curve Trajectory Generator — Pure Kinematic
 *          STM32 NUCLEO-G474RE | 1-DOF Robot Arm
 */

#ifndef SCURVE_TRAJECTORY_H
#define SCURVE_TRAJECTORY_H

#include <stdint.h>
#include <stdbool.h>

/* ─── Sample rate ────────────────────────────────────────────────────────── */
#define TRAJ_SAMPLE_RATE_HZ     1000.0f
#define TRAJ_DT                 (1.0f / TRAJ_SAMPLE_RATE_HZ)   /* 1 ms */

/* ─── Waypoints ──────────────────────────────────────────────────────────── */
#define WAYPOINT_COUNT          6
extern const float WAYPOINTS_DEG[WAYPOINT_COUNT];

/* ─── S-Curve parameters (ปรับตามมอเตอร์จริง) ───────────────────────────── */
#define SCURVE_V_MAX_DEG_S      1000.0f    /* deg/s   */
#define SCURVE_A_MAX_DEG_S2     1000.0f    /* deg/s²  */
#define SCURVE_J_MAX_DEG_S3     300.0f    /* deg/s³  */

/* ─── Phase enum ─────────────────────────────────────────────────────────── */
typedef enum {
    TRAJ_IDLE = 0,
    TRAJ_PHASE1,    /* Jerk+  : accel เพิ่ม   */
    TRAJ_PHASE2,    /* Accel  : accel คงที่   */
    TRAJ_PHASE3,    /* Jerk-  : accel ลด      */
    TRAJ_PHASE4,    /* Cruise : vel คงที่      */
    TRAJ_PHASE5,    /* Jerk-  : decel เริ่ม   */
    TRAJ_PHASE6,    /* Decel  : decel คงที่   */
    TRAJ_PHASE7,    /* Jerk+  : decel ลด      */
    TRAJ_DONE
} TrajPhase_t;

/* ─── Output ที่ส่งเข้า Control Block ───────────────────────────────────── */
typedef struct {
    float position_deg;     /* ref position [deg]   */
    float velocity_deg_s;   /* ref velocity [deg/s] */
    float accel_deg_s2;     /* ref accel    [deg/s²]*/
} TrajOutput_t;

/* ─── Initial Condition ของแต่ละ phase (คำนวณไว้ล่วงหน้า) ─────────────── */
typedef struct {
    float p;   /* position offset จาก q_start ณ ต้น phase [deg] */
    float v;   /* velocity ณ ต้น phase [deg/s]                   */
} PhaseIC_t;

/* ─── Trajectory Context ─────────────────────────────────────────────────── */
typedef struct {
    /* segment */
    float q_start;
    float q_end;
    float direction;        /* +1.0 หรือ -1.0 */
    float total_disp;       /* |q_end - q_start| */

    /* parameters */
    float v_max_eff;
    float a_max_eff;
    float j_max;

    /* phase durations [s] */
    float t1, t2, t3, t4, t5, t6, t7;

    /* boundary conditions ของแต่ละ phase (pre-computed) */
    PhaseIC_t ic[7];        /* ic[0]=phase1, ic[1]=phase2, ... ic[6]=phase7 */

    /* state */
    TrajPhase_t phase;
    float       t_phase;    /* เวลาใน phase ปัจจุบัน [s] */

    /* output */
    TrajOutput_t out;

    /* waypoint */
    uint8_t wp_index;
    bool    active;
} SCurveTraj_t;

/* ─── API ────────────────────────────────────────────────────────────────── */
void         SCurve_Init        (SCurveTraj_t *traj);
void         SCurve_SetSegment  (SCurveTraj_t *traj, float q_start, float q_end);
TrajOutput_t SCurve_Update      (SCurveTraj_t *traj);
bool         SCurve_IsDone      (const SCurveTraj_t *traj);
TrajOutput_t SCurve_RunWaypoints(SCurveTraj_t *traj, float current_pos);

#endif /* SCURVE_TRAJECTORY_H */
