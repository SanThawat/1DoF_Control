/**
 * @file    scurve_trajectory.h
 * @brief   S-Curve Trajectory Generator for 1-DOF Robot Arm
 *          Target: STM32 NUCLEO-G474RE
 *
 * S-Curve มี 7 phase:
 *   Phase 1: Jerk = +J_max  → accel เพิ่ม
 *   Phase 2: Jerk = 0       → accel คงที่ (A_max)
 *   Phase 3: Jerk = -J_max  → accel ลด → vel ถึง V_max
 *   Phase 4: Jerk = 0       → vel คงที่ (cruising)
 *   Phase 5: Jerk = -J_max  → decel เริ่ม
 *   Phase 6: Jerk = 0       → decel คงที่
 *   Phase 7: Jerk = +J_max  → decel ลด → หยุด
 *
 * Output → เข้า Cascade Position/Velocity Controller
 */

#ifndef SCURVE_TRAJECTORY_H
#define SCURVE_TRAJECTORY_H

#include <stdint.h>
#include <stdbool.h>

/* ─── คอนฟิก Trajectory ──────────────────────────────────────────────────── */

/** ความถี่ของ trajectory generator (Hz) → ควรเท่ากับ control loop */
#define TRAJ_SAMPLE_RATE_HZ     1000.0f
#define TRAJ_DT                 (1.0f / TRAJ_SAMPLE_RATE_HZ)   /* 1 ms */

/** ลิสต์เป้าหมาย (องศา) */
#define WAYPOINT_COUNT          6
extern const float WAYPOINTS_DEG[WAYPOINT_COUNT];

/* ─── พารามิเตอร์ S-Curve (ปรับตามมอเตอร์จริง) ─────────────────────────── */
#define SCURVE_V_MAX_DEG_S      120.0f    /* ความเร็วสูงสุด   [deg/s]     */
#define SCURVE_A_MAX_DEG_S2     200.0f    /* acceleration สูงสุด [deg/s²]  */
#define SCURVE_J_MAX_DEG_S3     600.0f    /* jerk สูงสุด      [deg/s³]    */

/* ─── State ของ Trajectory ───────────────────────────────────────────────── */
typedef enum {
    TRAJ_IDLE = 0,      /* ยังไม่เริ่ม / หยุดนิ่ง */
    TRAJ_PHASE1,        /* Jerk +  : accel เพิ่ม   */
    TRAJ_PHASE2,        /* Jerk 0  : accel คงที่   */
    TRAJ_PHASE3,        /* Jerk -  : accel ลด      */
    TRAJ_PHASE4,        /* cruise  : vel คงที่      */
    TRAJ_PHASE5,        /* Jerk -  : decel เริ่ม   */
    TRAJ_PHASE6,        /* Jerk 0  : decel คงที่   */
    TRAJ_PHASE7,        /* Jerk +  : decel ลด      */
    TRAJ_DONE           /* ถึงเป้าหมาย             */
} TrajPhase_t;

/* ─── ผลลัพธ์ที่ส่งเข้า Control Block ───────────────────────────────────── */
typedef struct {
    float position_deg;     /* ref position [deg]  → เข้า position loop */
    float velocity_deg_s;   /* ref velocity [deg/s]→ เข้า velocity loop (feedforward) */
    float accel_deg_s2;     /* ref accel    [deg/s²] (optional, สำหรับ accel FF) */
} TrajOutput_t;

/* ─── Context ของ Trajectory Generator ──────────────────────────────────── */
typedef struct {
    /* เป้าหมาย */
    float q_start;          /* ตำแหน่งเริ่ม [deg] */
    float q_end;            /* ตำแหน่งเป้า  [deg] */
    float direction;        /* +1.0 หรือ -1.0      */
    float total_disp;       /* |q_end - q_start|   */

    /* parameter ที่คำนวณได้ */
    float v_max_eff;        /* V_max ที่ใช้จริง (อาจน้อยกว่า SCURVE_V_MAX_DEG_S) */
    float a_max_eff;        /* A_max ที่ใช้จริง */
    float j_max;            /* J_max             */

    /* ระยะเวลาแต่ละ phase [s] */
    float t1, t2, t3;       /* phase 1,2,3 (accel zone) */
    float t4;               /* phase 4 cruise           */
    float t5, t6, t7;       /* phase 5,6,7 (decel zone) */

    /* state ปัจจุบัน */
    TrajPhase_t phase;
    float t_phase;          /* เวลาใน phase ปัจจุบัน [s] */

    /* ค่า ณ ต้น phase */
    float pos_phase_start;
    float vel_phase_start;
    float acc_phase_start;

    /* output */
    TrajOutput_t out;

    /* waypoint management */
    uint8_t wp_index;       /* waypoint ถัดไปที่จะไป */
    bool    active;
} SCurveTraj_t;

/* ─── API Functions ───────────────────────────────────────────────────────── */

/**
 * @brief  เริ่มต้น trajectory context ทั้งหมด
 * @param  traj  pointer ไปยัง SCurveTraj_t
 */
void SCurve_Init(SCurveTraj_t *traj);

/**
 * @brief  กำหนด segment ใหม่ (จาก q_start → q_end)
 *         เรียกเมื่อต้องการออกไปยัง waypoint ถัดไป
 * @param  traj     pointer ไปยัง SCurveTraj_t
 * @param  q_start  ตำแหน่งเริ่มต้น [deg]
 * @param  q_end    ตำแหน่งเป้าหมาย [deg]
 */
void SCurve_SetSegment(SCurveTraj_t *traj, float q_start, float q_end);

/**
 * @brief  เรียกทุก control loop (ทุก TRAJ_DT วินาที)
 *         อัปเดต position/velocity reference
 * @param  traj   pointer ไปยัง SCurveTraj_t
 * @retval TrajOutput_t  ค่า ref ที่ส่งเข้า control block
 */
TrajOutput_t SCurve_Update(SCurveTraj_t *traj);

/**
 * @brief  ตรวจสอบว่า segment ปัจจุบันเสร็จหรือยัง
 */
bool SCurve_IsDone(const SCurveTraj_t *traj);

/**
 * @brief  วิ่งผ่านทุก waypoint ตาม WAYPOINTS_DEG[]
 *         เรียกใน main loop — จะ auto-advance waypoint เมื่อถึงเป้า
 * @param  traj          pointer ไปยัง SCurveTraj_t
 * @param  current_pos   ตำแหน่งจริงปัจจุบัน [deg] (จาก encoder)
 * @retval TrajOutput_t  ค่า ref
 */
TrajOutput_t SCurve_RunWaypoints(SCurveTraj_t *traj, float current_pos);

#endif /* SCURVE_TRAJECTORY_H */
