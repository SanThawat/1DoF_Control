/**
 * @file    scurve_trajectory.c
 * @brief   S-Curve Trajectory Generator — Implementation
 *          STM32 NUCLEO-G474RE | 1-DOF Robot Arm
 *
 * ───────────────────────────────────────────────────────────────────────────
 *  S-Curve 7-Phase Model
 *
 *  accel
 *   ^
 *   |  ___________
 *   | /           \
 *   |/             \__________  ← A_max (phase 2)
 *   |                         \
 *   |          [phase 4]       \___________
 *   |                                      \           /
 *   |                                       \         /
 *   |                          -A_max →      \_______/
 *   +----------------------------------------------------→ t
 *     P1  P2      P3    P4     P5   P6      P7
 *
 *  velocity (integral ของ accel) จะได้รูป S
 *
 * ───────────────────────────────────────────────────────────────────────────
 *  คณิตศาสตร์หลัก
 *
 *  กำหนด:  J = jerk, A = accel, V = vel, P = pos
 *
 *  Phase 1 (0 ≤ τ ≤ t1):  J = +J_max
 *    a(τ) = J·τ
 *    v(τ) = v0 + ½·J·τ²
 *    p(τ) = p0 + v0·τ + ⅙·J·τ³
 *
 *  Phase 2 (0 ≤ τ ≤ t2):  J = 0, A = A_max
 *    a(τ) = A_max
 *    v(τ) = v1 + A_max·τ
 *    p(τ) = p1 + v1·τ + ½·A_max·τ²
 *
 *  Phase 3 (0 ≤ τ ≤ t3):  J = -J_max
 *    a(τ) = A_max - J·τ
 *    v(τ) = v2 + A_max·τ - ½·J·τ²
 *    p(τ) = p2 + v2·τ + ½·A_max·τ² - ⅙·J·τ³
 *
 *  Phase 4 (cruise): V = V_max_eff, A = 0, J = 0
 *
 *  Phase 5,6,7: สมมาตรกับ phase 3,2,1 (deceleration)
 *
 * ───────────────────────────────────────────────────────────────────────────
 */

#include "scurve_trajectory.h"
#include <math.h>
#include <string.h>

/* ─── Waypoints ─────────────────────────────────────────────────────────── */
const float WAYPOINTS_DEG[WAYPOINT_COUNT] = {90.0f,180.0f,360.0f,180.0f,270.0f};

/* ─── Helper macros ─────────────────────────────────────────────────────── */
#define FABS(x)     fabsf(x)
#define FMIN(a,b)   fminf((a),(b))
#define FSQRT(x)    sqrtf(x)
#define SIGN(x)     ((x) >= 0.0f ? 1.0f : -1.0f)

/* ─── ฟังก์ชัน internal ─────────────────────────────────────────────────── */

/**
 * @brief คำนวณระยะเวลาแต่ละ phase และ effective limits
 *
 * Algorithm:
 *   1. ระยะทาง accel zone ถ้าใช้ V_max เต็ม:
 *      d_acc = V_max²/(2·A_max) + V_max·A_max/(2·J_max)  [approx]
 *      ถ้า 2·d_acc > total_disp → ต้องลด V_max (short move)
 *
 *   2. ตรวจ A_max ว่าถึง V_max ก่อนหรือเปล่า (phase 2 อาจ = 0)
 */
static void _SCurve_ComputePhases(SCurveTraj_t *traj)
{
    float J = traj->j_max;
    float A = SCURVE_A_MAX_DEG_S2;
    float V = SCURVE_V_MAX_DEG_S;
    float D = traj->total_disp;   /* displacement รวม [deg] */

    /* ── ตรวจว่า A_max โตเกินไปจน phase 2 หาย ── */
    /* เวลาจาก 0 → V_max ด้วย jerk อย่างเดียว (ไม่มี phase 2) */
    float t_j = FSQRT(V / J);        /* เวลาถ้า phase 2 = 0 */
    float a_peak = J * t_j;          /* accel สูงสุดที่ได้     */

    float t1, t2, t3;

    if (a_peak <= A) {
        /* ถึง V_max ก่อนถึง A_max → phase 2 = 0 */
        t1 = t_j;
        t2 = 0.0f;
        t3 = t_j;
        A  = a_peak;   /* ใช้ effective A */
    } else {
        /* มี phase 2 */
        t1 = A / J;
        t2 = (V - J * t1 * t1) / A;
        t3 = t1;
    }

    /* ── ระยะทาง 1 acceleration zone (phase 1+2+3) ── */
    float d_acc = (V * (t1 + t2 + t3)) / 2.0f
                - (J * t1 * t1 * t1) / 6.0f * 0.0f  /* จัดรูป */
                + 0.0f;

    /* คำนวณ d_acc แม่นยำ:
       d1 = ⅙J·t1³
       v1 = ½J·t1²
       d2 = v1·t2 + ½A·t2²
       v2 = v1 + A·t2
       d3 = v2·t3 + ½A·t3² - ⅙J·t3³
       d_acc = d1+d2+d3  */
    {
        float v1 = 0.5f * J * t1 * t1;
        float d1 = (J * t1 * t1 * t1) / 6.0f;
        float d2 = v1 * t2 + 0.5f * A * t2 * t2;
        float v2 = v1 + A * t2;  /* = V_max */
        float d3 = v2 * t3 + 0.5f * A * t3 * t3 - (J * t3 * t3 * t3) / 6.0f;
        d_acc = d1 + d2 + d3;
        (void)v2;
    }

    /* ── Short move: ถ้า 2·d_acc > D ต้องลด V_max ── */
    if (2.0f * d_acc > D) {
        /* Solve: 2·d_acc(V_new) = D → bisection search */
        float v_lo = 0.0f, v_hi = V;
        float v_new = V;

        for (int iter = 0; iter < 32; iter++) {
            v_new = 0.5f * (v_lo + v_hi);

            /* คำนวณ t1_n, t2_n, t3_n สำหรับ v_new */
            float t1_n, t2_n, t3_n;
            float t_j_n = FSQRT(v_new / J);
            float a_p_n = J * t_j_n;

            if (a_p_n <= A) {
                t1_n = t_j_n; t2_n = 0.0f; t3_n = t_j_n;
                float An = a_p_n;
                float v1n = 0.5f * J * t1_n * t1_n;
                float d1n = (J * t1_n * t1_n * t1_n) / 6.0f;
                float d2n = 0.0f;
                float v2n = v1n;
                float d3n = v2n * t3_n + 0.5f * An * t3_n * t3_n
                           - (J * t3_n * t3_n * t3_n) / 6.0f;
                d_acc = d1n + d2n + d3n;
                (void)d2n;
            } else {
                t1_n = A / J;
                t2_n = (v_new - J * t1_n * t1_n) / A;
                t3_n = t1_n;
                float v1n = 0.5f * J * t1_n * t1_n;
                float d1n = (J * t1_n * t1_n * t1_n) / 6.0f;
                float d2n = v1n * t2_n + 0.5f * A * t2_n * t2_n;
                float v2n = v1n + A * t2_n;
                float d3n = v2n * t3_n + 0.5f * A * t3_n * t3_n
                           - (J * t3_n * t3_n * t3_n) / 6.0f;
                d_acc = d1n + d2n + d3n;
                (void)v2n;
            }

            if (2.0f * d_acc < D)
                v_lo = v_new;
            else
                v_hi = v_new;

            if ((v_hi - v_lo) < 1e-4f) break;
        }

        V = v_new;
        /* คำนวณ t1,t2,t3 ใหม่ด้วย V_new */
        float t_j_n = FSQRT(V / J);
        float a_p_n = J * t_j_n;
        if (a_p_n <= A) {
            t1 = t_j_n; t2 = 0.0f; t3 = t_j_n; A = a_p_n;
        } else {
            t1 = A / J;
            t2 = (V - J * t1 * t1) / A;
            t3 = t1;
        }
    }

    /* ── ระยะทาง cruise zone ── */
    {
        float v1 = 0.5f * J * t1 * t1;
        float d1 = (J * t1 * t1 * t1) / 6.0f;
        float d2 = v1 * t2 + 0.5f * A * t2 * t2;
        float v2 = v1 + A * t2;
        float d3 = v2 * t3 + 0.5f * A * t3 * t3 - (J * t3 * t3 * t3) / 6.0f;
        d_acc = d1 + d2 + d3;
        (void)v2;
    }

    float d_cruise = D - 2.0f * d_acc;
    if (d_cruise < 0.0f) d_cruise = 0.0f;

    float t4 = (V > 1e-6f) ? (d_cruise / V) : 0.0f;

    /* ── บันทึก ── */
    traj->v_max_eff = V;
    traj->a_max_eff = A;
    traj->j_max     = J;
    traj->t1 = t1; traj->t2 = t2; traj->t3 = t3;
    traj->t4 = t4;
    traj->t5 = t3; traj->t6 = t2; traj->t7 = t1;  /* สมมาตร */
}

/* ─── API: Init ──────────────────────────────────────────────────────────── */
void SCurve_Init(SCurveTraj_t *traj)
{
    memset(traj, 0, sizeof(SCurveTraj_t));
    traj->phase    = TRAJ_IDLE;
    traj->wp_index = 0;
    traj->active   = false;
    traj->j_max    = SCURVE_J_MAX_DEG_S3;
}

/* ─── API: กำหนด segment ──────────────────────────────────────────────────── */
void SCurve_SetSegment(SCurveTraj_t *traj, float q_start, float q_end)
{
    traj->q_start    = q_start;
    traj->q_end      = q_end;
    traj->direction  = SIGN(q_end - q_start);
    traj->total_disp = FABS(q_end - q_start);
    traj->j_max      = SCURVE_J_MAX_DEG_S3;

    if (traj->total_disp < 1e-4f) {
        /* displacement เล็กมาก → ถือว่าถึงเป้าแล้ว */
        traj->phase = TRAJ_DONE;
        traj->out.position_deg   = q_end;
        traj->out.velocity_deg_s = 0.0f;
        traj->out.accel_deg_s2   = 0.0f;
        return;
    }

    _SCurve_ComputePhases(traj);

    traj->phase           = TRAJ_PHASE1;
    traj->t_phase         = 0.0f;
    traj->pos_phase_start = q_start;
    traj->vel_phase_start = 0.0f;
    traj->acc_phase_start = 0.0f;
    traj->active          = true;
}

/* ─── API: Update (เรียกทุก TRAJ_DT) ────────────────────────────────────── */
TrajOutput_t SCurve_Update(SCurveTraj_t *traj)
{
    if (traj->phase == TRAJ_IDLE || traj->phase == TRAJ_DONE) {
        return traj->out;
    }

    float dt  = TRAJ_DT;
    float J   = traj->j_max;
    float A   = traj->a_max_eff;
    float V   = traj->v_max_eff;
    float dir = traj->direction;

    float τ   = traj->t_phase;   /* เวลาใน phase ปัจจุบัน */
    float p0  = traj->pos_phase_start;
    float v0  = traj->vel_phase_start;
    float a0  = traj->acc_phase_start;

    float pos, vel, acc;

    /* ── คำนวณ pos/vel/acc ตาม phase ── */
    switch (traj->phase)
    {
        /* ─ Phase 1: J = +J_max ─────────────────── */
        case TRAJ_PHASE1:
            acc = a0 + J * τ;
            vel = v0 + a0 * τ + 0.5f * J * τ * τ;
            pos = p0 + dir * (v0 * τ + 0.5f * a0 * τ * τ + (J * τ * τ * τ) / 6.0f);
            break;

        /* ─ Phase 2: J = 0, A = A_max ───────────── */
        case TRAJ_PHASE2:
            acc = a0;
            vel = v0 + a0 * τ;
            pos = p0 + dir * (v0 * τ + 0.5f * a0 * τ * τ);
            break;

        /* ─ Phase 3: J = -J_max ─────────────────── */
        case TRAJ_PHASE3:
            acc = a0 - J * τ;
            vel = v0 + a0 * τ - 0.5f * J * τ * τ;
            pos = p0 + dir * (v0 * τ + 0.5f * a0 * τ * τ - (J * τ * τ * τ) / 6.0f);
            break;

        /* ─ Phase 4: Cruise ─────────────────────── */
        case TRAJ_PHASE4:
            acc = 0.0f;
            vel = V;
            pos = p0 + dir * (V * τ);
            break;

        /* ─ Phase 5: J = -J_max (decel เริ่ม) ──── */
        case TRAJ_PHASE5:
            acc = a0 - J * τ;
            vel = v0 + a0 * τ - 0.5f * J * τ * τ;
            pos = p0 + dir * (v0 * τ + 0.5f * a0 * τ * τ - (J * τ * τ * τ) / 6.0f);
            break;

        /* ─ Phase 6: J = 0, A = -A_max ─────────── */
        case TRAJ_PHASE6:
            acc = a0;
            vel = v0 + a0 * τ;
            pos = p0 + dir * (v0 * τ + 0.5f * a0 * τ * τ);
            break;

        /* ─ Phase 7: J = +J_max (decel ลด) ─────── */
        case TRAJ_PHASE7:
            acc = a0 + J * τ;
            vel = v0 + a0 * τ + 0.5f * J * τ * τ;
            pos = p0 + dir * (v0 * τ + 0.5f * a0 * τ * τ + (J * τ * τ * τ) / 6.0f);
            break;

        default:
            pos = traj->q_end;
            vel = 0.0f;
            acc = 0.0f;
            break;
    }

    /* ── อัปเดต output ── */
    traj->out.position_deg   = pos;
    traj->out.velocity_deg_s = dir * vel;
    traj->out.accel_deg_s2   = dir * acc;

    /* ── เดิน t_phase ไปข้างหน้า ── */
    traj->t_phase += dt;

    /* ── ตรวจการเปลี่ยน phase ── */
    /* Helper: สิ้นสุด phase เมื่อ t_phase >= duration ของ phase นั้น */
    float phase_dur = 0.0f;
    switch (traj->phase) {
        case TRAJ_PHASE1: phase_dur = traj->t1; break;
        case TRAJ_PHASE2: phase_dur = traj->t2; break;
        case TRAJ_PHASE3: phase_dur = traj->t3; break;
        case TRAJ_PHASE4: phase_dur = traj->t4; break;
        case TRAJ_PHASE5: phase_dur = traj->t5; break;
        case TRAJ_PHASE6: phase_dur = traj->t6; break;
        case TRAJ_PHASE7: phase_dur = traj->t7; break;
        default: break;
    }

    if (traj->t_phase >= phase_dur) {
        /* คำนวณค่าปลาย phase (เอาไว้เป็น initial condition phase ถัดไป) */
        float τ_end = phase_dur;
        float pos_end, vel_end, acc_end;

        switch (traj->phase) {
            case TRAJ_PHASE1:
                acc_end = a0 + J * τ_end;
                vel_end = v0 + a0 * τ_end + 0.5f * J * τ_end * τ_end;
                pos_end = p0 + dir * (v0 * τ_end + 0.5f * a0 * τ_end * τ_end
                          + (J * τ_end * τ_end * τ_end) / 6.0f);
                traj->phase = (traj->t2 > 1e-6f) ? TRAJ_PHASE2 : TRAJ_PHASE3;
                break;
            case TRAJ_PHASE2:
                acc_end = a0;
                vel_end = v0 + a0 * τ_end;
                pos_end = p0 + dir * (v0 * τ_end + 0.5f * a0 * τ_end * τ_end);
                traj->phase = TRAJ_PHASE3;
                break;
            case TRAJ_PHASE3:
                acc_end = a0 - J * τ_end;
                vel_end = v0 + a0 * τ_end - 0.5f * J * τ_end * τ_end;
                pos_end = p0 + dir * (v0 * τ_end + 0.5f * a0 * τ_end * τ_end
                          - (J * τ_end * τ_end * τ_end) / 6.0f);
                acc_end = 0.0f;   /* ควรเป็น 0 พอดี */
                vel_end = V;      /* snap to V_max  */
                traj->phase = (traj->t4 > 1e-6f) ? TRAJ_PHASE4 : TRAJ_PHASE5;
                break;
            case TRAJ_PHASE4:
                acc_end = 0.0f;
                vel_end = V;
                pos_end = p0 + dir * (V * τ_end);
                traj->phase = TRAJ_PHASE5;
                break;
            case TRAJ_PHASE5:
                acc_end = a0 - J * τ_end;
                vel_end = v0 + a0 * τ_end - 0.5f * J * τ_end * τ_end;
                pos_end = p0 + dir * (v0 * τ_end + 0.5f * a0 * τ_end * τ_end
                          - (J * τ_end * τ_end * τ_end) / 6.0f);
                traj->phase = (traj->t6 > 1e-6f) ? TRAJ_PHASE6 : TRAJ_PHASE7;
                break;
            case TRAJ_PHASE6:
                acc_end = a0;
                vel_end = v0 + a0 * τ_end;
                pos_end = p0 + dir * (v0 * τ_end + 0.5f * a0 * τ_end * τ_end);
                traj->phase = TRAJ_PHASE7;
                break;
            case TRAJ_PHASE7:
                /* สิ้นสุด — snap to target */
                pos_end = traj->q_end;
                vel_end = 0.0f;
                acc_end = 0.0f;
                traj->phase = TRAJ_DONE;
                traj->out.position_deg   = pos_end;
                traj->out.velocity_deg_s = 0.0f;
                traj->out.accel_deg_s2   = 0.0f;
                traj->active = false;
                return traj->out;
            default:
                pos_end = traj->q_end;
                vel_end = 0.0f;
                acc_end = 0.0f;
                break;
        }

        /* ตั้งค่า initial condition phase ถัดไป */
        traj->pos_phase_start = pos_end;
        traj->vel_phase_start = FABS(vel_end);   /* ใช้ magnitude, dir แยก */
        traj->acc_phase_start = acc_end;
        traj->t_phase         = 0.0f;
    }

    return traj->out;
}

/* ─── API: IsDone ────────────────────────────────────────────────────────── */
bool SCurve_IsDone(const SCurveTraj_t *traj)
{
    return (traj->phase == TRAJ_DONE || traj->phase == TRAJ_IDLE);
}

/* ─── API: RunWaypoints (auto-advance) ───────────────────────────────────── */
TrajOutput_t SCurve_RunWaypoints(SCurveTraj_t *traj, float current_pos)
{
    /* ถ้ายังไม่เริ่ม → ตั้ง segment แรก */
    if (!traj->active && traj->wp_index == 0 && traj->phase == TRAJ_IDLE) {
        if (WAYPOINT_COUNT > 0) {
            SCurve_SetSegment(traj, current_pos, WAYPOINTS_DEG[0]);
            traj->wp_index = 1;
        }
    }

    /* ถ้า segment ปัจจุบันเสร็จ → ไป waypoint ถัดไป */
    if (SCurve_IsDone(traj) && traj->wp_index < WAYPOINT_COUNT) {
        float q_next = WAYPOINTS_DEG[traj->wp_index];
        SCurve_SetSegment(traj, traj->q_end, q_next);
        traj->wp_index++;
    }

    return SCurve_Update(traj);
}
