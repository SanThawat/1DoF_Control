/**
 * @file    scurve_trajectory.c
 * @brief   S-Curve Trajectory Generator — Pure Kinematic Implementation
 *          STM32 NUCLEO-G474RE | 1-DOF Robot Arm
 *
 * หลักการ: คำนวณ pos/vel/acc จากสมการ kinematic โดยตรง
 *           ไม่มี Euler integration → ไม่มี drift
 *
 * สมการแต่ละ phase (τ = เวลาใน phase ปัจจุบัน):
 *
 *  Phase 1: J = +J_max
 *    acc(τ) = J·τ
 *    vel(τ) = ½·J·τ²
 *    pos(τ) = p0 + v0·τ + ⅙·J·τ³
 *
 *  Phase 2: J = 0, A = A_max
 *    acc(τ) = A
 *    vel(τ) = v0 + A·τ
 *    pos(τ) = p0 + v0·τ + ½·A·τ²
 *
 *  Phase 3: J = -J_max
 *    acc(τ) = A - J·τ
 *    vel(τ) = v0 + A·τ - ½·J·τ²
 *    pos(τ) = p0 + v0·τ + ½·A·τ² - ⅙·J·τ³
 *
 *  Phase 4: cruise
 *    acc(τ) = 0
 *    vel(τ) = V_max
 *    pos(τ) = p0 + V_max·τ
 *
 *  Phase 5: J = -J_max  (decel เริ่ม, a จาก 0 ลงติดลบ)
 *    acc(τ) = -J·τ
 *    vel(τ) = v0 - ½·J·τ²
 *    pos(τ) = p0 + v0·τ - ⅙·J·τ³
 *
 *  Phase 6: J = 0, A = -A_max
 *    acc(τ) = -A
 *    vel(τ) = v0 - A·τ
 *    pos(τ) = p0 + v0·τ - ½·A·τ²
 *
 *  Phase 7: J = +J_max  (decel ลด, หยุด)
 *    acc(τ) = -A + J·τ
 *    vel(τ) = v0 - A·τ + ½·J·τ²
 *    pos(τ) = p0 + v0·τ - ½·A·τ² + ⅙·J·τ³
 */

#include "scurve_trajectory.h"
#include <math.h>
#include <string.h>

/* ─── Waypoints ─────────────────────────────────────────────────────────── */
const float WAYPOINTS_DEG[WAYPOINT_COUNT] = {10.0f, 45.0f, 60.0f, 90.0f, 180.0f, 360.0f};

/* ─── Helpers ───────────────────────────────────────────────────────────── */
#define SIGN(x)   ((x) >= 0.0f ? 1.0f : -1.0f)

/* ─── คำนวณ boundary conditions ณ ต้นแต่ละ phase ──────────────────────── */
static void _Compute_BoundaryConditions(SCurveTraj_t *traj)
{
    const float J  = traj->j_max;
    const float A  = traj->a_max_eff;
    const float V  = traj->v_max_eff;
    const float t1 = traj->t1;
    const float t2 = traj->t2;
    const float t3 = traj->t3;
    const float t4 = traj->t4;
    const float t5 = traj->t5;
    const float t6 = traj->t6;

    PhaseIC_t *ic = traj->ic;

    /* Phase 1 start */
    ic[0].p = 0.0f;
    ic[0].v = 0.0f;

    /* Phase 2 start = end of Phase 1 */
    ic[1].v = 0.5f * J * t1 * t1;
    ic[1].p = (J * t1 * t1 * t1) / 6.0f;

    /* Phase 3 start = end of Phase 2 */
    ic[2].v = ic[1].v + A * t2;
    ic[2].p = ic[1].p + ic[1].v * t2 + 0.5f * A * t2 * t2;

    /* Phase 4 start = end of Phase 3 (snap to V_max) */
    ic[3].v = V;
    ic[3].p = ic[2].p + ic[2].v * t3 + 0.5f * A * t3 * t3
              - (J * t3 * t3 * t3) / 6.0f;

    /* Phase 5 start = end of Phase 4 (cruise) */
    ic[4].v = V;
    ic[4].p = ic[3].p + V * t4;

    /* Phase 6 start = end of Phase 5 */
    ic[5].v = V - 0.5f * J * t5 * t5;
    ic[5].p = ic[4].p + V * t5 - (J * t5 * t5 * t5) / 6.0f;

    /* Phase 7 start = end of Phase 6 */
    ic[6].v = ic[5].v - A * t6;
    ic[6].p = ic[5].p + ic[5].v * t6 - 0.5f * A * t6 * t6;
}

/* ─── คำนวณ phase duration ───────────────────────────────────────────────── */
static void _SCurve_ComputePhases(SCurveTraj_t *traj)
{
    float J = traj->j_max;
    float A = SCURVE_A_MAX_DEG_S2;
    float V = SCURVE_V_MAX_DEG_S;
    float D = traj->total_disp;

    float t1, t2, t3;
    const float t_j    = sqrtf(V / J);
    const float a_peak = J * t_j;

    if (a_peak <= A) {
        t1 = t_j; t2 = 0.0f; t3 = t_j; A = a_peak;
    } else {
        t1 = A / J;
        t2 = (V - J * t1 * t1) / A;
        t3 = t1;
    }

    /* d_acc */
    float v1    = 0.5f * J * t1 * t1;
    float d1    = (J * t1 * t1 * t1) / 6.0f;
    float d2    = v1 * t2 + 0.5f * A * t2 * t2;
    float v2    = v1 + A * t2;
    float d3    = v2 * t3 + 0.5f * A * t3 * t3 - (J * t3 * t3 * t3) / 6.0f;
    float d_acc = d1 + d2 + d3;

    /* Short move */
    if (2.0f * d_acc > D) {
        float v_lo = 0.0f, v_hi = V, v_new = V;
        for (int i = 0; i < 40; i++) {
            v_new       = 0.5f * (v_lo + v_hi);
            float tjn   = sqrtf(v_new / J);
            float apn   = J * tjn;
            float t1n, t2n, t3n, An;
            if (apn <= A) {
                t1n = tjn; t2n = 0.0f; t3n = tjn; An = apn;
            } else {
                t1n = A / J; t2n = (v_new - J*t1n*t1n)/A; t3n = t1n; An = A;
            }
            float v1n = 0.5f * J * t1n * t1n;
            float da  = (J*t1n*t1n*t1n)/6.0f
                      + v1n*t2n + 0.5f*An*t2n*t2n
                      + (v1n+An*t2n)*t3n + 0.5f*An*t3n*t3n
                      - (J*t3n*t3n*t3n)/6.0f;
            if (2.0f * da < D) v_lo = v_new; else v_hi = v_new;
            if ((v_hi - v_lo) < 1e-4f) break;
        }
        V = v_new;
        float tjn = sqrtf(V / J);
        if (J * tjn <= A) {
            t1 = tjn; t2 = 0.0f; t3 = tjn; A = J * tjn;
        } else {
            t1 = A/J; t2 = (V - J*t1*t1)/A; t3 = t1;
        }
        v1    = 0.5f*J*t1*t1;
        d1    = (J*t1*t1*t1)/6.0f;
        d2    = v1*t2 + 0.5f*A*t2*t2;
        v2    = v1 + A*t2;
        d3    = v2*t3 + 0.5f*A*t3*t3 - (J*t3*t3*t3)/6.0f;
        d_acc = d1+d2+d3;
    }

    float d_cruise = D - 2.0f * d_acc;
    if (d_cruise < 0.0f) d_cruise = 0.0f;
    float t4 = (V > 1e-6f) ? (d_cruise / V) : 0.0f;

    traj->v_max_eff = V;
    traj->a_max_eff = A;
    traj->j_max     = J;
    traj->t1 = t1; traj->t2 = t2; traj->t3 = t3;
    traj->t4 = t4;
    traj->t5 = t3; traj->t6 = t2; traj->t7 = t1;

    _Compute_BoundaryConditions(traj);
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

/* ─── API: SetSegment ────────────────────────────────────────────────────── */
void SCurve_SetSegment(SCurveTraj_t *traj, float q_start, float q_end)
{
    traj->q_start    = q_start;
    traj->q_end      = q_end;
    traj->direction  = SIGN(q_end - q_start);
    traj->total_disp = fabsf(q_end - q_start);
    traj->j_max      = SCURVE_J_MAX_DEG_S3;

    if (traj->total_disp < 1e-4f) {
        traj->phase              = TRAJ_DONE;
        traj->out.position_deg   = q_end;
        traj->out.velocity_deg_s = 0.0f;
        traj->out.accel_deg_s2   = 0.0f;
        return;
    }

    _SCurve_ComputePhases(traj);

    traj->phase   = TRAJ_PHASE1;
    traj->t_phase = 0.0f;
    traj->active  = true;
}

/* ─── API: Update — Pure Kinematic ──────────────────────────────────────── */
TrajOutput_t SCurve_Update(SCurveTraj_t *traj)
{
    if (traj->phase == TRAJ_IDLE || traj->phase == TRAJ_DONE) {
        return traj->out;
    }

    const float J   = traj->j_max;
    const float A   = traj->a_max_eff;
    const float V   = traj->v_max_eff;
    const float dir = traj->direction;
    const float τ   = traj->t_phase;

    /* ic[] index = phase - 1 (0-based) */
    const int   ph  = (int)traj->phase - 1;
    const float p0  = traj->ic[ph].p;
    const float v0  = traj->ic[ph].v;

    float pos_offset, vel, acc;

    switch (traj->phase)
    {
        case TRAJ_PHASE1:   /* J = +J */
            acc        =  J * τ;
            vel        =  v0 + 0.5f * J * τ * τ;
            pos_offset =  p0 + v0 * τ + (J * τ * τ * τ) / 6.0f;
            break;

        case TRAJ_PHASE2:   /* A คงที่ */
            acc        =  A;
            vel        =  v0 + A * τ;
            pos_offset =  p0 + v0 * τ + 0.5f * A * τ * τ;
            break;

        case TRAJ_PHASE3:   /* J = -J */
            acc        =  A - J * τ;
            vel        =  v0 + A * τ - 0.5f * J * τ * τ;
            pos_offset =  p0 + v0 * τ + 0.5f * A * τ * τ - (J * τ * τ * τ) / 6.0f;
            break;

        case TRAJ_PHASE4:   /* Cruise */
            acc        =  0.0f;
            vel        =  V;
            pos_offset =  p0 + V * τ;
            break;

        case TRAJ_PHASE5:   /* J = -J  (decel เริ่ม) */
            acc        = -J * τ;
            vel        =  v0 - 0.5f * J * τ * τ;
            pos_offset =  p0 + v0 * τ - (J * τ * τ * τ) / 6.0f;
            break;

        case TRAJ_PHASE6:   /* -A คงที่ */
            acc        = -A;
            vel        =  v0 - A * τ;
            pos_offset =  p0 + v0 * τ - 0.5f * A * τ * τ;
            break;

        case TRAJ_PHASE7:   /* J = +J (decel ลด) */
            acc        = -A + J * τ;
            vel        =  v0 - A * τ + 0.5f * J * τ * τ;
            pos_offset =  p0 + v0 * τ - 0.5f * A * τ * τ + (J * τ * τ * τ) / 6.0f;
            break;

        default:
            traj->out.position_deg   = traj->q_end;
            traj->out.velocity_deg_s = 0.0f;
            traj->out.accel_deg_s2   = 0.0f;
            return traj->out;
    }

    if (vel < 0.0f) vel = 0.0f;   /* numerical safety */

    traj->out.position_deg   = traj->q_start + dir * pos_offset;
    traj->out.velocity_deg_s = dir * vel;
    traj->out.accel_deg_s2   = dir * acc;

    /* ── เดิน t_phase และตรวจเปลี่ยน phase ── */
    traj->t_phase += TRAJ_DT;

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
        traj->t_phase = 0.0f;
        switch (traj->phase) {
            case TRAJ_PHASE1:
                traj->phase = (traj->t2 > 1e-6f) ? TRAJ_PHASE2 : TRAJ_PHASE3; break;
            case TRAJ_PHASE2: traj->phase = TRAJ_PHASE3; break;
            case TRAJ_PHASE3:
                traj->phase = (traj->t4 > 1e-6f) ? TRAJ_PHASE4 : TRAJ_PHASE5; break;
            case TRAJ_PHASE4: traj->phase = TRAJ_PHASE5; break;
            case TRAJ_PHASE5:
                traj->phase = (traj->t6 > 1e-6f) ? TRAJ_PHASE6 : TRAJ_PHASE7; break;
            case TRAJ_PHASE6: traj->phase = TRAJ_PHASE7; break;
            case TRAJ_PHASE7:
                traj->out.position_deg   = traj->q_end;
                traj->out.velocity_deg_s = 0.0f;
                traj->out.accel_deg_s2   = 0.0f;
                traj->phase  = TRAJ_DONE;
                traj->active = false;
                break;
            default: break;
        }
    }

    return traj->out;
}

/* ─── API: IsDone ────────────────────────────────────────────────────────── */
bool SCurve_IsDone(const SCurveTraj_t *traj)
{
    return (traj->phase == TRAJ_DONE || traj->phase == TRAJ_IDLE);
}

/* ─── API: RunWaypoints ──────────────────────────────────────────────────── */
TrajOutput_t SCurve_RunWaypoints(SCurveTraj_t *traj, float current_pos)
{
    if (!traj->active && traj->wp_index == 0 && traj->phase == TRAJ_IDLE) {
        if (WAYPOINT_COUNT > 0) {
            SCurve_SetSegment(traj, current_pos, WAYPOINTS_DEG[0]);
            traj->wp_index = 1;
        }
    }

    if (SCurve_IsDone(traj) && traj->wp_index < WAYPOINT_COUNT) {
        SCurve_SetSegment(traj, traj->q_end, WAYPOINTS_DEG[traj->wp_index]);
        traj->wp_index++;
    }

    return SCurve_Update(traj);
}
