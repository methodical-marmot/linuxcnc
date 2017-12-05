#include "simple_tp.h"
#include "rtapi_math.h"

/*
* This file provides the limit3_planner() designed
* by John Morris (zultron) as a refactor of the original
* limit3.comp by John Kasunich.
*
* The code is used by for the hal limit3 component and
* conditionally for single joint or axis jogging and
* homing as enabled by #define USE_LIMIT3_PLANNER in
* src/emc/motion/control.c
*/

#define SET_NEXT_STATE(p,out_pos)           \
    do {                                    \
        *p->L3_out_old  =  *p->L3_curr_pos; \
        *p->L3_curr_pos =  out_pos;         \
        *p->L3_curr_vel =  (*p->L3_curr_pos - *p->L3_out_old)/period; \
        *p->L3_in_old   =  *p->L3_pos_cmd;  \
        *p->L3_active   =  fabs(*p->L3_curr_pos - *p->L3_pos_cmd) \
                        >= TINY_DP(*p->L3_max_acc,period);        \
        return;                                                   \
    } while (0)


#define VALID_NEXT(pos) ((pos) <= max_pos && (pos) >= min_pos)

// Distance = avg. velocity * time
#define S_GIVEN_VI_VF_T(vi,vf,t) ((vf + vi)/2 * t)
// Time = chg. velocity / acceleration
#define T_GIVEN_VI_VF_A(vi,vf,a) (fabs((vf - vi) / a))
// Final velocity = initial velocity + acceleration * time
#define VF_GIVEN_VI_A_T(vi,a,t) (vi + a*t)

struct limit3_parms {
    double *L3_pos_cmd;  // limit3 pin: in
    double *L3_curr_pos; // limit3 pin: out
    double *L3_min_pos;  // limit3 pin: min
    double *L3_max_pos;  // limit3 pin: max
    double *L3_max_vel;  // limit3 pin: maxv
    double *L3_max_acc;  // limit3 pin: maxa
    int    *L3_options;  // limit3 support pin: nobackoff
    double *L3_curr_vel; // limit3 instance variable
    double *L3_in_old;   // limit3 instance variable
    double *L3_out_old;  // limit3 instance variable
    double *L3_out_vel;  // limit3 instance variable
    int    *L3_active;   // limit3 instance variable
};

void limit3_planner(struct limit3_parms *p,double period)
{
    double in_vel, min_vel, max_vel, min_pos, max_pos;
    double stop_pos_max, stop_pos_min, stop_time_max, stop_time_min;
    double vel_match_time, vel_match_in_pos, vel_match_out_pos;
    int out_dir_rel;
    double a, v, s, t;
    int reason = 0; // Used for debugging

#define OUTPUT  *p->L3_curr_pos
#define INPUT   *p->L3_pos_cmd
#define IN_OLD  *p->L3_in_old
#define OUT_OLD *p->L3_out_old
#define OUT_VEL *p->L3_out_vel
#define MINPOS  *p->L3_min_pos
#define MAXPOS  *p->L3_max_pos
#define MAXVEL  *p->L3_max_vel
#define MAXACC  *p->L3_max_acc

    // Input velocity
    in_vel = (INPUT - IN_OLD) / period;
    // Output velocity:  v_cur = 2 * v_avg - v_prev
    // - FIXME:  why does this appear to lag by a cycle?
    OUT_VEL = 2 * (OUTPUT-OUT_OLD)/period - OUT_VEL;
    // Most negative/positive velocity reachable in one period
    // - vf = vi + a * t
    min_vel = fmax(VF_GIVEN_VI_A_T(OUT_VEL, -MAXACC, period), -MAXVEL);
    max_vel = fmin(VF_GIVEN_VI_A_T(OUT_VEL,  MAXACC, period),  MAXVEL);
    // Most negative/positive position reachable in one period
    // - cur. pos + (distance to reach min/max vel in one period)
    min_pos = OUTPUT + S_GIVEN_VI_VF_T(OUT_VEL, min_vel, period);
    max_pos = OUTPUT + S_GIVEN_VI_VF_T(OUT_VEL, max_vel, period);

    // Direction of output movement relative to input movement
    out_dir_rel = (OUT_VEL - in_vel < 0) ? -1 : 1;

    // Calculate shortest distance to stop after next step
    stop_time_max = T_GIVEN_VI_VF_A(max_vel, 0.0, -MAXACC);      // - time to decel from MAXVEL to 0
    stop_pos_max  = max_pos
                  + S_GIVEN_VI_VF_T(max_vel, 0.0, stop_time_max);// - distance to stop from max_pos
    stop_time_min = T_GIVEN_VI_VF_A(min_vel, 0.0, -MAXACC);      // - time to decel from min_vel to 0
    stop_pos_min  = min_pos                                      // - distance to stop from min_pos
                  + S_GIVEN_VI_VF_T(min_vel, 0.0, stop_time_min);

    // Follow input signal:  match position and velocity
    // - min time for velocity match
    vel_match_time = fabs(OUT_VEL-in_vel) / MAXACC;
    // - input position after velocity match
    vel_match_in_pos = INPUT + in_vel * vel_match_time;
    // - output position after velocity match
    vel_match_out_pos = OUTPUT
                      + OUT_VEL * (vel_match_time+period)
                      + 0.5 * (-out_dir_rel * MAXACC) * pow(vel_match_time,2);

    // Goal:  head toward and stop at min position limit when:
    // - Input signal is headed below min limit
    // - Input signal is below min limit & headed back, but too early to chase
    // - Output signal is moving toward min position limit and may overshoot
    reason = 1000;
    if  (   (vel_match_in_pos < MINPOS)
         || (INPUT <= MINPOS && vel_match_in_pos < vel_match_out_pos)
         || (stop_pos_min <= MINPOS && !VALID_NEXT(MINPOS))
        ) {
        reason += 100;
        t = 2 * (MINPOS-OUTPUT) / (OUT_VEL + 0); // t:  time to decel to stop on lim
        // Decide what to do, most urgent to least
        if (VALID_NEXT(MINPOS) && t < period) {
            // This period:  stop & lock on limit will stay within constraints
            SET_NEXT_STATE(p,MINPOS);
        }
        reason += 100;
        if (stop_time_max < period) {
            // <= two periods:  finesse acceleration for a greased landing
            a = (0-OUT_VEL)/t;            // a:  accel to decel to stop on lim
            v = OUT_VEL + a * period;     // v:  vel after accel a for 1 per.
            s = (v + OUT_VEL)/2 * period; // s:  dist after accel a for 1 per.
            SET_NEXT_STATE(p,OUTPUT + s);
        }
        reason += 100;
        if (stop_pos_min <= MINPOS && !VALID_NEXT(MINPOS)) {
            // N periods:  in danger of overshoot; maximum deceleration
            SET_NEXT_STATE(p,max_pos);
        }
        reason += 100;
        // No urgent timing:  maximum acceleration
        SET_NEXT_STATE(p,min_pos);
    }
    reason = 2000;
    if (   (vel_match_in_pos > MAXPOS)                     // Input above max limit
        || (INPUT >= MAXPOS && vel_match_in_pos > vel_match_out_pos)
        || (stop_pos_max >= MAXPOS && !VALID_NEXT(MAXPOS))
       ) {
        reason += 100;
        t = 2 * (MAXPOS-OUTPUT) / (OUT_VEL + 0); // t:  time to decel to stop on lim
        // Decide what to do, most urgent to least
        if (VALID_NEXT(MAXPOS) && t < period) {
            // This period:  stop & lock on limit will stay within constraints
            SET_NEXT_STATE(p,MAXPOS);
        }
        reason += 100;
        if (stop_time_min < period) {
            // <= two periods:  finesse acceleration for a greased landing
            a = (0-OUT_VEL)/t;            // a:  accel to decel to stop on lim
            v = OUT_VEL + a * period;     // v:  vel after accel a for 1 per.
            s = (v + OUT_VEL)/2 * period; // s:  dist after accel a for 1 per.
            SET_NEXT_STATE(p,s + OUTPUT);
        }
        reason += 100;
        if (stop_pos_max >= MAXPOS && !VALID_NEXT(MAXPOS)) {
            // N periods:  in danger of overshoot; maximum deceleration
            SET_NEXT_STATE(p,min_pos);
        }
        reason += 100;
        // No urgent timing:  maximum acceleration
        SET_NEXT_STATE(p,max_pos);
    }

    // Follow input signal
    //
    reason = 3000;
    // - Try to track input
    reason += 100;
    if (VALID_NEXT(INPUT)) {
        SET_NEXT_STATE(p,INPUT);
    }
    // - Try to match position and velocity without overshooting
    if (OUTPUT > INPUT) {                           // Output > input:
        reason += 100;
        if (   (vel_match_in_pos < vel_match_out_pos) // - Not overshooting
            || *p->L3_options & SIMPLE_TP_DISALLOW_BACKOFF) {
            SET_NEXT_STATE(p,min_pos);                //   - Move closer
        } else  {                                     // - Overshooting
            SET_NEXT_STATE(p,max_pos);                //   - Back off
        }
    } else {                                          // Output <= input
        reason += 200;
        if (   (vel_match_in_pos > vel_match_out_pos) // - Not overshooting
            || *p->L3_options & SIMPLE_TP_DISALLOW_BACKOFF) {
            SET_NEXT_STATE(p,max_pos);                //   - Move closer
        } else {                                      // - Overshooting
            SET_NEXT_STATE(p,min_pos);                //   - Back off
        }
    }

    // Shouldn't get here
    reason = 4000;
    SET_NEXT_STATE(p,(max_pos-min_pos)/2);
}
