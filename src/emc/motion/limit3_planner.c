/********************************************************************
* Description: limit3_planner.c
*
*   A simple single axis trajectory planner using simple_tp interface.
*   See simple_tp.h for API.
*
* Author: jmkasunich
* License: GPL Version 2
* Created on:
* System: Linux
*
* Copyright (c) 2004 All rights reserved.

* Refactor to used limit3_planner by John Morris (zultron):

Notes:
  1) tp->min_pos,max_pos are not used by joint->free_tp, axis->teleop_tp
     since limits are managed elsewhere in the motion module
  2) tp->options, SIMPLE_TP_DISALLOW_BACKOFF bit
         0: (DEFAULT) allow backoff to mitigate overshoot
         1: disallow backoff for special cases
********************************************************************/
#include "rtapi_math.h"
#include "simple_tp.h"
#include "limit3_planner_code.h"

void limit3_planner_update(simple_tp_t *tp, double period)
{
    struct limit3_parms parms;
    struct limit3_parms *p = &parms;

    // limit3_planner parameters:
    p->L3_pos_cmd  = &tp->pos_cmd;
    p->L3_curr_pos = &tp->curr_pos;
    p->L3_min_pos  = &tp->min_pos;
    p->L3_max_pos  = &tp->max_pos;
    p->L3_max_vel  = &tp->max_vel;
    p->L3_max_acc  = &tp->max_acc;
    p->L3_options  = &tp->options;
    p->L3_curr_vel = &tp->curr_vel;
    p->L3_in_old   = &tp->in_old;
    p->L3_out_vel  = &tp->out_vel;
    p->L3_out_old  = &tp->out_old;
    p->L3_active   = &tp->active;

    if (! tp->enable) {
        *p->L3_pos_cmd = *p->L3_curr_pos;
        SET_NEXT_STATE(p, *p->L3_curr_pos);
    }
    limit3_planner(p,period) ;
    return;
}
