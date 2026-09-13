#include "TrajectoryGenerator.h"

#include <math.h>

// std::min / std::max, written out so the compiler emits a plain minsd / maxsd instead of libm's
// NaN aware fmin / fmax. same result for every value that can reach them
static inline double dmin(double a, double b)
{
    return (b < a) ? b : a;
}

static inline double dmax(double a, double b)
{
    return (a < b) ? b : a;
}

// zeroes the limits and puts back the two window defaults a C struct cannot carry on its own
void MotionLimitsInit(MotionLimits *limits)
{
    limits->MaxVelocity = 0.0;
    limits->MaxAcceleration = 0.0;
    limits->MaxDeceleration = 0.0;
    limits->Jerk = 0.0;
    limits->NegativeLimit = 0.0;
    limits->PositiveLimit = 0.0;
    limits->InPositionWindow = 1e-4;
    limits->InVelocityWindow = 1e-3;
}

// quadratic equation solver, returns non zero if a real root exists
int solveQuadratic(double a, double b, double c, double *x1, double *x2)
{
    double discriminant = b * b - 4 * a * c;    // is there a root

    if (discriminant < 0)
    {
        return 0;       // no real root
    }

    double sqrt_discriminant = sqrt(discriminant);
    double denom = 2 * a;       // note: no a = 0 check

    *x1 = (-b + sqrt_discriminant) / denom;     // larger root
    *x2 = (-b - sqrt_discriminant) / denom;     // smaller root

    return 1;
}

// distance the axis covers if it starts braking right now, until standstill. magnitude, always
// positive. only needs the state and the two limits the braking phases use. static so the scan can
// inline it - the bisection below calls it once per step and it is the hot path of the whole core
static double CalcBrakeDistance(double cur_vel, double cur_acc, double max_dec, double jerk)
{
    const double acc_abs = fabs(cur_acc);
    const double vel_abs = fabs(cur_vel);

    double brake_distance = 0.0;        // distance i would cover if i braked right now
    double t1, t2, t3;          // phase durations
    double x1, x2, x3, x4;      // distance covered in each phase
    double v1, v2, v3;          // velocity at the end of each phase
    double signvelocity;        // jerk is applied opposite to the velocity

    if (cur_acc * cur_vel > TOLERANCE)      // acceleration and velocity same direction ----- so we are speeding up
    {
        // first the speeding up has to be stopped, that is the acceleration has to be zeroed.
        signvelocity = (cur_vel < 0.0) ? 1 : -1;        // velocity direction matters for the jerk
        t1 = acc_abs / jerk;        // time it takes to zero the current acceleration
        x1 = (cur_vel * t1) + (0.5 * cur_acc * t1 * t1) + (signvelocity / 6.0) * jerk * t1 * t1 * t1;      //distance covered while the acceleration is zeroed
        v1 = cur_vel + (cur_acc * t1) + (0.5 * signvelocity * jerk * t1 * t1);      // velocity reached once the acceleration is zero

        const double x1_abs = fabs(x1);     // the braking phases below run on magnitudes, same as the slowing down branch
        const double v1_abs = fabs(v1);

        if (v1_abs > ((max_dec * max_dec / jerk) + TOLERANCE))        // can the velocity reached at zero acceleration be killed without hitting max deceleration?
        {
            t2 = max_dec / jerk;        // time for acceleration 0 -> -max_dec
            x2 = (v1_abs * t2) - (0.1666667 * jerk * t2 * t2 * t2);     // distance covered in this phase
            v2 = v1_abs - (0.5 * jerk * t2 * t2);       // velocity left once we sit on max_dec

            t3 = (v2 - (0.5 * max_dec * max_dec / jerk)) / max_dec;     // time spent at constant max_dec
            x3 = (v2 * t3) - (0.5 * max_dec * t3 * t3);     // distance covered at constant deceleration
            v3 = v2 - (max_dec * t3);       // velocity left for the final jerk phase

            // Phase 4: acceleration -max_dec -> 0 (with jerk), velocity drops exactly to zero
            x4 = (0.1666667 * max_dec * t2 * t2);

            brake_distance = x1_abs + x2 + x3 + x4;     // trapezoidal profile
        }
        else
        {
            // we never reach max_dec, triangular profile
            t2 = sqrt(v1_abs / jerk);       // duration of each jerk half
            x2 = v1_abs * t2;       // total distance of both halves

            brake_distance = x1_abs + x2;
        }
    }
    else if (cur_acc * cur_vel < -TOLERANCE)     // acceleration and velocity opposite directions ----- so we are slowing down
    {
        // we are already braking, everything below runs on magnitudes so brake_distance comes out positive
        double vel_critic;      // velocity that will be lost while pulling the acceleration to zero
        t3 = acc_abs / jerk ;       // time it takes to zero the acceleration
        vel_critic = (acc_abs * t3) - (0.5 * jerk * t3 * t3);       // velocity calculation, magnitude like vel_abs

        if (vel_abs > (vel_critic + TOLERANCE))       // if the velocity is enough to stop the acceleration
        {
            // we need to decide what kind of acceleration profile it has to draw.
            if (acc_abs < (max_dec - TOLERANCE))
            {
                // we are under the deceleration limit, jerk can still push the deceleration up
                t1 = (max_dec - acc_abs) / jerk;        // time to raise the deceleration up to max_dec
                v1 = vel_abs - (acc_abs * t1) - (0.5 * jerk * t1 * t1);     // velocity left once we sit on max_dec

                if (v1 > ((0.5 * max_dec * max_dec / jerk) + TOLERANCE))        // is there velocity left over the final jerk phase?
                {
                    // trapezoidal, max_dec is reached and held for a while
                    x1 = (vel_abs * t1) - (0.5 * acc_abs * t1 * t1) - (0.1666667 * jerk * t1 * t1 * t1);     // distance covered while the deceleration is raised

                    t2 = (v1 - (0.5 * max_dec * max_dec / jerk)) / max_dec;     // time spent at constant max_dec
                    x2 = (v1 * t2) - (0.5 * max_dec * t2 * t2);     // distance covered at constant deceleration
                    v2 = v1 - (max_dec * t2);       // velocity left for the final jerk phase

                    t3 = max_dec / jerk;        // time for deceleration max_dec -> 0
                    x3 = (0.1666667 * max_dec * t3 * t3);       // velocity drops exactly to zero here

                    brake_distance = x1 + x2 + x3;
                }
                else
                {
                    // triangular, max_dec is never reached, the deceleration peaks below it
                    double acc_peak = sqrt((jerk * vel_abs) + (0.5 * acc_abs * acc_abs));       // highest deceleration this profile reaches

                    t1 = (acc_peak - acc_abs) / jerk;       // time to raise the deceleration up to the peak
                    x1 = (vel_abs * t1) - (0.5 * acc_abs * t1 * t1) - (0.1666667 * jerk * t1 * t1 * t1);     // distance covered while the deceleration is raised
                    v1 = vel_abs - (acc_abs * t1) - (0.5 * jerk * t1 * t1);     // velocity left at the peak

                    t2 = acc_peak / jerk;       // time for deceleration acc_peak -> 0
                    x2 = (0.1666667 * acc_peak * t2 * t2);      // velocity drops exactly to zero here

                    brake_distance = x1 + x2;
                }
            }
            else if (acc_abs > (max_dec + TOLERANCE))
            {
                // we are over the deceleration limit, jerk has to pull the deceleration back down to max_dec
                t1 = (acc_abs - max_dec) / jerk;        // time to lower the deceleration down to max_dec
                x1 = (vel_abs * t1) - (0.5 * acc_abs * t1 * t1) + (0.1666667 * jerk * t1 * t1 * t1);        // distance covered while the deceleration is lowered
                v1 = vel_abs - (acc_abs * t1) + (0.5 * jerk * t1 * t1);     // velocity left once we sit on max_dec

                // vel_abs > vel_critic already guarantees v1 > 0.5*max_dec^2/jerk, so max_dec is always held
                t2 = (v1 - (0.5 * max_dec * max_dec / jerk)) / max_dec;     // time spent at constant max_dec
                x2 = (v1 * t2) - (0.5 * max_dec * t2 * t2);     // distance covered at constant deceleration
                v2 = v1 - (max_dec * t2);       // velocity left for the final jerk phase

                t3 = max_dec / jerk;        // time for deceleration max_dec -> 0
                x3 = (0.1666667 * max_dec * t3 * t3);       // velocity drops exactly to zero here

                brake_distance = x1 + x2 + x3;
            }
            else
            {
                // we already sit exactly on max_dec, no jerk phase needed to get there
                t2 = (vel_abs - (0.5 * max_dec * max_dec / jerk)) / max_dec;        // time spent at constant max_dec
                x2 = (vel_abs * t2) - (0.5 * max_dec * t2 * t2);        // distance covered at constant deceleration
                v2 = vel_abs - (max_dec * t2);      // velocity left for the final jerk phase

                t3 = max_dec / jerk;        // time for deceleration max_dec -> 0
                x3 = (0.1666667 * max_dec * t3 * t3);       // velocity drops exactly to zero here

                brake_distance = x2 + x3;
            }
        }
        else if (vel_abs < (vel_critic - TOLERANCE))      // if the velocity is NOT enough to stop the acceleration
        {
            // we are over braking, the velocity hits zero before the deceleration can be zeroed
            double t_root1, t_root2;        // 0 = vel_abs - acc_abs*t + 0.5*jerk*t^2
            int solved = solveQuadratic((0.5 * jerk), -acc_abs, vel_abs, &t_root1, &t_root2);

            if (solved)
            {
                t1 = dmin(t_root1, t_root2);        // the first time the velocity reaches zero
            }
            else
            {
                t1 = acc_abs / jerk;        // cannot happen while vel_abs < vel_critic, the discriminant stays positive
            }

            x1 = (vel_abs * t1) - (0.5 * acc_abs * t1 * t1) + (0.1666667 * jerk * t1 * t1 * t1);     // distance covered until standstill

            brake_distance = x1;
        }
        else        // if the velocity is exactly enough to stop the acceleration
        {
            // zeroing the deceleration lands the velocity exactly on zero, nothing else to do
            t1 = acc_abs / jerk;        // time for deceleration acc_abs -> 0
            x1 = (0.1666667 * acc_abs * t1 * t1);       // velocity drops exactly to zero here

            brake_distance = x1;
        }
    }
    else        // velocity or acceleration is zero ----- constant velocity or standing still
    {
        if ((vel_abs < TOLERANCE) && (acc_abs < TOLERANCE))
        {
            // standing still, there is nothing to brake
            brake_distance = 0.0;
        }
        else if (acc_abs < TOLERANCE)
        {
            // constant velocity, the braking profile starts straight from vel_abs
            if (vel_abs > ((max_dec * max_dec / jerk) + TOLERANCE))     // is max_dec reached at all?
            {
                t1 = max_dec / jerk;        // time for acceleration 0 -> -max_dec
                x1 = (vel_abs * t1) - (0.1666667 * jerk * t1 * t1 * t1);        // distance covered while the deceleration is raised
                v1 = vel_abs - (0.5 * jerk * t1 * t1);      // velocity left once we sit on max_dec

                t2 = (v1 - (0.5 * max_dec * max_dec / jerk)) / max_dec;     // time spent at constant max_dec
                x2 = (v1 * t2) - (0.5 * max_dec * t2 * t2);     // distance covered at constant deceleration

                t3 = max_dec / jerk;        // time for deceleration max_dec -> 0
                x3 = (0.1666667 * max_dec * t3 * t3);       // velocity drops exactly to zero here

                brake_distance = x1 + x2 + x3;      // trapezoidal profile
            }
            else
            {
                // we never reach max_dec, triangular profile
                t1 = sqrt(vel_abs / jerk);      // duration of each jerk half
                x1 = vel_abs * t1;      // total distance of both halves

                brake_distance = x1;
            }
        }
        else
        {
            // standing still but the acceleration is not zero, the axis is about to run away.
            // the jerk has to kill that acceleration first and the axis picks up velocity on the way
            t1 = acc_abs / jerk;        // time it takes to zero the current acceleration
            x1 = (0.3333333 * acc_abs * t1 * t1);       // distance covered while the acceleration is zeroed
            v1 = (0.5 * acc_abs * t1);      // velocity picked up on the way, magnitude

            if (v1 > ((max_dec * max_dec / jerk) + TOLERANCE))      // is max_dec reached at all?
            {
                t2 = max_dec / jerk;        // time for acceleration 0 -> -max_dec
                x2 = (v1 * t2) - (0.1666667 * jerk * t2 * t2 * t2);     // distance covered while the deceleration is raised
                v2 = v1 - (0.5 * jerk * t2 * t2);       // velocity left once we sit on max_dec

                t3 = (v2 - (0.5 * max_dec * max_dec / jerk)) / max_dec;     // time spent at constant max_dec
                x3 = (v2 * t3) - (0.5 * max_dec * t3 * t3);     // distance covered at constant deceleration

                x4 = (0.1666667 * max_dec * t2 * t2);       // velocity drops exactly to zero here

                brake_distance = x1 + x2 + x3 + x4;     // trapezoidal profile
            }
            else
            {
                // we never reach max_dec, triangular profile
                t2 = sqrt(v1 / jerk);       // duration of each jerk half
                x2 = v1 * t2;       // total distance of both halves

                brake_distance = x1 + x2;
            }
        }
    }

    (void)v3;       // written by the trapezoidal branch, kept so the phase list stays complete

    return brake_distance;
}

// the external entry, everything inside this file calls the static one above
double BrakeDistance(double cur_vel, double cur_acc, double max_dec, double jerk)
{
    return CalcBrakeDistance(cur_vel, cur_acc, max_dec, jerk);
}

// the scan both modes share. everything here works in the direction frame the caller picked, and
// the sign is put back on the way out. vel_bound is what the settling velocity may not exceed in
// that frame; diff_pos is the distance left to the target, or negative when there is no target
static TrajectoryStep ScanStep(const MotionState *state, const MotionLimits *limits, double deltaTime,
                               double dir, double vel_bound, double diff_pos,
                               double max_acc, double max_dec, double jerk)
{
    TrajectoryStep step;
    step.State = *state;
    step.Jerk = 0.0;
    step.BrakeDistance = 0.0;
    step.InPosition = 0;
    step.InVelocity = 0;

    const double cur_pos = state->Position;
    const double vel_dir = state->Velocity * dir;       // velocity in the direction the scan is solved in
    const double acc_dir = state->Acceleration * dir;

    // every limit is a statement about the state the axis will be in at the END of this scan, not
    // the one it is in now. tested on the current state every switch lands a scan late, and what one
    // late scan costs cannot be given back - max_acc * deltaTime of velocity at the limit, the
    // matching ground at the target. so the jerk itself is searched: the largest one whose end of
    // scan state still honours every limit
    const double acc_limit_up = (vel_dir < -TOLERANCE) ? max_dec : max_acc;     // an acceleration along the frame speeds the axis up unless it is still running the other way
    const double acc_limit_down = (vel_dir > TOLERANCE) ? max_dec : max_acc;    // one against it slows the axis down unless it is still running the other way
    const double acc_floor = -acc_limit_down;

    // software limits are honoured by asking where the axis comes to rest, not where it is now
    const int limits_on = (limits->PositiveLimit > limits->NegativeLimit);
    const double neg_limit = limits->NegativeLimit;
    const double pos_limit = limits->PositiveLimit;
    const double breach_now = limits_on ? dmax(0.0, dmax(neg_limit - cur_pos, cur_pos - pos_limit)) : 0.0;      // how far outside it already is

    // loop invariants of the three polynomials below, hoisted without regrouping anything: the
    // originals summed left to right, so splitting off the leading terms keeps every bit identical
    const int has_target = (diff_pos >= 0.0);
    const int needs_brake = (limits_on || has_target);       // otherwise the brake distance is never asked for
    const double vel_base = vel_dir + (acc_dir * deltaTime);
    const double pos_base = (vel_dir * deltaTime) + (0.5 * acc_dir * deltaTime * deltaTime);
    const double two_jerk = 2.0 * jerk;

    double jerk_lo = -jerk;     // the fallback below covers the case where even this is not feasible
    double jerk_hi = jerk;
    int feasible = 0;

    for (int i = 0; i < BISECTION_STEPS; i++)
    {
        const double jerk_try = (0.5 * (jerk_lo + jerk_hi));
        const double acc_end = acc_dir + (jerk_try * deltaTime);
        const double vel_end = vel_base + (0.5 * jerk_try * deltaTime * deltaTime);
        const double pos_end = pos_base + (0.1666667 * jerk_try * deltaTime * deltaTime * deltaTime);

        int ok = ((acc_end <= acc_limit_up) && (acc_end >= acc_floor))
                 && ((vel_end + ((acc_end * fabs(acc_end)) / two_jerk)) <= vel_bound);      // where the velocity settles once that acceleration is nulled

        if (ok && needs_brake)
        {
            const double brake_end = CalcBrakeDistance(vel_end, acc_end, max_dec, jerk);

            if (has_target)     // there is a target to stop on
            {
                const double diff_end = diff_pos - pos_end;     // distance still left to it after this scan
                ok = ((diff_end >= 0.0) && (brake_end <= diff_end));
            }

            if (ok && limits_on)        // where it would come to rest, signed and back in world coordinates
            {
                const double stop_pos = cur_pos + (dir * (pos_end + ((vel_end < 0.0) ? -brake_end : brake_end)));
                const double breach_end = dmax(0.0, dmax(neg_limit - stop_pos, stop_pos - pos_limit));

                // inside the limits, or at least closer to them than it already is - the second half
                // is what lets an axis that starts outside drive back in instead of being frozen
                ok = ((breach_end <= 0.0) || (breach_end < breach_now));
            }
        }

        if (ok)
        {
            jerk_lo = jerk_try;
            feasible = 1;
        }
        else
        {
            jerk_hi = jerk_try;
        }
    }

    // every midpoint passed, so the boundary sits above the limit and the limit itself is feasible.
    // without this the search only ever converges towards it and the jerk output never reads exactly
    // +Jerk in the phases where it is saturated
    double jerk_cmd = (jerk_hi >= jerk) ? jerk : jerk_lo;

    if (!feasible)      // nothing this scan can do keeps the axis inside every limit, so relieve the one that is already broken
    {
        const double settle_now = vel_dir + ((acc_dir * fabs(acc_dir)) / two_jerk);

        if (settle_now > vel_bound)     // already past the velocity it is allowed to settle at: pull that down, hardest first
        {
            jerk_cmd = -jerk;
        }
        else if (vel_dir > 0.0)         // otherwise it is the distance that cannot be held, so brake the motion it has
        {
            jerk_cmd = -jerk;
        }
        else if (vel_dir < 0.0)
        {
            jerk_cmd = jerk;
        }
        else
        {
            jerk_cmd = (acc_dir > 0.0) ? -jerk : jerk;      // standing still, kill the acceleration
        }
    }

    double acc_next = acc_dir + (jerk_cmd * deltaTime);     // the acceleration this scan ends with
    acc_next = dmax(acc_floor, dmin(acc_limit_up, acc_next));
    acc_next = dmax(acc_dir - (jerk * deltaTime), dmin(acc_dir + (jerk * deltaTime), acc_next));     // a state handed in past the limit is walked back at the jerk limit, never jumped
    jerk_cmd = (acc_next - acc_dir) / deltaTime;        // what really gets integrated, so the polynomials stay exact where the clamp bites

    const double vel_next = vel_dir + (acc_dir * deltaTime) + (0.5 * jerk_cmd * deltaTime * deltaTime);

    step.State.Position = cur_pos + (dir * ((vel_dir * deltaTime) + (0.5 * acc_dir * deltaTime * deltaTime) + (0.1666667 * jerk_cmd * deltaTime * deltaTime * deltaTime)));
    step.State.Velocity = dir * vel_next;
    step.State.Acceleration = dir * acc_next;
    step.Jerk = dir * jerk_cmd;

    return step;
}

// the limits have to be ordered or the profile degenerates: with the jerk under the acceleration
// limits the acceleration ramp alone outlasts the move, with the acceleration limits under the
// velocity one the velocity ramp does, and the axis never takes a single proper step. ordered into
// locals only, the caller's MotionLimits is const and stays as it was handed in
static int OrderLimits(const MotionLimits *limits, double *max_acc, double *max_dec, double *jerk)
{
    *max_acc = dmax(limits->MaxAcceleration, limits->MaxVelocity);
    *max_dec = dmax(limits->MaxDeceleration, limits->MaxVelocity);
    *jerk = dmax(limits->Jerk, dmax(*max_acc, *max_dec));

    return ((limits->MaxVelocity > 0.0) && (*jerk > 0.0));
}

TrajectoryStep GenerateTrajectory(const MotionState *MOTION_RESTRICT state,
                                  const MotionLimits *MOTION_RESTRICT limits,
                                  const MotionCommand *MOTION_RESTRICT command,
                                  double deltaTime)
{
    TrajectoryStep step;
    step.State = *state;        // nothing moves unless the scan below says so
    step.Jerk = 0.0;
    step.BrakeDistance = 0.0;
    step.InPosition = 0;
    step.InVelocity = 0;

    double max_acc, max_dec, jerk;

    if (!OrderLimits(limits, &max_acc, &max_dec, &jerk))
    {
        return step;        // unusable limits, the axis cannot be commanded at all
    }

    const double brake_distance = CalcBrakeDistance(state->Velocity, state->Acceleration, max_dec, jerk);

    step.BrakeDistance = brake_distance;        // of the state this scan was entered with

    // the brake distance above belongs to the state this scan started with, so the axis is
    // only walked forward after that value has been taken
    if (deltaTime <= 0.0)
    {
        return step;
    }

    double dir;                     // the direction the scan is solved in
    double vel_bound;               // the settling velocity may not pass this, in that frame
    double diff_pos = -1.0;         // distance left to the target, negative when there is no target

    if (command->Type == MOTION_COMMAND_POSITION)
    {
        diff_pos = fabs(command->Target - state->Position);

        // in position: close enough, slow enough to stop inside the window as well, and with an
        // acceleration small enough that parking it is not a jerk step
        if ((diff_pos < limits->InPositionWindow) && (brake_distance < limits->InPositionWindow)
            && (fabs(state->Acceleration) <= (jerk * deltaTime)))
        {
            step.State.Velocity = 0.0;
            step.State.Acceleration = 0.0;
            step.InPosition = 1;
            return step;
        }

        dir = (command->Target < state->Position) ? -1.0 : 1.0;
        vel_bound = limits->MaxVelocity;
    }
    else
    {
        const double vel_cmd = dmax(-limits->MaxVelocity, dmin(limits->MaxVelocity, command->Target));      // a command above the axis limit is not honoured

        // at velocity: the same idea as the in position window, one axis up. without it the settling
        // velocity only ever converges towards the command and the acceleration never quite reaches
        // zero. it may only be taken while the software limits still allow another scan at that
        // velocity - parking returns early, so anything skipped here is not checked at all
        int may_park = ((fabs(state->Velocity - vel_cmd) < limits->InVelocityWindow)
                        && (fabs(state->Acceleration) <= (jerk * deltaTime)));

        if (may_park && (limits->PositiveLimit > limits->NegativeLimit))
        {
            const double brake_cmd = CalcBrakeDistance(vel_cmd, 0.0, max_dec, jerk);
            const double stop_pos = state->Position + (vel_cmd * deltaTime) + ((vel_cmd < 0.0) ? -brake_cmd : brake_cmd);
            may_park = ((stop_pos >= limits->NegativeLimit) && (stop_pos <= limits->PositiveLimit));
        }

        if (may_park)
        {
            step.State.Position = state->Position + (vel_cmd * deltaTime);
            step.State.Velocity = vel_cmd;
            step.State.Acceleration = 0.0;
            step.InVelocity = 1;
            return step;
        }

        // solved in the direction the velocity has to move in, so "do not pass the command" stays the
        // same one sided test the position mode uses against max_vel. there is no target to stop on,
        // so diff_pos stays negative and only the software limits bound the travel - in this mode
        // they are the only thing that ever brings the axis to a stop
        dir = (vel_cmd < state->Velocity) ? -1.0 : 1.0;
        vel_bound = vel_cmd * dir;
    }

    step = ScanStep(state, limits, deltaTime, dir, vel_bound, diff_pos, max_acc, max_dec, jerk);
    step.BrakeDistance = brake_distance;

    return step;
}
