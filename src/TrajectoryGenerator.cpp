#include "TrajectoryGenerator.h"

#include <algorithm>
#include <cmath>

// quadratic equation solver, returns true if a real root exists
bool solveQuadratic(double a, double b, double c, double &x1, double &x2)
{
    double discriminant = b * b - 4 * a * c;    // is there a root

    if (discriminant < 0)
    {
        return false;       // no real root
    }

    double sqrt_discriminant = std::sqrt(discriminant);
    double denom = 2 * a;       // note: no a = 0 check

    x1 = (-b + sqrt_discriminant) / denom;      // larger root
    x2 = (-b - sqrt_discriminant) / denom;      // smaller root

    return true;
}

void GenerateTrajectory(Axis& axis, double deltaTime)
{
    const double max_vel = axis.MaxVelocity;
    const double max_dec = axis.MaxDeceleration;
    const double max_acc = axis.MaxAcceleration;
    const double jerk = axis.Jerk;

    // precompute for jerk decision
    const double cur_pos = axis.CurrentPosition;
    const double cur_acc = axis.CurrentAcceleration;
    const double cur_vel = axis.CurrentVelocity;
    const double acc_abs = std::abs(cur_acc);
    const double vel_abs = std::abs(cur_vel);

    double brake_distance = 0.0;        // distance i would cover if i braked right now
    double t1, t2, t3;          // phase durations
    double x1, x2, x3, x4;      // distance covered in each phase
    double v1, v2, v3;          // velocity at the end of each phase
    double signvelocity;        // jerk is applied opposite to the velocity

    if (cur_acc * cur_vel > TOLERANCE)      // acceleration and velocity same direction ----- so we are speeding up
    {
        // first the speeding up has to be stopped, that is the acceleration has to be zeroed.
        cur_vel < 0.0 ? signvelocity = 1 : signvelocity = -1;       // velocity direction matters for the jerk
        t1 = acc_abs / jerk;        // time it takes to zero the current acceleration
        x1 = (cur_vel * t1) + (0.5 * cur_acc * t1 * t1) + (signvelocity / 6.0) * jerk * t1 * t1 * t1;      //distance covered while the acceleration is zeroed
        v1 = cur_vel + (cur_acc * t1) + (0.5 * signvelocity * jerk * t1 * t1);      // velocity reached once the acceleration is zero

        const double x1_abs = std::abs(x1);     // the braking phases below run on magnitudes, same as the slowing down branch
        const double v1_abs = std::abs(v1);

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
            t2 = std::sqrt(v1_abs / jerk);      // duration of each jerk half
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
                    double acc_peak = std::sqrt((jerk * vel_abs) + (0.5 * acc_abs * acc_abs));       // highest deceleration this profile reaches

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
            bool solved = solveQuadratic((0.5 * jerk), -acc_abs, vel_abs, t_root1, t_root2);

            if (solved)
            {
                t1 = std::min(t_root1, t_root2);        // the first time the velocity reaches zero
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
        // TODO: if v != 0 plain braking profile, if v == 0 brake_distance = 0

    }

    axis.BrakeDistance = brake_distance;        // handed out so the UI can read it every scan

    // ---------------------------------------------------------------- the motion itself
    // the brake distance above belongs to the state this scan started with, so the axis is
    // only walked forward after that value has been handed out

    if (axis.CurrentState != AxisState::Stopping)
    {
        return;     // nothing else is driven yet
    }

    if (deltaTime <= 0.0)
    {
        return;
    }

    const double dir = (cur_vel < 0.0) ? -1.0 : 1.0;        // the direction we are travelling in
    const double dec_abs = -cur_acc * dir;      // deceleration magnitude, negative while we are still speeding up

    // ---- DECELERATION TARGET ----
    double dec_target;

    if ((dec_abs > 0.0) && (((dec_abs * dec_abs) / (2.0 * jerk)) >= vel_abs))
    {
        dec_target = 0.0;       // what we already have eats the whole velocity, let the deceleration go
    }
    else
    {
        dec_target = max_dec;       // brake as hard as the limit allows
    }

    // ---- DECELERATION UPDATE ----
    double new_dec = dec_abs;       // jerk walks the deceleration towards the target and stops there

    if (dec_target > dec_abs)
    {
        new_dec = dec_abs + (jerk * deltaTime);
        if (new_dec > dec_target)
        {
            new_dec = dec_target;
        }
    }
    else if (dec_target < dec_abs)
    {
        new_dec = dec_abs - (jerk * deltaTime);
        if (new_dec < dec_target)
        {
            new_dec = dec_target;
        }
    }

    const double new_acc_full = -new_dec * dir;
    double jerk_cmd = (new_acc_full - cur_acc) / deltaTime;     // jerk that is really applied after the clamp, keeps the integration exact

    // ---- VELOCITY / POSITION UPDATE ----
    double step = deltaTime;        // how much of this scan we actually travel
    double new_vel = cur_vel + (cur_acc * deltaTime) + (0.5 * jerk_cmd * deltaTime * deltaTime);
    bool standstill = false;

    if ((new_vel * dir) <= 0.0)     // we run through standstill inside this scan
    {
        standstill = true;

        if (std::abs(jerk_cmd) < TOLERANCE)
        {
            step = -cur_vel / cur_acc;      // constant acceleration, plain linear solution
        }
        else
        {
            double t_root1, t_root2;        // 0 = cur_vel + cur_acc*t + 0.5*jerk_cmd*t^2
            bool solved = solveQuadratic((0.5 * jerk_cmd), cur_acc, cur_vel, t_root1, t_root2);

            if (solved)
            {
                double t_small = std::min(t_root1, t_root2);
                double t_large = std::max(t_root1, t_root2);
                step = (t_small > 0.0) ? t_small : t_large;     // the first crossing that lies ahead of us
            }
        }

        step = std::clamp(step, 0.0, deltaTime);
        new_vel = 0.0;
    }

    axis.CurrentPosition = cur_pos + (cur_vel * step) + (0.5 * cur_acc * step * step) +
                           (0.1666667 * jerk_cmd * step * step * step);
    axis.CurrentVelocity = new_vel;
    axis.CurrentAcceleration = cur_acc + (jerk_cmd * step);

    if (standstill)
    {
        axis.CurrentVelocity = 0.0;
        axis.CurrentAcceleration = 0.0;     // over braking leaves a leftover deceleration, it dies with the motion
        axis.CurrentState = AxisState::Idle;
    }
}
