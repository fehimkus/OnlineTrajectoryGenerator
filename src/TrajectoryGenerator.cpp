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
    const double diff_pos = std::abs(axis.TargetPosition - axis.CurrentPosition);
    const double dir = (axis.TargetPosition < axis.CurrentPosition) ? -1.0 : 1.0;      // direction of travel, the motion below is solved in it and the sign put back at the end
    const double vel_dir = cur_vel * dir;       // velocity towards the target
    const double acc_dir = cur_acc * dir;       // acceleration towards the target
    const double brake_velocity = max_vel - ((acc_dir * acc_abs) / (2.0 * jerk));      // velocity threshold: above this the jerk has to be eased off or max_vel is overshot
    
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
                t1 = std::sqrt(vel_abs / jerk);     // duration of each jerk half
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
                t2 = std::sqrt(v1 / jerk);      // duration of each jerk half
                x2 = v1 * t2;       // total distance of both halves

                brake_distance = x1 + x2;
            }
        }
    }

    axis.BrakeDistance = brake_distance;        // handed out so the UI can read it every scan
    axis.CommandedJerk = 0.0;
    axis.VelocityCommand = 0.0;

    


    // ---------------------------------------------------------------- the motion itself
    // the brake distance above belongs to the state this scan started with, so the axis is
    // only walked forward after that value has been handed out

    if (deltaTime <= 0.0)
    {
        return;
    }

    // in position: close enough, slow enough to stop inside the window as well, and with an
    // acceleration small enough that parking it is not a jerk step. without this the axis can
    // never settle - the envelope is a cube root of the remaining distance, so its slope runs
    // away at the target and a finite scan can only ever hunt around it
    if ((diff_pos < axis.InPositionWindow) && (brake_distance < axis.InPositionWindow) && (acc_abs <= (jerk * deltaTime)))
    {
        axis.CurrentVelocity = 0.0;
        axis.CurrentAcceleration = 0.0;
        axis.CommandedJerk = 0.0;
        axis.VelocityCommand = 0.0;
        return;
    }

    if(diff_pos > TOLERANCE)
    {
        const double acc_limit_up = (vel_dir < -TOLERANCE) ? max_dec : max_acc;     // an acceleration towards the target speeds the axis up unless it is still running the other way
        const double acc_limit_down = (vel_dir > TOLERANCE) ? max_dec : max_acc;    // one away from the target slows it down unless it is still running the other way

        // one velocity command, no accelerate / brake split: the envelope carries both, and a
        // branch test sitting on its own switching surface is what made the two fight.
        // the envelope only knows how to brake from a standing acceleration, so the ground covered
        // while the jerk nulls the one the axis has comes off the remaining distance first
        double t_null = acc_abs / jerk;     // time the jerk needs to null the acceleration
        const double jerk_null = (acc_dir < 0.0) ? jerk : -jerk;        // it always opposes the acceleration

        if ((acc_dir < 0.0) && (vel_dir > 0.0) && (vel_dir < ((acc_abs * acc_abs) / (2.0 * jerk))))
        {
            double t_root1, t_root2;        // over braking: the velocity reaches zero before the acceleration does, so the ramp never completes
            if (solveQuadratic((0.5 * jerk), -acc_abs, vel_dir, t_root1, t_root2))
            {
                t_null = t_root2;       // smaller root, standstill
            }
        }

        const double x_null = (vel_dir * t_null) + (0.5 * acc_dir * t_null * t_null) + (0.1666667 * jerk_null * t_null * t_null * t_null);       // distance covered while that happens, signed
        const double diff_pos_zero = diff_pos - x_null;     // what is left of the distance at that point

        double vel_allowed;     // the velocity that distance permits

        if (diff_pos_zero <= 0.0)       // the target is gone before the acceleration can be nulled
        {
            vel_allowed = 0.0;
        }
        else if (diff_pos_zero > ((max_dec * max_dec * max_dec) / (jerk * jerk)))        // max_dec is reached on the way down, trapezoidal brake
        {
            double vel_root1, vel_root2;
            solveQuadratic((1.0 / (2.0 * max_dec)), (max_dec / (2.0 * jerk)), -diff_pos_zero, vel_root1, vel_root2);      // inverse of diff_pos_zero = v*v/(2*max_dec) + v*max_dec/(2*jerk)
            vel_allowed = vel_root1;        // larger root, the positive one
        }
        else        // max_dec is never reached, triangular brake
        {
            vel_allowed = std::cbrt(diff_pos_zero * diff_pos_zero * jerk);      // inverse of diff_pos_zero = v^1.5 / sqrt(jerk)
        }

        const double vel_command = std::min(vel_allowed, max_vel);      // the envelope can never permit more than the limit

        // peak acceleration of the ramp that takes (vel_dir, acc_dir) to (vel_command, 0)
        const double delta_vel = vel_command - vel_dir;
        const double delta_vel_zero = (acc_dir * acc_abs) / (2.0 * jerk);       // velocity still picked up while the acceleration is nulled

        double acc_target;      // where the acceleration wants to be this scan

        if (delta_vel > delta_vel_zero)     // the command is above where nulling the acceleration lands, so speed up
        {
            acc_target = std::sqrt((jerk * delta_vel) + (0.5 * acc_dir * acc_dir));
        }
        else        // it is below, so slow down
        {
            acc_target = -std::sqrt((0.5 * acc_dir * acc_dir) - (jerk * delta_vel));
        }

        const double acc_exact = ((2.0 * delta_vel) / deltaTime) - acc_dir;     // the acceleration that puts the velocity exactly on the command at the end of this scan

        if (delta_vel > 0.0)        // never ask for more than that, the square roots have unbounded slope at delta_vel = 0
        {
            acc_target = std::min(acc_target, acc_exact);
        }
        else
        {
            acc_target = std::max(acc_target, acc_exact);
        }

        if (diff_pos_zero <= 0.0)       // the acceleration can no longer be nulled before the target, braking is all that is left
        {
            acc_target = -acc_limit_down;       // the ramp law would read the standing deceleration as "too much" and release it
        }

        acc_target = std::max(-acc_limit_down, std::min(acc_limit_up, acc_target));     // the limit follows the speed, not the sign of the acceleration

        double acc_next;        // the acceleration this scan ends with

        if (acc_target > acc_dir)       // jerk walks towards the target and stops exactly on it
        {
            acc_next = std::min(acc_target, acc_dir + (jerk * deltaTime));
        }
        else
        {
            acc_next = std::max(acc_target, acc_dir - (jerk * deltaTime));
        }

        const double jerk_cmd = (acc_next - acc_dir) / deltaTime;       // what really gets integrated, so the polynomials stay exact where the clamps bite

        axis.VelocityCommand = dir * vel_command;       // what this scan aimed at
        axis.CurrentPosition = cur_pos + (dir * ((vel_dir * deltaTime) + (0.5 * acc_dir * deltaTime * deltaTime) + (0.1666667 * jerk_cmd * deltaTime * deltaTime * deltaTime)));
        axis.CurrentVelocity = dir * (vel_dir + (acc_dir * deltaTime) + (0.5 * jerk_cmd * deltaTime * deltaTime));
        axis.CurrentAcceleration = dir * acc_next;
        axis.CommandedJerk = dir * jerk_cmd;
    }
    else        // the target is inside the tolerance ----- the axis is there, park it
    {
        axis.CurrentPosition = axis.TargetPosition;
        axis.CurrentVelocity = 0.0;
        axis.CurrentAcceleration = 0.0;
        axis.CommandedJerk = 0.0;
        axis.VelocityCommand = 0.0;
    }
}
