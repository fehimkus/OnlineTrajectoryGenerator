#include "Profilerold.h"

#include <cmath>

bool Profiler::solveQuadratic(double a, double b, double c, double &x1, double &x2)
{
    if (std::fabs(a) < PROFILER_TOLERANCE)          // degenerates to b*x + c = 0
    {
        if (std::fabs(b) < PROFILER_TOLERANCE)
        {
            return false;
        }

        x1 = (-c / b);
        x2 = x1;
        return true;
    }

    const double discriminant = ((b * b) - (4.0 * a * c));

    if (discriminant < 0.0)
    {
        return false;
    }

    const double sqrt_discriminant = std::sqrt(discriminant);

    // stable form, the naive (-b + sqrt) cancels when b is large next to sqrt(discriminant)
    const double q = (b >= 0.0) ? (-0.5 * (b + sqrt_discriminant))
                                : (-0.5 * (b - sqrt_discriminant));

    x1 = (q / a);
    x2 = (c / q);

    return true;
}

bool Profiler::solve2x2System(const double A[2][2], const double b[2], double x[2])
{
    const double det = ((A[0][0] * A[1][1]) - (A[0][1] * A[1][0]));

    if (std::fabs(det) < 1e-12)
    {
        x[0] = 0.0;
        x[1] = 0.0;
        return false;
    }

    const double inv_det = (1.0 / det);

    x[0] = (((A[1][1] * b[0]) - (A[0][1] * b[1])) * inv_det);
    x[1] = (((A[0][0] * b[1]) - (A[1][0] * b[0])) * inv_det);

    return true;
}

void Profiler::BuildRamp(double v_start, double a_start, double v_end,
                         double &ta, double &tb, double &tc_out,
                         double &ja, double &jc_out,
                         double &xa, double &xb, double &xc_out,
                         double &va, double &vb, double &vc_out,
                         double &aa, double &ab, double &ac_out) const
{
    const double delta_v = (v_end - v_start);
    const double delta_v_zero = ((a_start * std::fabs(a_start)) / (2.0 * Jerk));   // velocity gained while nulling a_start

    double a_peak = 0.0;

    // the ramp has to speed up, the peak acceleration is positive and bounded by MaxAcceleration
    if (delta_v > delta_v_zero)
    {
        a_peak = std::sqrt((Jerk * delta_v) + (0.5 * a_start * a_start));

        if (a_peak > MaxAcceleration)
        {
            a_peak = MaxAcceleration;
        }
    }
    // the ramp has to slow down, the peak acceleration is negative and bounded by MaxDeceleration
    else if (delta_v < delta_v_zero)
    {
        a_peak = -std::sqrt((0.5 * a_start * a_start) - (Jerk * delta_v));

        if (a_peak < -MaxDeceleration)
        {
            a_peak = -MaxDeceleration;
        }
    }
    // nulling a_start already lands exactly on v_end
    else
    {
        a_peak = 0.0;
    }

    // first sub phase, a_start -> a_peak
    const double sign_a = (a_peak >= a_start) ? 1.0 : -1.0;
    ja = (sign_a * Jerk);
    ta = ((a_peak - a_start) / ja);
    const double delta_v_a = (sign_a * (((a_peak * a_peak) - (a_start * a_start)) / (2.0 * Jerk)));

    // last sub phase, a_peak -> 0
    const double sign_c = (a_peak >= 0.0) ? -1.0 : 1.0;
    jc_out = (sign_c * Jerk);
    tc_out = ((0.0 - a_peak) / jc_out);
    const double delta_v_c = (-sign_c * ((a_peak * a_peak) / (2.0 * Jerk)));

    // middle sub phase holds a_peak for whatever velocity the two jerk phases could not cover
    if (std::fabs(a_peak) < PROFILER_TOLERANCE)
    {
        tb = 0.0;
    }
    else
    {
        tb = ((delta_v - delta_v_a - delta_v_c) / a_peak);

        if (tb < 0.0)                                // the triangular case leaves nothing for the constant phase
        {
            tb = 0.0;
        }
    }

    xa = ((v_start * ta) + (0.5 * a_start * ta * ta) + (0.1666667 * ja * ta * ta * ta));
    va = (v_start + (a_start * ta) + (0.5 * ja * ta * ta));
    aa = (a_start + (ja * ta));

    xb = ((va * tb) + (0.5 * aa * tb * tb));
    vb = (va + (aa * tb));
    ab = aa;

    xc_out = ((vb * tc_out) + (0.5 * ab * tc_out * tc_out) + (0.1666667 * jc_out * tc_out * tc_out * tc_out));
    vc_out = (vb + (ab * tc_out) + (0.5 * jc_out * tc_out * tc_out));
    ac_out = (ab + (jc_out * tc_out));
}

double Profiler::ProfileDistance(double v_peak, double v_start, double a_start) const
{
    double ta = 0.0, tb = 0.0, tc_local = 0.0;
    double ja = 0.0, jc_local = 0.0;
    double xa = 0.0, xb = 0.0, xc_local = 0.0;
    double va = 0.0, vb = 0.0, vc_local = 0.0;
    double aa = 0.0, ab = 0.0, ac_local = 0.0;

    BuildRamp(v_start, a_start, v_peak,
              ta, tb, tc_local, ja, jc_local,
              xa, xb, xc_local, va, vb, vc_local, aa, ab, ac_local);

    const double rising = (xa + xb + xc_local);

    BuildRamp(v_peak, 0.0, 0.0,
              ta, tb, tc_local, ja, jc_local,
              xa, xb, xc_local, va, vb, vc_local, aa, ab, ac_local);

    const double falling = (xa + xb + xc_local);

    return (rising + falling);
}

void Profiler::CalculatePositionProfile()
{
    Valid = false;
    Overshoot = false;

    if ((Jerk < PROFILER_TOLERANCE) || (MaxAcceleration < PROFILER_TOLERANCE) ||
        (MaxDeceleration < PROFILER_TOLERANCE) || (MaxVelocity < PROFILER_TOLERANCE))
    {
        return;                                      // the limits cannot carry a profile
    }

    // everything is solved in the direction of travel, the sign is put back at the very end
    const double dir = (TargetDistance >= 0.0) ? 1.0 : -1.0;
    const double target = (dir * TargetDistance);
    const double v_start = (dir * CurrentVelocity);
    const double a_start = (dir * CurrentAcceleration);

    // velocity the axis lands on if the current acceleration is simply nulled
    const double v_zero = (v_start + ((a_start * std::fabs(a_start)) / (2.0 * Jerk)));

    // lowest peak the profile may be built around
    //   a_start > 0 : the velocity keeps rising until v_zero, nothing below it is reachable without dipping
    //   otherwise   : standstill is reachable, unless v_zero is already past it
    double v_low = 0.0;

    if (a_start > 0.0)
    {
        v_low = v_zero;
    }
    else if (v_zero < 0.0)
    {
        v_low = v_zero;
    }

    BrakeDistance = (dir * ProfileDistance(v_low, v_start, a_start));

    double v_high = MaxVelocity;

    if (v_high < v_low)                              // already above the velocity limit, the limit cannot be honoured
    {
        v_high = v_low;
    }

    double v_peak = v_low;

    // the target is closer than the shortest possible stop, the axis is going to run past it
    if (target < (dir * BrakeDistance))
    {
        Overshoot = true;
        v_peak = v_low;
    }
    else if (ProfileDistance(v_high, v_start, a_start) <= target)
    {
        v_peak = v_high;                             // the two ramps fit, the rest is covered at constant velocity
    }
    else
    {
        // ProfileDistance crosses the target somewhere between the two bounds, bisect for it
        double lo = v_low;
        double hi = v_high;

        for (int i = 0; i < PROFILER_BISECTION_STEPS; ++i)
        {
            const double mid = (0.5 * (lo + hi));

            if (ProfileDistance(mid, v_start, a_start) > target)
            {
                hi = mid;
            }
            else
            {
                lo = mid;
            }
        }

        v_peak = (0.5 * (lo + hi));
    }

    // rising side
    BuildRamp(v_start, a_start, v_peak,
              t11, t12, t13, j11, j13,
              x11, x12, x13, v11, v12, v13, a11, a12, a13);

    j12 = 0.0;

    // cruise, whatever the two ramps could not cover
    const double covered = ProfileDistance(v_peak, v_start, a_start);

    jc = 0.0;
    ac = 0.0;
    vc = v_peak;

    if ((std::fabs(v_peak) > PROFILER_TOLERANCE) && (target > covered))
    {
        tc = ((target - covered) / v_peak);
    }
    else
    {
        tc = 0.0;
    }

    xc = (v_peak * tc);

    // falling side
    BuildRamp(v_peak, 0.0, 0.0,
              t21, t22, t23, j21, j23,
              x21, x22, x23, v21, v22, v23, a21, a22, a23);

    j22 = 0.0;

    TotalDistance = (x11 + x12 + x13 + xc + x21 + x22 + x23);
    TotalTime = (t11 + t12 + t13 + tc + t21 + t22 + t23);
    PeakVelocity = v_peak;

    // put the direction back on every signed quantity, the durations stay as they are
    j11 *= dir; j12 *= dir; j13 *= dir; jc *= dir; j21 *= dir; j22 *= dir; j23 *= dir;
    x11 *= dir; x12 *= dir; x13 *= dir; xc *= dir; x21 *= dir; x22 *= dir; x23 *= dir;
    v11 *= dir; v12 *= dir; v13 *= dir; vc *= dir; v21 *= dir; v22 *= dir; v23 *= dir;
    a11 *= dir; a12 *= dir; a13 *= dir; ac *= dir; a21 *= dir; a22 *= dir; a23 *= dir;

    TotalDistance *= dir;
    PeakVelocity *= dir;

    Valid = true;
}

bool Profiler::Evaluate(double t, double &x, double &v, double &a) const
{
    const double durations[7] = { t11, t12, t13, tc, t21, t22, t23 };
    const double jerks[7] = { j11, j12, j13, jc, j21, j22, j23 };

    x = CurrentPosition;
    v = CurrentVelocity;
    a = CurrentAcceleration;

    if (!Valid)
    {
        return false;
    }

    if (t < 0.0)
    {
        t = 0.0;
    }

    double remaining = t;

    for (int i = 0; i < 7; ++i)
    {
        const double step = (remaining < durations[i]) ? remaining : durations[i];
        const double jj = jerks[i];

        x = (x + (v * step) + (0.5 * a * step * step) + (0.1666667 * jj * step * step * step));
        const double v_next = (v + (a * step) + (0.5 * jj * step * step));
        a = (a + (jj * step));
        v = v_next;

        remaining = (remaining - step);

        if (remaining <= 0.0)
        {
            return true;
        }
    }

    return false;                                    // t is past the end of the profile
}
