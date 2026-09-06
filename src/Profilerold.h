#pragma once

inline constexpr double PROFILER_TOLERANCE = 1e-9;
inline constexpr int PROFILER_BISECTION_STEPS = 100;

// point to point jerk limited profiler, 7 segments
//   11 / 12 / 13 - rising side : jerk raises acceleration / constant acceleration / jerk brings it back to zero
//   c            - cruise      : constant velocity at the peak
//   21 / 22 / 23 - falling side: jerk raises deceleration / constant deceleration / jerk brings it back to zero
// the profile starts at (CurrentVelocity, CurrentAcceleration) and always ends at v = 0, a = 0
class Profiler
{
public:
    // inputs, filled by the caller before CalculatePositionProfile
    double CurrentPosition = 0.0;         // mm
    double CurrentVelocity = 0.0;         // mm/s, signed
    double CurrentAcceleration = 0.0;     // mm/s^2, signed
    double TargetDistance = 0.0;          // mm, signed, relative to CurrentPosition

    double MaxVelocity = 0.0;             // mm/s, magnitude
    double MaxAcceleration = 0.0;         // mm/s^2, magnitude
    double MaxDeceleration = 0.0;         // mm/s^2, magnitude
    double Jerk = 0.0;                    // mm/s^3, magnitude

    // phase durations
    double t11 = 0.0, t12 = 0.0, t13 = 0.0;
    double tc = 0.0;
    double t21 = 0.0, t22 = 0.0, t23 = 0.0;

    // signed jerk applied in each phase
    double j11 = 0.0, j12 = 0.0, j13 = 0.0;
    double jc = 0.0;
    double j21 = 0.0, j22 = 0.0, j23 = 0.0;

    // distance covered in each phase
    double x11 = 0.0, x12 = 0.0, x13 = 0.0;
    double xc = 0.0;
    double x21 = 0.0, x22 = 0.0, x23 = 0.0;

    // velocity at the end of each phase
    double v11 = 0.0, v12 = 0.0, v13 = 0.0;
    double vc = 0.0;
    double v21 = 0.0, v22 = 0.0, v23 = 0.0;

    // acceleration at the end of each phase
    double a11 = 0.0, a12 = 0.0, a13 = 0.0;
    double ac = 0.0;
    double a21 = 0.0, a22 = 0.0, a23 = 0.0;

    // results
    double PeakVelocity = 0.0;            // signed velocity the cruise phase runs at
    double TotalDistance = 0.0;           // signed, has to match TargetDistance unless Overshoot
    double TotalTime = 0.0;               // s
    double BrakeDistance = 0.0;           // signed, shortest distance to standstill from the current state
    bool Overshoot = false;               // the target is closer than BrakeDistance, the axis cannot stop on it
    bool Valid = false;                   // false if the limits are unusable

    void CalculatePositionProfile();

    // profile state at time t measured from the start of the profile, false past the end
    bool Evaluate(double t, double &x, double &v, double &a) const;

    // quadratic equation solver, returns true if a real root exists
    static bool solveQuadratic(double a, double b, double c, double &x1, double &x2);

    // 2x2 linear system solver, returns false on a near singular matrix
    static bool solve2x2System(const double A[2][2], const double b[2], double x[2]);

private:
    // brings (v_start, a_start) to (v_end, 0) inside the acceleration limits
    // ta / tb / tc_out are the three sub phase durations, ja / jc_out the jerk applied in the first and last one
    void BuildRamp(double v_start, double a_start, double v_end,
                   double &ta, double &tb, double &tc_out,
                   double &ja, double &jc_out,
                   double &xa, double &xb, double &xc_out,
                   double &va, double &vb, double &vc_out,
                   double &aa, double &ab, double &ac_out) const;

    // distance of the whole profile if the peak velocity is v_peak, cruise excluded
    double ProfileDistance(double v_peak, double v_start, double a_start) const;
};
