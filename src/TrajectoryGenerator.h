#pragma once

inline constexpr double TOLERANCE = 1e-6;

inline constexpr int BISECTION_STEPS = 15;       // fixed cost, the jerk search runs every scan

// what the axis is doing right now
struct MotionState
{
    double Position = 0.0;              // mm
    double Velocity = 0.0;              // mm/s, signed
    double Acceleration = 0.0;          // mm/s^2, signed
};

// what it is not allowed to exceed. all magnitudes except the two position limits
struct MotionLimits
{
    double MaxVelocity = 0.0;           // mm/s
    double MaxAcceleration = 0.0;       // mm/s^2
    double MaxDeceleration = 0.0;       // mm/s^2
    double Jerk = 0.0;                  // mm/s^3
    double NegativeLimit = 0.0;         // mm, software position limit
    double PositiveLimit = 0.0;         // mm, ignored together with NegativeLimit when not ordered
    double InPositionWindow = 1e-4;     // mm, the axis is declared to be on the target inside this
    double InVelocityWindow = 1e-3;     // mm/s, the same idea for the velocity mode
};

// what the axis is being told to do. the unit of Target follows Type: mm for a position command,
// mm/s for a velocity one, which is why it is named here and not left to a bare double
enum class MotionCommandKind
{
    Position,           // drive to Target and stop on it
    Velocity,           // drive to Target and hold it - what MC_MoveVelocity and a jog are built on
};

struct MotionCommand
{
    MotionCommandKind Type = MotionCommandKind::Position;
    double Target = 0.0;                // mm when Position, mm/s when Velocity
};

// what the core hands back for one scan
struct TrajectoryStep
{
    MotionState State;                  // where the axis is at the end of the scan
    double Jerk = 0.0;                  // what was really applied, signed (mm/s^3)
    double BrakeDistance = 0.0;         // of the state the scan was entered with, magnitude (mm)
    bool InPosition = false;            // the axis is parked on the target (position mode)
    bool InVelocity = false;            // the axis is running at the commanded velocity (velocity mode)
};

// quadratic equation solver, returns true if a real root exists
bool solveQuadratic(double a, double b, double c, double &x1, double &x2);

// distance the axis covers if it starts braking right now, until standstill. magnitude
double BrakeDistance(double cur_vel, double cur_acc, double max_dec, double jerk);

// One scan of the trajectory core. Pure: it owns nothing, keeps nothing between calls and writes
// nothing back - the caller holds the axis and feeds a fresh command every scan if it wants to.
// Everything a motion layer is (state machine, Done/Busy/Aborted, buffering, blending, feed
// override) belongs to that caller; a feed override is just a scaled MaxVelocity handed in here.
//
// In a velocity command there is no target to stop at, so only the software limits bound the travel:
// with them unset (PositiveLimit <= NegativeLimit) the axis runs on for ever, with them set it comes
// to rest on the limit. A velocity above MaxVelocity is clamped to it.
TrajectoryStep GenerateTrajectory(const MotionState& state, const MotionLimits& limits,
                                  const MotionCommand& command, double deltaTime);
