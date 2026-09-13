#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#define TOLERANCE 1e-6

#define BISECTION_STEPS 15              // fixed cost, the jerk search runs every scan

// the compiler is told the four arguments never alias, so nothing has to be reloaded after a write
#if defined(__GNUC__) || defined(__clang__)
#  define MOTION_RESTRICT __restrict
#else
#  define MOTION_RESTRICT
#endif

// what the axis is doing right now
typedef struct
{
    double Position;                    // mm
    double Velocity;                    // mm/s, signed
    double Acceleration;                // mm/s^2, signed
} MotionState;

// what it is not allowed to exceed. all magnitudes except the two position limits
typedef struct
{
    double MaxVelocity;                 // mm/s
    double MaxAcceleration;             // mm/s^2
    double MaxDeceleration;             // mm/s^2
    double Jerk;                        // mm/s^3
    double NegativeLimit;               // mm, software position limit
    double PositiveLimit;               // mm, ignored together with NegativeLimit when not ordered
    double InPositionWindow;            // mm, the axis is declared to be on the target inside this
    double InVelocityWindow;            // mm/s, the same idea for the velocity mode
} MotionLimits;

// what the axis is being told to do. the unit of Target follows Type: mm for a position command,
// mm/s for a velocity one, which is why it is named here and not left to a bare double
typedef enum
{
    MOTION_COMMAND_POSITION = 0,        // drive to Target and stop on it
    MOTION_COMMAND_VELOCITY = 1,        // drive to Target and hold it - what MC_MoveVelocity and a jog are built on
} MotionCommandKind;

typedef struct
{
    MotionCommandKind Type;
    double Target;                      // mm when Position, mm/s when Velocity
} MotionCommand;

// what the core hands back for one scan
typedef struct
{
    MotionState State;                  // where the axis is at the end of the scan
    double Jerk;                        // what was really applied, signed (mm/s^3)
    double BrakeDistance;               // of the state the scan was entered with, magnitude (mm)
    int InPosition;                     // the axis is parked on the target (position mode)
    int InVelocity;                     // the axis is running at the commanded velocity (velocity mode)
} TrajectoryStep;

// zeroes the limits and puts back the two window defaults a C struct cannot carry on its own
void MotionLimitsInit(MotionLimits *limits);

// quadratic equation solver, returns non zero if a real root exists
int solveQuadratic(double a, double b, double c, double *x1, double *x2);

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
TrajectoryStep GenerateTrajectory(const MotionState *MOTION_RESTRICT state,
                                  const MotionLimits *MOTION_RESTRICT limits,
                                  const MotionCommand *MOTION_RESTRICT command,
                                  double deltaTime);

#ifdef __cplusplus
}
#endif

#endif
