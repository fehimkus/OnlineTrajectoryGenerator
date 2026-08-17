#pragma once

enum class AxisState
{
    Idle,
    ContinuousMotion,
    DiscreteMotion,
    Stopping,                        // jerk limited braking down to standstill
};

struct Axis
{
    AxisState CurrentState = AxisState::Idle;

    // Target / parameters
    double MaxVelocity = 100.0;      // velocity limit (positive, mm/s)
    double TargetPosition = 0.0;     // target position (mm)
    double MaxAcceleration = 500.0;  // mm/s^2
    double MaxDeceleration = 500.0;  // mm/s^2
    double Jerk = 5000.0;            // mm/s^3

    // Software position limits
    double NegativeLimit = -500.0;   // mm
    double PositiveLimit = 500.0;    // mm

    // Live state
    double CurrentPosition = 0.0;
    double CurrentVelocity = 0.0;
    double CurrentAcceleration = 0.0;

    // Trajectory generator output
    double BrakeDistance = 0.0;      // distance needed to stop from the current state (mm)
};
