#pragma once

#include "Axis.h"

inline constexpr double TOLERANCE = 1e-6;

// One simulation step: in ContinuousMotion it drives the axis toward the target
// velocity (MaxVelocity) jerk-limited; position advances as the integral of velocity.
void GenerateTrajectory(Axis& axis, double deltaTime);
