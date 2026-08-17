#pragma once

#include "Axis.h"

inline constexpr double TOLERANCE = 1e-6;

// quadratic equation solver, returns true if a real root exists
bool solveQuadratic(double a, double b, double c, double &x1, double &x2);

// One scan. Called once per cycle and does everything for that cycle:
//   1. writes the brake distance of the state it was entered with into axis.BrakeDistance
//   2. moves the axis on jerk limited — in Stopping it brakes down to standstill and then
//      drops back to Idle. Position carries the jerk term as well.
void GenerateTrajectory(Axis& axis, double deltaTime);
