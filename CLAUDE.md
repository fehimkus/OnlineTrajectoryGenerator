# UltimateMotion — Claude working notes

## Rules for this file

- **Read this file at the start of every session.** Look here before reading any code.
- **Update this file at the end of every prompt.** New decisions, new formulas, bugs
  found or fixed, changes of approach -> write them into the matching section. If there
  is nothing to update, say "no update", do not skip it silently.
- Everything in this repo is written in **English**: this file, code comments, docs.

## Goal

An online (real-time) trajectory generator, staying within motion control standards
(PLCopen Motion Control semantics, jerk-limited S-curve profiles).

It works **scan based**: the function is called once per scan cycle and produces the
profile that fits the state at that instant (CurrentPosition / CurrentVelocity /
CurrentAcceleration). There is NO precomputed, stored, fixed profile. Every call makes
its own decision.

That is the core difference from the older approach in sample.cpp: there the profile was
computed once from start to end (point-to-point), here it is recomputed every cycle.

## Current stage

**Brake distance calculation.** The order of work:

1. `brake_distance` = the distance covered if braking started right now, until standstill.
   It must be correct in every condition (speeding up / slowing down / constant velocity /
   standing still).
2. Then that distance gets compared against the remaining distance to the target to decide
   accelerate / cruise / brake.
3. Then the actual profile generation.

We are on step 1 only. This is still an early stage — do not do big refactors, the existing
if/else tree is meant to be filled in.

## File map

| File | Contents |
|---|---|
| `src/Axis.h` | Axis state + parameters (limits, jerk, live values) |
| `src/TrajectoryGenerator.h` | `GenerateTrajectory(Axis&, double)` declaration, `TOLERANCE = 1e-6` |
| `src/TrajectoryGenerator.cpp` | **The file being worked on.** Brake distance calculation lives here |
| `src/MainWindow.cpp/.h` | Qt6 UI: 4 charts (position/velocity/acceleration/jerk), simulation loop |
| `src/main.cpp` | Entry point |
| `src/sample.cpp` | **Reference only, does not compile.** The old point-to-point profiler |
| `docs/*.drawio` | Jerk-limited profile diagrams, decision tree draft |

`sample.cpp` is not in CMakeLists and includes a `profiler.h` that does not exist — do not
try to compile it, just read it.

## Why sample.cpp matters

It is the user's own point-to-point trajectory generator. Written in an archaic style, it
tried to take the current values into account but stayed point-to-point. Still:

- **It shows the user's coding style** — new code should look like it.
- It holds reusable methods: `solveQuadratic`, `solve2x2System`, Newton-Raphson for three
  different systems (`newtonRaphson1/2/3` + `computeSystem*` + `computeJacobian*`), the
  `CalculateShorterProfile` decision tree, and the `std::cbrt` solution for the minimum
  profile duration.
- It contains correct versions of the phase formulas (the x/v/a triples).

## Build

```bash
cmake -B build -S .
cmake --build build -j
./build/UltimateMotion
```

Syntax check only:

```bash
g++ -fsyntax-only -std=c++17 -Isrc src/TrajectoryGenerator.cpp
```

Requirements: `qt6-base-dev qt6-charts-dev cmake g++`. C++17.

## Code style (derived from sample.cpp)

- Comments are **English, lowercase, short**, placed at the end of the line with `//`.
  Do not write long derivations or explanation blocks — the user does not want them,
  one line is enough.
- Allman braces (`{` on its own line), 4 space indent.
- Formulas are written out **fully parenthesized and unsimplified**. Do not extract them
  into helper functions, leave them inline.
- Literal constants like `0.1666667` are used instead of `1.0/6.0`. Keep doing that.
- Cases are laid out as a **decision tree** of `if / else if / else`, with a comment at the
  top of each branch naming the condition it handles.
- Intermediate results go into clearly named locals like `double x11_modified_t2`.
- sample.cpp uses both `std::pow(t, 2)` and plain `t * t`; new code prefers plain products.

## Conventions

**sample.cpp phase naming** (point-to-point, 7-segment S-curve):

- `t11 / t12 / t13` — rising side: jerk raises acceleration / constant acceleration /
  jerk brings acceleration back down
- `t21 / t22 / t23` — falling side: jerk raises deceleration / constant deceleration /
  jerk brings it back down
- Each phase has a matching `x** / v** / a**` = distance covered / velocity at phase end /
  acceleration at phase end

**TrajectoryGenerator.cpp braking phases** (current work):

- phase 1 (`t1,x1,v1`) — zero the current acceleration with jerk
- phase 2 (`t2,x2,v2`) — jerk takes acceleration `0 -> -max_dec`
- phase 3 (`t3,x3,v3`) — constant `-max_dec`
- phase 4 (`x4`) — jerk takes acceleration `-max_dec -> 0`, velocity lands exactly on zero

**Sign:** `signvelocity` is the direction of the jerk to apply. When braking, jerk always
opposes the velocity. `cur_vel < 0 ? +1 : -1`.

## Brake distance decision tree — progress

```
cur_acc * cur_vel > 0   (speeding up)                          [MOSTLY DONE]
    |- phase1: zero the acceleration -> v1
    |- v1 > max_dec^2/jerk ? trapezoidal (4 phases)             [done]
    |- otherwise          : triangular (t2 = sqrt(v1/jerk))     [done]

cur_acc * cur_vel < 0   (slowing down)                         [FILLED IN]
    |- t3 = acc_abs/jerk, vel_critic = velocity lost while zeroing the acceleration
    |- vel_abs > vel_critic  (velocity is enough)
    |     |- acc_abs < max_dec : raise deceleration to max_dec  [done]
    |     |     |- v1 > 0.5*max_dec^2/jerk ? trapezoidal
    |     |     |- otherwise   : triangular, acc_peak below max_dec
    |     |- acc_abs > max_dec : lower deceleration to max_dec  [done]
    |     |     |- always trapezoidal, see the note below
    |     |- acc_abs = max_dec : constant phase + final jerk    [done]
    |- vel_abs < vel_critic  (velocity is not enough)           [done]
    |     |- over braking, velocity hits zero before the acceleration does
    |     |- solveQuadratic(0.5*jerk, -acc_abs, vel_abs), smaller root
    |- vel_abs = vel_critic  (exactly enough)                   [done]
    |     |- brake_distance = acc_abs*t1^2/6, t1 = acc_abs/jerk
```

The whole `cur_acc * cur_vel < 0` branch works on **magnitudes** (`vel_abs`, `acc_abs`), so
its `brake_distance` comes out positive. The speeding-up branch still mixes signed and
unsigned values — that is bug 2/3/4 below.

Why `acc_abs > max_dec` never needs a triangular sub-case: lowering the deceleration
`acc_abs -> max_dec` costs `(acc_abs^2 - max_dec^2)/(2*jerk)` of velocity and the final jerk
phase costs `max_dec^2/(2*jerk)`; together exactly `vel_critic`. The branch is only entered
when `vel_abs > vel_critic`, so the constant-`max_dec` phase always has positive duration.

Formulas reached in that subtree:

- triangular peak: `acc_peak = sqrt(jerk*vel_abs + 0.5*acc_abs^2)`
- final jerk phase always collapses to `x = acceleration * t^2 / 6` (same shorthand as `x4`)

```

velocity or acceleration is zero                                [EMPTY]
    |- v != 0, a == 0 : plain braking profile
    |- v == 0         : brake_distance = 0
```

## Known bugs (reported to the user, deliberately not fixed)

`src/TrajectoryGenerator.cpp`:

1. **line 49** — there is no variable named `s`, it should be `signvelocity`. **This is why
   the file does not compile.**
2. **line 47 vs 49/50** — the assignment yields `-sign(cur_vel)`, but the formulas are
   written as `- signvelocity * jerk`, so they expect `sign(cur_vel)`. One of the two signs
   has to be flipped.
3. **line 50** — `v1 = v0 + a0*t1 - 0.5*sign*j*t1^2`; since the acceleration ramps down
   linearly the result should be `v0 + a0*t1/2`. Right now velocity drops too much (fixing
   item 2 fixes this too).
4. **line 52** — the `v1 > max_dec^2/jerk` comparison assumes `v1` is positive; motion in
   the negative direction needs `std::abs(v1)`.
5. `solveQuadratic` — no `a == 0` check, risk of division by zero.
6. `max_vel`, `max_acc`, `deltaTime` and `brake_distance` are currently unread/unused (the
   decision step is not written yet, so this is expected).
7. **line 81** — `vel_critic = cur_acc*t3 + 0.5*jerk*t3^2` is signed, `vel_abs` is not, so
   the whole slowing-down subtree picks the wrong branch. With `cur_vel > 0` it comes out
   negative, with `cur_vel < 0` it comes out three times too large. Magnitude version:
   `vel_critic = (acc_abs * t3) - (0.5 * jerk * t3 * t3)`, i.e. `0.5*acc_abs^2/jerk`.
   The filled-in branches below it are written for this magnitude version.

Verified correct (not bugs): `x4 = max_dec*t2^2/6`, `x2 = v1*t2` for the triangular profile,
and phase 3 reserving `0.5*max_dec^2/j` of velocity for the final jerk phase.

The `cur_acc*cur_vel < 0` subtree was cross checked against a 1e-7 step numeric integration
(9 cases: trapezoidal / triangular / at the limit / over the limit / over braking / exactly
critical). Closed form and simulation agree to ~1e-6, and every case except over braking
lands on `v = 0, a = 0`. Over braking ends with a residual deceleration — unavoidable, the
axis is already braking harder than jerk can undo before standstill.

## Working style notes

- The user writes the code themselves; do not change the logic unless asked, just do what
  was asked.
- When comments are requested, keep them **short**.
- Point out bugs, but do not fix them unprompted — ask first.

---
*Last update: 2026-08-17 — the `cur_acc*cur_vel < 0` (slowing down) subtree in
TrajectoryGenerator.cpp got filled in, all double comparisons now go through `TOLERANCE`,
and the `vel_critic` sign bug was found (item 7, not fixed). Earlier the same day: README.md
got an "Under construction" section; whole repo translated to English. UI strings in
MainWindow.cpp are still Turkish, pending the user's decision.*
