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

### Brake test rig (UI)

The UI is a test bench for step 1, nothing else. Everything that braking does not use was
stripped out (target/max velocity, max acceleration, live value read-outs, pause, reset) —
what is left is only what this test needs. **Every stage gets its own test**, so expect this
UI to be rewritten rather than extended.

"Rastgele Fren Testi" drops the axis into a random `CurrentVelocity` / `CurrentAcceleration`,
freezes the `BrakeDistance` of that instant, brakes to standstill with `MoveAxis`, and shows:

- **Hesaplanan mesafe** — the frozen prediction from `GenerateTrajectory`
- **Gerçekleşen mesafe** — what the motion really covered
- **Hata** — the difference, this is the number the whole stage is about
- **Seçilen dal** — which top level branch the random state landed in
- **Alınan + canlı - tutulan** — travelled + live brake distance - frozen value, has to stay 0

`Tarama periyodu` sets the scan the generator is called with. It separates a real formula
error from scan granularity: discretization error falls with the scan period (~1e-2 mm at
1 ms, ~1e-4 at 10 us, ~1e-6 at 100 ns), a formula error does not move at all.

**`GenerateTrajectory` is the only thing the caller triggers.** One call per scan does both
halves, in this order and no other:

1. the decision tree writes `axis.BrakeDistance` for the state the scan was entered with
2. then, if `CurrentState == Stopping`, the motion section walks `x/v/a` forward

The order is what makes the frozen value comparable: the distance handed out belongs to the
state *before* the axis moves. The motion section returns early for every other state, so
calling it while Idle only refreshes the brake distance.

Even though they now sit in one function, the motion section shares **no formula** with the
brake distance tree: it walks the deceleration towards a target (`0` if the deceleration
already eats the whole velocity, otherwise `max_dec`) and integrates `x/v/a` with the full
polynomials. Keep it that way — if it were built on the same formulas the test would prove
nothing. The only things the two halves share are the `cur_*` / `max_dec` / `jerk` locals
read at the top of the function.

**"Log Al"** appends the last finished test to `braketest_log.csv` in the working directory
(gitignored, header written on first use). One row per press, start condition and stop
condition together, so a suspicious case can be replayed later:

```
time, start_vel, start_acc, start_pos, max_dec, jerk, scan_ms, branch,
held_brake, measured_dist, error, stop_time, stop_pos,
vel_before_last_scan, acc_before_last_scan
```

The last two columns are the state going into the scan that hit standstill — that is where
an over braking case leaves its leftover deceleration.

## File map

| File | Contents |
|---|---|
| `src/Axis.h` | Axis state + parameters (limits, jerk, live values), `BrakeDistance` output |
| `src/TrajectoryGenerator.h` | `GenerateTrajectory` + `solveQuadratic` declarations, `TOLERANCE = 1e-6` |
| `src/TrajectoryGenerator.cpp` | **The file being worked on.** Brake distance calculation *and* the motion equations, in one function |
| `src/MainWindow.cpp/.h` | Qt6 UI: brake test rig, 3 charts (position/velocity/acceleration) |
| `src/main.cpp` | Entry point |
| `src/sample.cpp` | **Reference only, does not compile.** The old point-to-point profiler |
| `src/motionold.cpp` | **Reference only, does not compile.** The user's old scan based generator |
| `docs/*.drawio` | Jerk-limited profile diagrams, decision tree draft |

`sample.cpp` and `motionold.cpp` are both in `.gitignore`.

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
./build/braketest
```

The binary is named after the test it carries, not after the project. Renaming the target
means the CMake cache has to be thrown away (`rm -rf build`) before reconfiguring.

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
cur_acc * cur_vel > 0   (speeding up)          [DONE, VERIFIED ON THE RIG]
    |- phase1: zero the acceleration -> v1, signed, uses signvelocity
    |- from there on magnitudes: x1_abs / v1_abs
    |- v1_abs > max_dec^2/jerk ? trapezoidal (4 phases)         [done]
    |- otherwise             : triangular (t2 = sqrt(v1_abs/jerk))  [done]

cur_acc * cur_vel < 0   (slowing down)         [DONE, VERIFIED ON THE RIG]
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

1. ~~**line 49** — there is no variable named `s`~~ **FIXED** (`s` -> `signvelocity`), it was
   blocking the build and the UI test rig needed a compiling file. This was the only fix
   applied without asking; everything below is still open.
2. ~~**line 47 vs 49/50** — `signvelocity` fought the sign written in the formulas~~
   **FIXED** on request. The assignment stayed as documented (`signvelocity` = the jerk
   direction = `-sign(cur_vel)`) and the two formulas flipped to `+ signvelocity * jerk`.
3. ~~**line 50** — `v1` dropped too much~~ **FIXED** by item 2, it now yields `v0 + a0*t1/2`.

   How the old numbers came about, on the logged case v=195.8776, a=+177.1689, j=5000,
   max_dec=500: `x1` was `2*(1/6)*j*t1^3 = 0.074` mm too long from item 2, and `v1` was
   `a0^2/j = 6.278` mm/s too high from item 3, which stretched the rest of the profile by
   another 2.86 mm. It predicted 59.4994; it now says 56.5732 against a real 56.5731 at a
   0.01 ms scan.
4. ~~**line 52** — the `v1 > max_dec^2/jerk` comparison assumed `v1` is positive~~ **FIXED**:
   `x1_abs` / `v1_abs` are taken after phase 1 and the braking phases run on magnitudes,
   the same way the slowing-down branch does. This also killed item 8.
5. `solveQuadratic` — no `a == 0` check, risk of division by zero. The motion section works
   around it by testing `std::abs(jerk_cmd) < TOLERANCE` before calling.
6. `max_vel` and `max_acc` are still unread (the decision step is not written yet, so this is
   expected). `deltaTime` and `brake_distance` are used now.
7. ~~**line 81** — `vel_critic` was signed while `vel_abs` is not~~ **FIXED** on request,
   now `vel_critic = (acc_abs * t3) - (0.5 * jerk * t3 * t3)`, i.e. `0.5*acc_abs^2/jerk`.
   With this the slowing-down subtree is clean: over 2000 random states the worst error
   dropped from -64.04 mm to -0.032 mm, and that leftover is scan granularity (it falls to
   -0.00046 mm at a 0.01 ms scan), not a formula error.
8. ~~**speeding-up branch with `cur_vel < 0`** returned NaN~~ **FIXED** by item 4.
   v=-100 a=-200 now gives 20.1227, mirroring v=+100 a=+200 exactly.
9. **the `else` branch (v or a is zero) is still empty** — the only real gap left.
   `brake_distance` stays 0 while the axis really needs a full braking profile.
   Rig case: v=100 a=0 -> predicted 0.0000, really covered 14.9795 mm.

   It also punches a **one scan hole in the middle of every other profile**: while braking
   from a speeding-up state the acceleration passes through zero, `cur_acc * cur_vel` falls
   inside `±TOLERANCE` for that scan, and the live brake distance collapses to 0. The rig's
   "Alınan + canlı - tutulan" meter catches it as a -16.0 mm spike on v=100 a=200 (exactly
   `x1 - held` = 4.107 - 20.123, i.e. the whole remaining profile going missing for a scan).

Verified correct (not bugs): `x4 = max_dec*t2^2/6`, `x2 = v1*t2` for the triangular profile,
and phase 3 reserving `0.5*max_dec^2/j` of velocity for the final jerk phase.

The `cur_acc*cur_vel < 0` subtree was cross checked against a 1e-7 step numeric integration
(9 cases: trapezoidal / triangular / at the limit / over the limit / over braking / exactly
critical). Closed form and simulation agree to ~1e-6, and every case except over braking
lands on `v = 0, a = 0`. Over braking ends with a residual deceleration — unavoidable, the
axis is already braking harder than jerk can undo before standstill.

**Current state of both filled branches**, 4000 random states, `max_dec=500`, `jerk=5000`,
`v` in ±200, `a` in ±2000, worst error against the real motion:

| branch | before the fixes | now, 1 ms scan | now, 0.01 ms scan |
|---|---|---|---|
| speeding up | -1723.67 mm | -0.0348 mm | ~-0.0005 mm |
| slowing down | -64.04 mm | -0.0349 mm | -0.00046 mm |

Both leftovers scale with the scan period, so they are granularity, not formula error.
Nothing in either branch is known to be wrong any more — the open item is bug 9.

## Working style notes

- The user writes the code themselves; do not change the logic unless asked, just do what
  was asked.
- When comments are requested, keep them **short**.
- Point out bugs, but do not fix them unprompted — ask first.

---
*Last update: 2026-08-17 — `MoveAxis` folded into `GenerateTrajectory` itself and removed as
a separate function: the caller now triggers one method per scan, which hands out the brake
distance first and then moves the axis. MotionGenerator.cpp/.h deleted. Bugs 2/3/4/8 (the
`signvelocity` sign and the unsigned comparison) fixed on request, so **both filled branches
are clean**: worst error over 4000 random states is 0.035 mm at a 1 ms scan, all of it scan
granularity. Bug 7 was fixed earlier the same session. The only open gap is bug 9, the empty
`else` branch. Same day: UI trimmed down to the brake test only, binary renamed to
`braketest`, "Log Al" writes start+stop conditions to `braketest_log.csv`; the UI became a
brake test rig: random start state, frozen vs.
measured brake distance, 3 charts, adjustable scan period; `MoveAxis` in the new
MotionGenerator.cpp carries the jerk limited motion equations; `Axis.BrakeDistance` and
`AxisState::Stopping` added; bug 1 fixed so it builds; bugs 7/8/9 confirmed on the rig.
Earlier the same day: the slowing-down subtree got filled in and all double comparisons went
through `TOLERANCE`; README.md got an "Under construction" section; whole repo translated to
English. UI strings in MainWindow.cpp are still Turkish, pending the user's decision.*
