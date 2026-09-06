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

**Online tracking.** Steps 1 and 2 are done, step 3 is the decision tree's own shape.
The order of work:

1. `brake_distance` = the distance covered if braking started right now, until standstill.
   It must be correct in every condition (speeding up / slowing down / constant velocity /
   standing still).
2. Then that distance gets compared against the remaining distance to the target to decide
   accelerate / cruise / brake.
3. Then the actual profile generation.

Step 1 is finished and cross checked (the last gap, the empty `else`, was filled 2026-09-06).
Step 2 and 3 now live in the same function as the `Tracking` branch — see the section below.
Still do not do big refactors, the existing if/else tree is meant to be filled in.

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
| `src/OnlineWindow.cpp/.h` | Qt6 UI: **online rig**, handwheel slider + 4 charts (position/velocity/acceleration/jerk) |
| `src/online_main.cpp` | Entry point of the online rig |
| `src/main.cpp` | Entry point of the brake rig |
| `src/Profilerold.h/.cpp` | Point-to-point profiler, 7-segment S-curve, peak-velocity solver. Renamed to `...old` by the user once the work moved online; still in CMakeLists so it keeps compiling |
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
./build/onlinetest        # online tracking rig, the current stage
./build/braketest         # brake distance rig, the previous stage
```

Binaries are named after the test they carry, not after the project. Renaming the target
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

velocity or acceleration is zero                [DONE, CROSS CHECKED AGAINST THE MOTION]
    |- v == 0, a == 0 : brake_distance = 0
    |- v != 0, a == 0 : plain braking profile straight from vel_abs
    |     |- vel_abs > max_dec^2/jerk ? trapezoidal (3 phases)
    |     |- otherwise                : triangular, t1 = sqrt(vel_abs/jerk), x1 = vel_abs*t1
    |- v == 0, a != 0 : the axis is about to run away, the jerk kills the acceleration first
          |- x1 = acc_abs*t1^2/3, v1 = acc_abs*t1/2 = acc_abs^2/(2*jerk), t1 = acc_abs/jerk
          |- then brake from v1, same trapezoidal / triangular split
```

The closed form of the `v != 0, a == 0` trapezoid collapses to
**`brake_distance = v^2/(2*max_dec) + v*max_dec/(2*jerk)`** — the plain `v^2/(2a)` plus what the
two jerk phases cost. Derived from the 3-phase sum and checked against it; this is the form the
online branch inverts to get `vel_allowed`.

## Point-to-point profiler (`src/Profiler.h/.cpp`)

Written 2026-09-06 to replace the solver in `profilerold.cpp` / `sample.cpp`. Separate from
the scan-based `TrajectoryGenerator` — this one plans a whole move in one call.

### Why the old solver was replaced

`CalculateShorterProfile` was a six-branch tree (CASE 1/1a/1b-1/1b-2/2/2a/2b-1/2b-2/3/3a/3b)
driven by three 2x2 Newton-Raphson systems. Reviewed on request; it is wrong in ways that
cannot be patched branch by branch:

- `t12` subtracts `MaxAcceleration*t11` where it needs `(2*max_acc^2 - cur_acc^2)/(2*jerk)` —
  correct only at `cur_acc == 0`, i.e. the whole point of the generator never worked.
- `t22` subtracts `CurrentVelocity` on the braking side, which goes `MaxVelocity -> 0`.
- Triangular rising side sets `t13 = t11` and drops `cur_acc`; valid only at `a0 == 0`.
  Same line does `sqrt(MaxVelocity - CurrentVelocity)` -> NaN when `cur_vel > max_vel`.
- Falling side is integrated from `MaxVelocity`, not from `v13`, so the two halves are
  disconnected whenever the rising side does not reach the limit.
- `a13` is written `a11 + jerk*t13` in five places and `a11 - jerk*t13` in five others.
- `t23` is integrated with `t13` in CASE 2.
- CASE 2's `_modified_t2` block applies the *start* conditions to the braking phases.
- `c3` in CASE 2b-1 adds terms of dimension `s^3` and `s^4` to a distance.
- `computeJacobian2` returns `2.0` and `-1.0` where the derivatives are `2*t21` and `-t11`.
- `newtonRaphson1` solves for `t21` but `c2` is built from the *old* `t21` and never
  refreshed, and `t23` is left at `MaxDeceleration/jerk`, so `a23 != 0` — the profile ends
  with residual acceleration.
- No cruise phase at all (6 segments, not 7); no direction sign; no reversal case.
- CASE 3 compares `t21 == t11` on doubles, so it is dead code.

### What replaced it

One monotone scalar solve instead of the branch tree:

```
f(v_peak) = X_up(v0, a0 -> v_peak) + X_down(v_peak -> 0)

f(v_low)  > TargetDistance  -> Overshoot, the axis cannot stop on the target
f(MaxVel) <= TargetDistance -> v_peak = MaxVelocity, remainder becomes the cruise phase
otherwise                   -> bisect f(v_peak) = TargetDistance on [v_low, MaxVelocity]
```

- `BuildRamp` is the single ramp primitive: `(v_start, a_start) -> (v_end, 0)` inside the
  limits. It returns three sub-phase durations, the **signed jerk of each phase**, and the
  x/v/a at every phase end. Both the rising and the falling side are the same call, so the
  two halves cannot drift apart the way they did in the old code.
- Peak acceleration: `a_peak = +sqrt(jerk*dv + 0.5*a0^2)` when speeding up,
  `a_peak = -sqrt(0.5*a0^2 - jerk*dv)` when slowing down, then clamped to
  `MaxAcceleration` / `MaxDeceleration`. The branch is picked by comparing `dv` against
  `dv_zero = a0*|a0|/(2*jerk)`, the velocity gained while nulling `a0`.
- `a_start > a_limit` needs no special case: the first sub-phase carries a signed jerk, so
  it lowers the acceleration just as happily as it raises it.
- `v_low` (lowest usable peak) is `v_zero` when `a0 > 0` — a positive acceleration cannot be
  undone before `v_zero`, and peaks below it make `f` non-monotone (it humps: at
  `v0=100, a0=1000, j=5000` both `v_peak=0` and `v_peak=200` give 73.33 mm while
  `v_peak=100` gives 89.9). Otherwise `v_low` is `0`, or `v_zero` if that is already negative.
- `BrakeDistance = f(v_low)` falls out of the same machinery.
- Everything is solved in the direction of travel (`dir = sign(TargetDistance)`) and the sign
  is put back on every signed member at the end.
- `Evaluate(t, x, v, a)` walks the 7 segments with the full polynomials — this is what a
  test rig or a scan-based caller should use.

`solveQuadratic` kept but fixed: `a == 0` falls back to the linear root, and the roots use
the stable `q = -0.5*(b + sign(b)*sqrt(D))` form instead of `(-b + sqrt(D))/(2a)`.
`solve2x2System` kept unchanged. The three Newton-Raphson systems were **not** carried over;
they stay in `profilerold.cpp` for reference.

### Verified

200k random states, `max_vel=200`, `max_acc=1000`, `max_dec=800`, `jerk=5000`,
`v0` in +/-200, `a0` in +/-800, `TargetDistance` in +/-800, profile sampled at 600 points:

| check | worst |
|---|---|
| `x_end - TargetDistance` | 4.0e-12 mm |
| `v_end` | 4.7e-12 mm/s |
| `a_end` | 8.6e-12 mm/s^2 |
| velocity over the limit | 1.6e-12 mm/s |
| acceleration over the limit | 3.4e-13 mm/s^2 |
| negative phase duration | none |

The velocity limit is only ever exceeded when the *start* state already carries the axis past
it (`|v0 + a0*|a0|/(2*jerk)| > max_vel`) — nulling that acceleration costs `a0^2/(2*jerk)` of
velocity and nothing can prevent it. Those states are excluded from the table above; with
them in, the worst excursion is 62.8 mm/s and is exactly that quantity.

### Open / not covered

- **Overshoot is flagged, not solved.** When `TargetDistance < BrakeDistance` the profiler
  sets `Overshoot = true` and emits the fastest stop; it does not plan the reversal move
  back to the target. The caller has to re-plan once the axis is at standstill.
- `MaxDeceleration` is picked whenever the ramp's peak acceleration is negative. For forward
  motion that is exactly "slowing down"; for a ramp that crosses zero velocity it is an
  approximation.
- `v0` above `MaxVelocity` is accepted (the limit is raised to `v_low` for that move) rather
  than planned down to `MaxVelocity` first.
- Software position limits (`NegativeLimit` / `PositiveLimit`) are not consulted.
- Nothing calls it yet — it is in CMakeLists so it compiles, but no UI or test rig drives it.

## `src/motionold.cpp` — known defects (reference file, not fixed)

Reviewed 2026-09-06. The user's old scan-based generator. Nothing here is fixed; listed so
the same mistakes do not get copied into `TrajectoryGenerator.cpp`.

ContinuousMotion:

- The software-limit block writes `axis.MaxVelocity = 0.0` *after* `max_vel` was already
  copied into a local, so it takes effect a scan late — and it destroys the caller's
  parameter permanently. It also ignores the brake distance (the axis overruns the limit)
  and tests the sign of `max_vel` instead of `cur_vel`.
- `in_jerk_phase` compares `abs_diff < a0^2/(2*jerk)` without checking that `cur_acc` points
  toward the target; with the acceleration on the wrong side the jerk drives it further away.
- The jerk phase never clamps the acceleration at zero, so it crosses over and rings.
- `limit` is chosen by `|max_vel| > |cur_vel|`; after a direction reversal it keeps using
  `MaxDeceleration` while the axis is accelerating.

DiscreteMotion:

- No jerk at all — acceleration is a step to `+/-MaxAcceleration` / `+/-MaxDeceleration`,
  while ContinuousMotion is jerk limited.
- `d_brake = v^2/(2*max_dec)` ignores jerk and `CurrentAcceleration`, so it always
  undershoots; no `max_dec == 0` guard.
- `CurrentAcceleration = -current_v / dt` is an unbounded one-scan stop, past both limits,
  gated by a unit-dependent `abs_err < 1.0`.
- `std::clamp(v, -MaxVelocity, MaxVelocity)` is UB if `MaxVelocity` is negative.
- The stop test needs `abs_err < TOLERANCE_POSITION` *and* a small velocity in the same scan;
  the position steps by `v*dt` and can jump the window, leaving the axis hunting.
- `if (dt <= 0) return nullptr;` skips both the `prevCycletime` update and the SHM write.
- Acceleration is not zeroed once the velocity clamps at `MaxVelocity`.

Both states:

- `deltaTime` comes from the wall clock with no ceiling; a stale `prevCycletime` produces one
  huge scan and a position jump.
- The wrap at the end folds `CurrentPosition` but not `TargetPosition`, so a DiscreteMotion
  move takes the long way round the moment it wraps. `EncoderExceedPoint == 0` makes both
  `while` loops spin forever.

## Online tracking — the `Tracking` branch of `GenerateTrajectory`

Written 2026-09-06. `AxisState::Tracking` turns `GenerateTrajectory` into the online generator:
`TargetPosition` is allowed to be somewhere else on every single scan and **nothing is stored
between scans**. The whole command is rebuilt from `(cur_pos, cur_vel, cur_acc)` and the target
of that instant. This is the difference from `Profilerold`, which solves a whole move once.

### The two nested questions

```
1. how fast am i allowed to be right here?   -> vel_command   (from the remaining distance)
2. what acceleration gets me to that speed?  -> acc_target    (from the current acceleration)
3. jerk walks the acceleration to acc_target, integrate one scan
```

**Step 1** is the brake-distance-vs-remaining-distance comparison this file started with, only it
is solved *for velocity* instead of being asked as a yes/no. A yes/no ("can i still stop? then
speed up, else brake") switches once per scan around the switching point and chatters; its
inverse is continuous. Two closed forms, both inverses of the tree above:

- trapezoidal brake: `remaining = v^2/(2*max_dec) + v*max_dec/(2*jerk)`, solved with
  `solveQuadratic`. The second term is what the two jerk phases cost on top of the plain
  `v^2/(2a)`. Cross checked against the 3-phase sum in the tree, they agree exactly.
- triangular brake: `remaining = v^1.5 / sqrt(jerk)`, so `v = cbrt(remaining^2 * jerk)`.
- the split is at `remaining = max_dec^3 / jerk^2`, where `max_dec` is just barely reached.
- `vel_allowed` is then capped by `MaxVelocity`.

**The envelope is asked at the `a = 0` point, not here.** It only knows how to brake from a
standing acceleration, so the stretch the axis covers while the jerk kills the current
acceleration comes off the remaining distance first:

```
t_null = |a| / jerk
x_null = v*t_null + (1/3)*a*t_null^2          // signed
remaining_zero = remaining - x_null
```

Forgetting this is what makes a controller of this shape overshoot by exactly one acceleration
ramp — on the rig it was 1.7 mm per step before the correction and 0.07 mm after.

**Step 2** is the same peak acceleration `Profilerold::BuildRamp` solves for, recomputed every
scan instead of once per move:

```
delta_vel      = vel_command - cur_vel
delta_vel_zero = a*|a| / (2*jerk)             // velocity still picked up while a is zeroed

delta_vel > delta_vel_zero -> acc_target = +sqrt(jerk*delta_vel + 0.5*a^2)
otherwise                  -> acc_target = -sqrt(0.5*a^2 - jerk*delta_vel)
```

Two things clamp it afterwards:

- **`acc_exact`**, the acceleration that lands the velocity exactly on the command at the end of
  this scan: `acc_exact = 2*delta_vel/deltaTime - cur_acc` (trapezoid rule over one step).
  The square roots have unbounded slope at `delta_vel = 0`, so within a scan of the command they
  ask for far more acceleration than the scan can use and the sign flips every cycle — the
  acceleration stays tidy but the **jerk output turns into noise sitting on its limit**, which is
  exactly what the jerk chart showed before this clamp. There is never a reason to ask for more
  than `acc_exact`.
- **the limit**, `max_acc` or `max_dec`. Which one applies is decided by whether the acceleration
  fights the motion or feeds it, **not by its sign** — travelling backwards a positive
  acceleration is a deceleration, and picking by sign alone swaps the two limits on every move
  that runs in the negative direction. That bug was live for one iteration and showed up as
  `|a| = 984` against a `max_dec` of 800.

**Step 3** walks `a` towards `acc_target` by `jerk * deltaTime` and stops exactly on it, then
`jerk_cmd = (new_acc - cur_acc)/deltaTime` is what really gets integrated, so the polynomials stay
exact even on the scan where the clamp bites.

### In position window

`Axis.InPositionWindow` (default 1e-4 mm). A time optimal position loop has **infinite gain at the
target**: `vel_allowed` is a cube root of the remaining distance and its slope runs away as that
distance goes to zero. With a finite scan the axis can never settle on it — measured before the
window existed: a stable 18-scan limit cycle, +/-1.2e-6 mm wide, with the jerk pinned at its limit
for ever. Every real drive answers this with an in position window and so does this one: inside the
window, slow enough that the brake distance also fits inside the window, and with an acceleration
small enough that parking it is not a jerk step, the axis is declared to be there and parked.

### Verified

20000 random `(v0, a0, target)` states, `max_vel=200 max_acc=1000 max_dec=800 jerk=5000`,
0.2 ms scan, run to standstill:

| check | worst |
|---|---|
| acceleration over `max_acc` (while speeding up) | 0 |
| deceleration over `max_dec` (while slowing down) | 0 |
| jerk over `Jerk` | 2.8e-10 |
| velocity over `max_vel` | 0.2 mm/s |
| overshoot, when the axis **could** stop on the target (9489 cases) | 0.036 mm |
| overshoot, when it **could not** (10511 cases) | 55.97 mm |
| cases that did not land exactly on the target | none |

The second overshoot row is the start state already carrying the axis past the target; it brakes,
reverses and comes back. Overshoot scales with the scan: 0.138 mm at 2 ms, 0.0114 at 1 ms,
0.00026 at 0.2 ms, 0.000053 at 0.1 ms — `dt^2`, so granularity, not formula error. The jerk sign
changes after entering the last 1% of a step fall the same way: 39 at 2 ms, 9 at 1 ms, 2 at 0.1 ms,
i.e. at any sane scan period it is the normal 7-segment switching and not chatter.

### Online rig (UI)

`./build/onlinetest`, `src/OnlineWindow.cpp/.h`. Left panel, right a 2x2 grid of charts.

- **Handwheel** — an absolute slider whose position *is* `TargetPosition`, sampled **inside the
  scan loop**, so the generator is handed a fresh target on every scan the way a real encoder read
  would. Drag it fast and the axis falls behind under its limits; let go and it settles on it. The
  spin box next to it is the same value typed in, which is how step inputs are given. `Strok ±`
  sets the slider span, `Rastgele adım` fires a random step.
- **Charts** — position (actual / target / predicted stop), velocity (actual / velocity command /
  ±limit), acceleration (±limits), jerk (±limits). Rolling window, width adjustable.
- **`Zaman ölçeği`** slows simulated time down to 0.02x so a transient can be watched; the scan
  period stays whatever `Tarama periyodu` says, only fewer scans run per frame.
- **`Tepe |v| / |a| / |j|`** are the peaks since the last reset — the read-out that proves the
  limits actually hold.
- Simulated time follows the wall clock with a cap of 40000 scans per frame, so a stalled frame
  cannot turn into a position jump.

### Not covered

- **Software position limits (`NegativeLimit` / `PositiveLimit`) are still not consulted.** The
  handwheel can drive the axis anywhere.
- `MaxVelocity` is respected as an envelope cap, but a start state whose acceleration already
  carries the axis past it will exceed it — nulling that acceleration costs `a^2/(2*jerk)` of
  velocity and nothing can prevent it. Same limitation as `Profilerold`.
- No feed override, no `MaxVelocity` change ramp: changing the limit mid-move is applied on the
  next scan with no smoothing (the jerk limit still holds, so it is not a step in the motion).
- The rig writes no log file; `braketest` still owns `braketest_log.csv`.

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
9. ~~**the `else` branch (v or a is zero) is still empty**~~ **FIXED** 2026-09-06, the online
   branch runs through it on every scan so it could not stay open. It also closed the one scan
   hole it punched in the middle of every other profile: while braking from a speeding-up state
   the acceleration passes through zero, `cur_acc * cur_vel` falls inside `±TOLERANCE` for that
   scan, and the live brake distance used to collapse to 0.

   Cross checked the way the rig does it — freeze `BrakeDistance`, then brake with the `Stopping`
   branch, which shares no formula with the tree — over all three sub cases and both signs. Error
   is `+2.05e-2 mm` at a 1 ms scan and `+1.4e-4 mm` at 0.01 ms, i.e. granularity. The rig case
   from before the fix, `v=100 a=0`, now predicts 15.00000 against a measured 14.97946.

10. **the `Stopping` branch cannot start from `v == 0`.** `dir = (cur_vel < 0.0) ? -1.0 : 1.0`
    makes `dir = +1` at standstill and the `(new_vel * dir) <= 0.0` test then declares standstill
    on the first scan, so the axis never moves. Found while cross checking bug 9: with
    `v=0, a=-800` the tree says 14.12267 mm (correct, it mirrors the `a=+800` case which measures
    14.10) but the motion covers 0.0004 mm. Only affects `Stopping`; the `Tracking` branch has its
    own integration and handles `v == 0` fine. Not fixed — reported.

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
*Last update: 2026-09-06 (later the same day) — **the online generator**.
`AxisState::Tracking` added and `GenerateTrajectory` extended with the online branch: per scan it
inverts the brake distance into an allowed velocity, turns that into a peak acceleration, and walks
the acceleration there with jerk — no stored profile, the target may move every scan. The empty
`else` branch of the brake tree (bug 9) was filled because the online loop runs through it every
scan, and cross checked against the motion. `Axis` gained `InPositionWindow`, `CommandedJerk`,
`VelocityCommand`. Second binary `onlinetest` with `src/OnlineWindow.cpp/.h`: absolute handwheel
slider sampled inside the scan loop, 4 charts, adjustable scan period and time scale. Three
defects were found and fixed during the work: the velocity envelope was being asked at the wrong
point (1.7 mm overshoot per step), `max_acc`/`max_dec` were picked by the sign of the acceleration
instead of by whether it fights the motion, and the square root law chattered the jerk at its limit
near zero velocity error. Bug 10 found and reported: the older `Stopping` branch cannot start from
`v == 0`. The user renamed `Profiler.*` to `Profilerold.*`; CMake and the include follow.*

*Previous update: 2026-09-06 — `src/Profiler.h/.cpp` added: a new point-to-point jerk-limited
profiler replacing the solver in `profilerold.cpp`. The six-branch `CalculateShorterProfile`
tree and its three 2x2 Newton-Raphson systems are gone, replaced by one `BuildRamp` primitive
and a bisection on the peak velocity; the missing 7th (cruise) segment, the direction sign and
an `Overshoot` flag were added, and `solveQuadratic` got its `a == 0` guard and a numerically
stable root form. Verified over 200k random states: distance / terminal velocity / terminal
acceleration all within 1e-11, no limit violated. `profilerold.cpp` left untouched as
reference. Both reference files' defect lists written down above. Nothing calls the new
profiler yet.*

*Previous update: 2026-08-17 — `MoveAxis` folded into `GenerateTrajectory` itself and removed as
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
