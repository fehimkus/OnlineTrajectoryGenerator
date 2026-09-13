# UltimateMotion — Claude working notes

## Rules for this file

- **Read this file at the start of every session.** Look here before reading any code.
- **Update this file at the end of every prompt.** New decisions, new formulas, bugs
  found or fixed, changes of approach -> write them into the matching section. If there
  is nothing to update, say "no update", do not skip it silently.
- Everything in this repo is written in **English**: this file, code comments, docs.

## Velocity mode — added 2026-09-13

```cpp
enum class MotionCommandKind { Position, Velocity };
struct MotionCommand { MotionCommandKind Type; double Target; };    // mm when Position, mm/s when Velocity

TrajectoryStep GenerateTrajectory(const MotionState&, const MotionLimits&,
                                  const MotionCommand&, double deltaTime);
```

The velocity command is what an `MC_MoveVelocity` or a jog is built on. It started as a second free
function, `GenerateVelocityStep`; on the user's call (2026-09-13) the two were merged into **one
entry point split by a single `if`**, with the command's kind named in a small struct so the unit of
`Target` is written at the call site rather than implied by a bare `double`. Behaviour is identical
either way — every number below was re-measured after the merge and none moved.

Both branches share one body, `ScanStep` (110 lines), which works in whatever direction frame the
caller picks. The position branch picks it from the position error and passes the remaining distance;
the velocity branch picks it from the **velocity** error and passes `diff_pos < 0` to mean "no target
to stop at". Sharing matters: the open signed-stop-displacement defect will be fixed once for both.

Why not express a velocity command as a position one (aim at the software limit, cap `MaxVelocity`
at `|v_cmd|`)? It nearly works and was considered, but it breaks on `v_cmd = 0` (`MaxVelocity = 0`
makes the core refuse to move at all), needs an infinite target when no software limits are set, and
pushes the fake target and its sign onto exactly the caller this is meant to spare.

In velocity mode the software limits are the **only** thing that ever stops the axis, which is the
strongest argument for having kept them in the core. `MotionLimits` gained `InVelocityWindow`
(1e-3 mm/s), `TrajectoryStep` gained `InVelocity` — the same park idea one axis up, because the
settling velocity otherwise only converges towards the command without ever reaching it.

Measured over 2000 runs per scan period, limits randomised, start states filtered to ones the limits
could actually have produced:

| | 2 ms | 1 ms | 0.2 ms |
|---|---|---|---|
| reached the command | **2000/2000** | **2000/2000** | **2000/2000** |
| `\|v - cmd\|` at the end | **0.0e+00** | **0.0e+00** | **0.0e+00** |
| wander after reaching | **0** | **0** | **0** |
| over `MaxVelocity` | **0.0e+00** | **0.0e+00** | **0.0e+00** |
| over `Jerk` | 1.1e-10 | 2.8e-10 | 2.1e-9 |

`200 -> -150 -> 0` stepped mid run at 1 ms lands exactly on all three with no limit exceeded; a
`+200 mm/s` command against a `+100 mm` software limit comes to rest at **100.000000 mm** (breach
3.7e-07); a command above `MaxVelocity` is clamped to it. **The position mode is byte-for-byte
unchanged** by the refactor and the fixes below.

### Three real defects found building it

1. **The acceleration clamp could break the jerk limit** — this one predated velocity mode and no
   earlier test had provoked it. If the caller hands in a state whose acceleration is outside the
   limit that currently applies, the final clamp moved it inside **in one scan**: measured jerk
   **3.7e6** against a limit of 5000. The clamp is now rate-limited to `jerk * deltaTime`, so an
   out-of-limit start acceleration is walked back instead of jumped. Worst jerk excursion 4e-10.
2. **The at-velocity park skipped the whole scan**, software limits included, so an axis cruising on
   command ignored them for ever — it ran 3860 mm past a +100 mm limit. The park is now only taken
   when the limits still allow another scan at that velocity.
3. **The fallback pointed the wrong way in velocity mode.** "Brake the motion the axis has" is right
   when the distance cannot be held; when it is the *velocity* bound that is broken, the thing to do
   is pull the settling velocity down. The axis was overshooting the command by exactly
   `max_dec^2/(2*jerk)` = 64 mm/s, one acceleration ramp. The fallback now looks at which constraint
   is actually broken.

### Open in velocity mode

- **The acceleration limit is exceeded transiently at a zero crossing.** An acceleration that was
  legally `max_dec` while slowing a backward motion becomes an *acceleration* the moment the velocity
  crosses zero, and if `max_acc` is smaller it is over it. The core walks it back at the jerk limit —
  dropping it instantly would break jerk, so the excursion is unavoidable and bounded by
  `max_dec - max_acc`.
- **Some reversing cases pass the command and come back** (worst ~280-400 mm/s with `MaxVelocity` up
  to 500). Reaching and holding are unaffected (2000/2000, zero wander). Not chased yet; likely the
  same root as the zero crossing above.
- The `MotionLimits` -> `MotionConstraints` rename was proposed and is still unanswered. The name
  matters because a motion block folds *command* dynamics and *axis* limits into these fields — the
  core does not care which is which, but the name suggests it only takes machine limits.

## Prior art worth knowing about

Jerk-limited **online trajectory generation** is a well worked field; this core is a good solution to
a known problem, not a new one. Reference points to measure against (not verified from inside this
session, but they exist and are worth reading):

- **Ruckig** (Berscheid & Kroger, 2021) — open source, jerk limited, **multi-axis with time
  synchronisation**, and it handles arbitrary *target* states (non-zero velocity and acceleration),
  not just target positions. Widely used in robotics.
- **Reflexxes Motion Libraries** (Kroger) — its predecessor, Type II / IV.
- Every large CNC and robot vendor has an in-house equivalent.

What this core does is the **single-axis, zero-target-velocity** case. That is a strictly easier
problem than the ones above, and the honest framing is that the value here is ownership,
auditability and the verification trail, not novelty.

**What would actually raise it, in order of impact:** (1) close the signed-stop-displacement defect —
it sits exactly on the selling point, the moving target; (2) multi-axis with time synchronisation,
which is the real step from component to product; (3) non-zero target velocity, which would let the
core guarantee a corner velocity instead of leaving blending to the caller; (4) a run on real
hardware with real jitter, which is worth more than the whole simulation table.

## README

`README.md` was rewritten 2026-09-13 and is now the outward-facing description: what the core is, why
scan-based, where it applies (CNC, robotics, mobile robots, gantries), a measured verification table,
and an explicit **what is not verified yet** section. Keep that honest section honest — the numbers in
it come from this file, so when a measurement here changes, the README changes with it.

`docs/rig.png` is a screenshot of the rig and `docs/rig.mp4` a 20 s recording of it (1600x950,
17 fps, 7.6 MB). Both were made **without touching the user's screen**, by running the binary under
`QT_QPA_PLATFORM=offscreen` with a temporary flag in `online_main.cpp`:

- `--shot <path>` clicks the random-step button, waits, calls `QWidget::grab()` and quits
- `--record <dir>` grabs a frame every 40 ms on a schedule — 0-5 s clicking the random-step button,
  5-15 s driving the handwheel `QSlider` along a two-frequency sine so it looks hand-swung, 15-20 s
  random again — then quits

Both flags were reverted after the capture; add them back the same way to refresh. The frames were
encoded with a throwaway Swift program using `AVAssetWriter` (no ffmpeg, no imageio and no Pillow on
this machine; Qt cannot write GIF either). Capture runs at ~17 fps, not the 40 ms the timer asks for,
because grabbing and saving a PNG that size takes longer — so encode at the measured rate, not the
nominal one, or the video plays fast. **Never use `screencapture` on the user's desktop for this.**

## What this repo is

**A trajectory core, not a motion controller.** It answers one question, once per scan: given where
the axis is, what it may not exceed, and where it is told to go, what is the state one scan later.

It owns nothing. There is no `Axis` struct any more (removed 2026-09-13 — live state, target and
parameters belong to the layer above), nothing is kept between calls, and nothing is written back:

```cpp
TrajectoryStep GenerateTrajectory(const MotionState& state, const MotionLimits& limits,
                                  double target, double deltaTime);
```

`MotionState` is position / velocity / acceleration. `MotionLimits` is the four dynamic limits, the
two software position limits and the in-position window. `TrajectoryStep` returns the state at the
end of the scan, the jerk that was really applied, the brake distance of the state it was entered
with, and an `InPosition` flag.

**What belongs to the caller** — the motion layer written on top: the state machine, `Done` / `Busy`
/ `Active` / `CommandAborted` / `Error`, buffer modes, blending and `MC_Stop` priority. Blending needs
nothing new from the core: handing in a fresh target on any scan is exactly what the core is built
for, and the caller picks the switch instant to get the corner velocity it wants. A **feed override**
is likewise just a scaled `MaxVelocity` handed in — a `FeedOverride` field was added and removed the
same day once that was clear.

**`deltaTime` is a plain parameter.** Calling the core on a deterministic cycle is the integrator's
responsibility, not the core's.

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
| `src/TrajectoryGenerator.h` | **The core's whole interface**: `MotionState`, `MotionLimits`, `TrajectoryStep`, `GenerateTrajectory`, `BrakeDistance`, `solveQuadratic`, `TOLERANCE`, `BISECTION_STEPS` |
| `src/TrajectoryGenerator.cpp` | **The file being worked on.** The brake distance tree and the scan's jerk search |
| `src/OnlineWindow.cpp/.h` | Qt6 UI: **the test rig**, handwheel slider + 4 charts (position/velocity/acceleration/jerk) |
| `src/online_main.cpp` | Entry point of the rig |
| `docs/*.drawio` | Jerk-limited profile diagrams, decision tree draft |

That is the whole tree now. `Axis.h`, `MainWindow.*`, `main.cpp`, `Profilerold.*` and `sample.cpp`
were all removed on 2026-09-13 (`Axis.h` and the brake rig by this session, the rest by the user).
The sections further down that discuss `Profilerold` and `sample.cpp` are kept because their findings
still matter — but the files are gone, so treat them as history, not as things to read.

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
./build/onlinetest        # the only binary: the test rig for the core
```

`braketest` was removed 2026-09-13 — it drove the core through `AxisState::Stopping` and waited for
`Idle`, neither of which exists any more, so it had silently stopped testing anything. The rig shows
the brake distance anyway. Renaming a target means the CMake cache has to be thrown away
(`rm -rf build`) before reconfiguring.

Syntax check only:

```bash
g++ -fsyntax-only -std=c++17 -Isrc src/TrajectoryGenerator.cpp
```

Requirements: `qt6-base-dev qt6-charts-dev cmake g++`. C++17.

On **macOS** those Debian packages do not exist; Qt6 comes from Homebrew as one keg-only
formula that already carries QtCharts:

```bash
brew install qt                                        # 6.11.2, arm64 bottle
cmake -B build -S . -DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt
cmake --build build -j
```

`CMAKE_PREFIX_PATH` is needed because the formula is keg-only — without it `find_package(Qt6)`
does not see it. Both binaries build clean on Apple clang (2026-09-13).

## Code style (derived from sample.cpp)

- Comments are **English, lowercase, short**, placed at the end of the line with `//`.
  Do not write long derivations or explanation blocks — the user does not want them,
  one line is enough.
- Allman braces (`{` on its own line), 4 space indent.
- Formulas are written out **fully parenthesized and unsimplified**, and left inline where they are
  used. Pulling a *reusable quantity* out into its own function is fine when something else has to
  evaluate it — `BrakeDistance` is one, the scan-loop feasibility search has to call it on a
  predicted state. (The old rule said never to extract; dropped 2026-09-13 on the user's call.)
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

11. ~~**unqualified `abs(...)` at lines 270/273**~~ **FIXED** on request 2026-09-13, both are
    `std::abs` now. On macOS the old form happened to be safe — libc++'s `<stdlib.h>` adds
    `abs(double)` overloads, which is why the editor's inlay hint showed that call's parameter as
    `__lcpp_x` — but with a toolchain that only pulls in C's `<stdlib.h>` it binds `abs(int)` and
    truncates `cur_acc` to a whole number.

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

## New architecture attempt (in progress, 2026-09-13)

The user is rewriting the online decision tree by hand inside `GenerateTrajectory`; the previous
`Tracking` / `Stopping` motion sections were removed and the bottom of the file is an unfinished
`if (diff_pos > TOLERANCE)` block (does not compile yet — that is work in progress, not a defect).
Two new top level locals:

- `diff_pos = TargetPosition - CurrentPosition` — remaining distance, signed
- `brake_velocity = cur_vel + (cur_acc * acc_abs) / (2 * jerk)` — the velocity the axis ends up
  with if the jerk starts zeroing the acceleration right now. Signed, valid in every branch, no
  speeding-up / slowing-down split. Same quantity as `delta_vel_zero` in the Tracking notes and
  `dv_zero` in `Profilerold::BuildRamp`, and identical to the speeding-up branch's `v1`
  (checked: worst difference 1.1e-13 over the v/a grid).

The concept is right — the brake envelope can only be asked at the `a = 0` point, and this is the
velocity there. Naming note: it is a *lower* bound while speeding up and an *upper* bound while
slowing down, so "min hiz" only holds in one branch.

Fixed on request the same day (first draft of the variable):

- it was declared `const` and assigned later, with `Jerk` instead of `jerk` and no `;` — the file
  did not build
- it was filled **inside the speeding-up branch only**, so the other three branches saw `0.0`;
  moved out of the tree, computed once next to `diff_pos`
- the formula was dimensionally wrong: `0.5*a*a*t*t` (that is `a^2*t^2`, the position polynomial's
  shape) where the velocity polynomial needs `a*t`, and `0.1666667*jerk*t^3` (a distance) where it
  needs `0.5*jerk*t^2`. `t` was written `cur_acc/jerk`, negative for a negative acceleration, and
  the jerk term carried no sign, so it grew the acceleration instead of zeroing it.

One signed `brake_velocity` answers both branch questions; a second velocity is not needed
(asked 2026-09-13). Speeding up it is "where the velocity settles if the acceleration is eased
now"; slowing down the same expression equals `sign(v) * (vel_abs - vel_critic)`, so "release the
jerk now and land exactly on zero" is simply `brake_velocity == 0`. Checked over 1567 slowing-down
states, worst difference 1.1e-13. Its **sign** then picks the existing sub-branches for free:
same sign as `cur_vel` = braking not enough, zero = exactly critical, opposite sign = over braking.
In that last case the value is not an attainable velocity (the velocity crosses zero before the
acceleration is nulled) — only the sign is usable, the real answer comes from the `solveQuadratic`
over-braking branch.

What actually differs per branch is not the velocity but (a) the distance `x_null`, (b) which limit
applies, `max_acc` or `max_dec` — decided by whether the acceleration feeds or fights the motion,
never by its sign, and (c) `vel_allowed`, the envelope inverted out of the remaining distance.
That third one is a genuinely separate velocity and is still missing: `brake_velocity` says where
the axis is forced to go, `vel_allowed` says where it is permitted to go, and the decision is the
comparison of the two.

Still open: the matching **distance** `x_null = v*t_null + (1/3)*a*t_null^2` is not computed. The
Tracking notes say forgetting it costs exactly one acceleration ramp of overshoot (1.7 mm per step
on the rig). Not added — it was not asked for.

### Motion block at the bottom — written 2026-09-13

The `if (diff_pos > TOLERANCE) / if (diff_pos > brake_distance)` block, i.e. "target ahead, the
distance is not the constraint yet, run up to `max_vel`". The other cases are still empty on
purpose. Two rounds of review then a fix applied on request.

`brake_velocity` was re-purposed by the user and is now the **switching threshold on `cur_vel`**,
not a velocity the axis will reach: `brake_velocity = max_vel - (cur_acc*acc_abs)/(2*jerk)`.
Same inequality as before, moved to the other side — `cur_vel + a*|a|/(2*jerk) <= max_vel`. The
term has to stay **signed** (`cur_acc * acc_abs`); `acc_abs * acc_abs` moves the threshold the
wrong way whenever `cur_acc < 0`, which is exactly the tail of every ramp.

What the block does now, in order:

1. bang-bang target from the threshold: `acc_target = max_acc` below it, `0` above it — the upper
   side only has to **null** the acceleration, nothing in an accelerating block asks for a
   deceleration
2. `acc_exact = 2*(max_vel - cur_vel)/deltaTime - cur_acc` — the acceleration that lands the
   velocity exactly on `max_vel` at the end of this scan; `acc_target` is never above it
3. `acc_target` clamped to `[-max_dec, max_acc]`. `max_dec` appears **only** as this floor, and
   only bites when `acc_exact` pulls the target negative — that is a start state already above
   `max_vel`, where coming back down really is a deceleration
4. jerk **walks** `cur_acc` towards `acc_target` by at most `jerk*deltaTime` and stops exactly on it
5. `jerk_cmd = (acc_next - cur_acc)/deltaTime`, then `x/v/a` integrated with the full polynomials

Defects found and fixed in this round:

- `|cur_acc| < max_acc` was used as a **gate on both directions**, so the acceleration froze at
  `max_acc` for ever and the jerk-down phase never started. Measured before the fix
  (`max_vel=200 max_acc=1000 jerk=5000`, 1 ms, from rest): `a` stuck at 1000, `v` stuck at 200.5,
  position running away linearly. It is a clamp on the value now, not a gate on the branch.
- `if (|cur_vel| < max_vel)` latched the velocity: once over the limit it was never written again.
  Removed — the velocity is always integrated, `acc_exact` is what keeps it on the limit.
- Euler (`v += a*dt`, `x += v*dt`) replaced by the full polynomials with the real `jerk_cmd`.

**Why step 4 exists:** applying `acc_exact` directly as the new acceleration respects `max_vel`
perfectly (peak `|v|` = 200.000000000) but asks for an acceleration change of any size in one scan
— measured `|jerk|` up to 200000 against a limit of 5000. The jerk limit can only be held by
walking towards a target, never by writing a computed acceleration straight out.

Verified (same limits, run to `max_vel`, simulation of the exact code):

| check | 2 ms | 1 ms | 0.2 ms | 0.1 ms |
|---|---|---|---|---|
| velocity over `max_vel` | 2.0 | 0.0 | 0.17 | 0.10 |
| `|a|` over `max_acc` | 0 | 0 | 0 | 0 |
| `|jerk|` over `Jerk` | 0 | 0 | 0 | 0 |

Start states that already sit above `max_vel` come back down to it; the excursion reported for
those is the start state itself.

**The `-max_dec` target was a defect, caught by the user.** The first version of step 1 asked for
`-max_dec` above the threshold. That drives the acceleration *below* zero, the velocity then falls
back under the threshold, the target flips to `+max_acc`, and the pair oscillates: parked on
`max_vel` the jerk sat pinned at its limit for ever (+/-5 in `a` at 1 ms). It was reported as an
open design problem — wrongly, it was just the wrong target. With `0` there:

| parked on `max_vel`, 1 ms | target `-max_dec` | target `0` |
|---|---|---|
| `\|jerk\|` | 5000, pinned | 0.06 |
| `\|a\|` | 5 | 3e-5 |

and every start state, including `v0 = 250` above the limit, settles on exactly 200.000000000.

### The braking side, `diff_pos <= brake_distance`

Written on request the same day. **Same five steps as the accelerating side**; the only thing that
differs is where the envelope velocity comes from — `max_vel` there, the remaining distance here:

```
diff_pos > max_dec^3/jerk^2 ? solveQuadratic(1/(2*max_dec), max_dec/(2*jerk), -diff_pos)  // trapezoidal
                            : cbrt(diff_pos*diff_pos*jerk)                                // triangular
```

both inverses of the brake distance tree at the top of the file, then capped by `max_vel`. The
threshold is the same expression `brake_velocity` is, written against `vel_allowed`:
`vel_threshold = vel_allowed - (cur_acc*acc_abs)/(2*jerk)`. Targets: `0` under the threshold
(coasting is enough, the distance shrinks and the envelope comes down to meet the velocity),
`-max_dec` over it — here the velocity really is coming down, so the floor is the honest limit.
`acc_exact` is against `vel_allowed` instead of `max_vel`, the clamp, the jerk walk and the
polynomials are identical. `axis.VelocityCommand` is now filled in on both sides.

Verified against the real code (a driver linked to `TrajectoryGenerator.cpp`, not a re-implementation):

| 100 mm from rest | 2 ms | 1 ms | 0.2 ms | 0.1 ms |
|---|---|---|---|---|
| distance left at the target | 3.5e-3 | 9.7e-4 | 2.0e-6 | 8.2e-7 |
| overshoot | 3.5e-3 | 9.7e-4 | 2.0e-6 | 0 |

`dt^2`, so granularity. 20000 random `(target, v0, a0)` at 1 ms: no acceleration over `max_acc`
while the acceleration feeds the motion, jerk over the limit 5.6e-11, worst overshoot 0.087 mm.

**Found in the sweep, reported, not fixed:**

### Which limit applies — the user's rule, 2026-09-13

**Forward or backward, `max_acc` while the speed grows, `max_dec` while it shrinks.** Not the sign
of the acceleration, and not "feeds/fights" spelled out per branch — one rule, two locals computed
once inside `if (diff_pos > TOLERANCE)`:

```cpp
acc_limit_up   = (cur_vel < -TOLERANCE) ? max_dec : max_acc;   // a positive acceleration grows a forward motion, shrinks a backward one
acc_limit_down = (cur_vel >  TOLERANCE) ? max_dec : max_acc;   // a negative acceleration shrinks a forward motion, grows a backward one
```

Both branches clamp with these instead of `max_acc` / `max_dec`. Before the rule, an axis running
backwards towards a target ahead of it took `a = 1000` against a `max_dec` of 800 the whole way from
`v = -150` to `v = 0`; it is 800 now. Over the 20000 state sweep, after the start state's own
acceleration has been walked down: **0** over `max_acc` while speeding up, **0** over `max_dec` while
slowing down. Worst overshoot fell from 0.087 to 0.058 mm.

On the accelerating side the floor is now an explicit branch rather than a bare `-max_dec`, because
seeing a deceleration limit in an accelerating block reads wrong (the user's point):

```cpp
if (cur_vel > max_vel)  acc_target = std::max(-acc_limit_down, acc_target);   // the velocity itself has to come down
else                    acc_target = std::max(0.0, acc_target);              // nothing else here asks for a deceleration
```

Behaviour is unchanged — measured before the rewrite: the floor never altered the output while the
acceleration ramps down to zero (0 of 186176 accelerating states, 0 of 166652 states entering with
`a < 0` below `max_vel`), it only bit in 2547 of 47172 states with `cur_vel > max_vel`.

### The axis hunts around the target — open

Traced at 1 ms, 100 mm from rest, the last 80 scans. The envelope `vel_allowed = cbrt(d^2*jerk)`
is a cube root, so it collapses faster than any scan can follow:

| remaining `d` | 10 mm | 1 mm | 0.1 | 0.01 | 0.001 | 0.0001 |
|---|---|---|---|---|---|---|
| `vel_allowed` | 79.37 | 17.10 | 3.68 | 0.794 | 0.171 | 0.037 |

At 1 mm/s a 1 ms scan already covers 1e-3 mm, ten times the `InPositionWindow`, so the last stretch
cannot be resolved at all. What the trace shows is not a gentle undershoot but a **limit cycle**:
the velocity overruns the envelope, full braking takes it through zero to `-1.59 mm/s`, the axis is
now moving *away* from the target so `brake_distance` no longer covers `diff_pos` and the
**accelerating** branch takes over (`VelocityCommand` jumps back to `max_vel`), it runs forward to
`+2.05 mm/s`, overruns the envelope again. The position wanders between 0.013 and 0.05 mm of the
target until some scan happens to land inside `TOLERANCE`, and it leaves that branch still carrying
1.41 mm/s.

This is the same infinite-gain-at-the-target behaviour the Tracking notes record (there: a stable
18-scan limit cycle with the jerk pinned). `Axis.InPositionWindow` (1e-4 mm) exists for exactly this
and is **not used yet** — inside the window, with a velocity small enough that its own brake distance
also fits inside it, the axis should be declared on target and parked at `v = 0, a = 0`. Not added,
it is a new behaviour.

Velocity over `max_vel` in the sweep is 90.7 mm/s and comes only from start states whose own
acceleration already carries them past the limit (`a0^2/(2*jerk)`, up to 100 mm/s for `a0 = 1000`).
Same limitation as `Profilerold` and the Tracking branch.

### One tree, solved in the direction of travel — 2026-09-13

`diff_pos` is `std::abs(TargetPosition - CurrentPosition)`, so it is **always positive** and there is
no mirrored branch: a second `else if (diff_pos < -TOLERANCE)` was written and then deleted on the
user's call. Instead the motion runs in the direction of travel, the same way `Profilerold` does:

```cpp
dir     = (TargetPosition < CurrentPosition) ? -1.0 : 1.0;
vel_dir = cur_vel * dir;        // velocity towards the target
acc_dir = cur_acc * dir;        // acceleration towards the target
```

Every comparison, threshold and clamp inside the tree is written against `vel_dir` / `acc_dir`, and
`dir` is put back on position, velocity, acceleration, `CommandedJerk` and `VelocityCommand` when
they are written. `brake_velocity` uses `acc_dir` too. Nothing about the logic changed, only the
frame. `acc_limit_up` / `acc_limit_down` now read `vel_dir`, so the limit rule covers both directions
in one line.

Verified: 5000 mirrored pairs (the same move forwards and backwards, every sign flipped), worst
`|forward + backward|` over the whole run **0.000e+00** — the two directions are bit-identical.

### In position window — added so the rig can settle

A branch in front of the tree:

```cpp
diff_pos < InPositionWindow  &&  brake_distance < InPositionWindow  &&  acc_abs <= jerk*deltaTime
    -> v = 0, a = 0, jerk = 0, return
```

Without it the axis never settles. Measured at 1 ms: the residual velocity around the target is
~0.3 mm/s, so one scan moves 3e-4 mm while `TOLERANCE` is 1e-6 mm — the old park condition could
not be hit and the axis hunted in a limit cycle of about 1e-3 mm. Over 20000 random
`(target, v0, a0)` at 1 ms:

| `InPositionWindow` | settled | still hunting |
|---|---|---|
| 1e-4 mm (old default) | 17596 | 2404 |
| 1e-2 mm | 20000 | 0 |

so the online rig's `Konum penceresi` default was raised from 1e-4 to 1e-2 mm
(`OnlineWindow.cpp`). That is the only change the rig needed; everything else in it was already
written.

### Why the axis rings on a step — investigated 2026-09-13, nothing changed

Seen on the online rig: after a step the position oscillates for a second or two, the velocity
command draws vertical lines between +/-`max_vel` and the jerk sits saturated. Reproduced exactly
(-30 -> 23.5255 mm, 1 ms, `max_vel=200 max_acc=1000 max_dec=800 jerk=5000`). It is not numerical
noise, it is **the two branches fighting**:

| t | err | v | a | brake_dist | vel_allowed | vel_threshold | err-bd | branch |
|---|---|---|---|---|---|---|---|---|
| 0.540 | +3.328 | +65.1 | -800 | 3.570 | 38.1 | 102.1 | -0.241 | brake |
| 0.584 | +1.191 | +33.5 | -610 | 1.193 | 19.2 | 56.4 | -0.001 | brake |
| 0.588 | +1.062 | +31.1 | -590 | 1.061 | 17.8 | 52.6 | **+0.001** | **ACCEL** |
| 0.668 | +0.033 | -0.065 | -190 | 0.000 | | | +0.033 | ACCEL |

1. The braking branch drives the velocity onto `vel_allowed(diff_pos)`, which is the envelope for an
   axis at **zero acceleration**, while the axis is carrying `a = -800`. `acc_exact` therefore asks
   for -53200 mm/s^2 at t=0.540 and is clamped to `-max_dec`: full braking.
2. In the same scan the branch's own test says the opposite — `vel_dir = 65.1 < vel_threshold = 102.1`
   picks `acc_target = 0`, "coasting is enough" — and `acc_exact` overrides it. **The decision and the
   clamp target two different velocities**: `vel_threshold` carries the `a^2/(2*jerk)` credit,
   `vel_allowed` does not. In any real approach `acc_exact` always wins, so the coasting sub-branch is
   dead code.
3. Braking onto the `a = 0` envelope while still decelerating is over-braking, so the margin
   `diff_pos - brake_distance` grows monotonically and crosses zero (-0.241 -> +0.001 above).
4. The top level test then selects the **accelerating** branch, which has no notion of being nearly
   there: it asks for `max_vel` / `max_acc` and the jerk unwinds the accumulated deceleration at full
   rate (-590 -> -190 over 80 scans) while the axis is still closing. It stops 0.033 mm short with
   -190 still on it, is pulled backwards, and the pair repeats. Damped, but slow: 12 zero crossings
   over 0.8 s here.

Prevalence, 2000 forward steps from rest (1..400 mm) at 1 ms: **1998** of them change branch more
than twice over the last 5 mm, worst 60 changes, worst backwards velocity on a purely forward move
**64.8 mm/s**.

Root cause: the envelope is asked **at the current distance**, with no correction for the ground
covered while the acceleration is nulled — the `x_null = v*t_null + (1/3)*a*t_null^2` term this file
already lists as missing. The branch test asks one question (`brake_distance`, acceleration included)
and the clamp asks another (`vel_allowed`, acceleration ignored); the two describe different
switching surfaces, so the axis drifts off the one it is regulating to. There is also no hysteresis:
on the envelope `brake_distance == diff_pos` by construction, so the test sits exactly on its own
switching surface for the whole approach.

**The `x_null` correction was tried on request and REVERTED — it does not hold on the braking side.**
Asking the envelope at the `a = 0` point assumes the axis passes through `a = 0` before the target.
On a hard brake it does not: the acceleration is meant to sit at `-max_dec` and only return to zero
*at* the target. Measured on the -30 -> 23.5255 step, the whole approach has `x_null > diff_pos`
(2.32 mm of ground needed to null `a = -710` against 2.18 mm remaining), so `diff_pos_zero` stays
negative, `vel_allowed` collapses to 0 and the threshold degenerates to `-a*|a|/(2*jerk) = +a^2/(2*jerk)`
= +50.4 mm/s against `v = 49.9` — the test means "you cannot stop" but the branch reads it as "coasting
is enough" and *releases* the brake. Ringing dropped (worst backwards velocity 64.8 -> 41.3 mm/s,
branch changes 60 -> 38) but 1987 of 2000 moves still rang, so the patch was backed out.

**What is actually missing: `vel_stoppable(d, a)`** — the velocity from which the axis can still stop
in `d` *given the acceleration it already has*. The `a = 0` envelope `vel_allowed(d)` is only its
`a = 0` special case, and on a real brake trajectory the velocity is systematically **above** it
(the axis is already decelerating, so it needs less than a standing start would), which is exactly why
regulating onto `vel_allowed` over-brakes and hands the margin to the accelerating branch.

It is the inverse of the brake tree already at the top of this file, and for fixed `a` the trapezoidal
brake distance is **quadratic in `v`**, so `solveQuadratic` inverts it. With `A = |a|`, `D = max_dec`,
`J = jerk`, `t1 = (D - A)/J`, `dv1 = A*t1 + 0.5*J*t1^2`, `dv3 = D^2/(2*J)`:

```
brake_distance(v, a) = (v*t1 - 0.5*A*t1^2 - (1/6)*J*t1^3) + ((v - dv1)^2 - dv3^2)/(2*D) + D^3/(6*J^2)

solveQuadratic( 1/(2*D),
                t1 - dv1/D,
                -0.5*A*t1^2 - (1/6)*J*t1^3 + (dv1^2 - dv3^2)/(2*D) + D^3/(6*J^2) - d )
```

Verified over 10974 random trapezoidal states: the closed form agrees with the tree in this file to
**6.8e-7 mm**, and the inversion returns the original velocity to **8.4e-6 mm/s**. The triangular case
(peak deceleration below `max_dec`) and the `a > 0` case still need their own inversions; only the
trapezoidal one has been checked.

Both branches need it, and that is what stops them fighting: the braking branch regulates `vel_dir`
onto `vel_stoppable(d, a)` instead of `vel_allowed(d)`, and the accelerating branch caps its command
with the same quantity instead of jumping to `max_vel`, so the two meet continuously on the surface
the top level test is already using.

**`TOLERANCE` was suspected and ruled out (2026-09-13).** Three experiments, none of which moves the
ringing:

| experiment | result |
|---|---|
| value swept 1e-12 .. 1e-6 | unchanged: 62 vs 60 branch changes, 65.6 vs 64.8 mm/s backwards |
| the dimensional misuse removed (`cur_acc * cur_vel` tested against `0.0`) | unchanged: 62 changes, 65.6 mm/s |
| value raised to 1e-1 | ringing "disappears" only because the axis parks 0.1 mm out; at 1e-3 the moves that still ring have the same worst backwards velocity, 64.79 mm/s |

Two real defects in how `TOLERANCE` is used did come out of it, both separate from the ringing:

- **Dimensions.** Of the 15 uses only `diff_pos > TOLERANCE` is a length. The others measure
  `mm^2/s^3` (`cur_acc * cur_vel`, lines 50/85), `mm/s` (lines 61/92/160/198/229/284/285) and
  `mm/s^2` (lines 95/130) against the same 1e-6. Harmless at these magnitudes — over 3.76 M scans only
  2505 fall inside the `|v| < TOLERANCE` band, and the `|a*v|` band is entered during cruise where the
  acceleration is exactly zero anyway — but it **moves with the unit scale**: in metres instead of
  millimetres the velocities are 1000x smaller and the `a*v` band swallows states 1e6 times larger.
- `diff_pos > TOLERANCE` doubles as the in-position deadband, and 1e-6 mm is far below what one scan
  can resolve (a scan at the residual 0.3 mm/s covers 3e-4 mm). That is why the separate
  `InPositionWindow` branch had to be added.

### The ringing fix, applied 2026-09-13

**The `diff_pos > brake_distance` branch split is gone.** One law now runs the whole motion:

1. `t_null = acc_abs/jerk`, `x_null = vel_dir*t_null + (1/3)*acc_dir*t_null^2`,
   `diff_pos_zero = diff_pos - x_null` — the envelope only knows how to brake from a standing
   acceleration, so the ground covered while the jerk nulls the one the axis has comes off first
2. `vel_allowed` from `diff_pos_zero` (0 if it is negative, else the trapezoidal `solveQuadratic`
   or the triangular `cbrt`), capped by `max_vel` -> `vel_command`
3. the `BuildRamp` peak-acceleration law for the ramp `(vel_dir, acc_dir) -> (vel_command, 0)`,
   picked by `delta_vel` against `delta_vel_zero = acc_dir*acc_abs/(2*jerk)`
4. `acc_exact` clamp, then **the guard**: `diff_pos_zero <= 0` forces `acc_target = -acc_limit_down`
5. limit clamp, jerk walk, full polynomials

**Step 4's guard is what makes it work.** Without it the ramp law reads a standing `-800` as "you are
braking harder than needed" and releases it to `-455` while the axis is 0.03 mm from the target — a
velocity-regulating law has no reason to care where it stops. Traced; overshoot doubled without it.

Measured, `max_vel=200 max_acc=1000 max_dec=800 jerk=5000`:

| | before | after |
|---|---|---|
| the rig's step (-30 -> 23.5255), zero crossings | 12 | **3** |
| the same, time to park | 1.434 s | **0.826 s** |
| worst backwards velocity on a forward move, 2 ms | 65.42 | **15.05** |
| the same, 1 ms / 0.2 ms / 0.1 ms | 64.78 / 4.56 / 1.88 | **7.62 / 1.70 / 0.84** |
| overshoot, 2 ms / 1 ms | 2.660 / 0.265 | **0.685 / 0.246** |
| overshoot, 0.2 ms / 0.1 ms | 0.0009 / 0.000 | 0.0225 / 0.0067 |

Limits: 0 over `max_acc` while speeding up, 0 over `max_dec` while slowing down, jerk over the limit
5.7e-10. Mirror symmetry still exact (5000 pairs, 0.000e+00). Both directions, 20000 random states at
a 1e-2 window: 20000/20000 park.

**The trade is honest:** the ringing is 4-8x smaller at every scan period and still scales with it,
but the fine-scan overshoot got worse (0.0009 -> 0.0225 mm at 0.2 ms). 86 of 2000 moves still stray
off the surface.

### The single overshoot after that — `x_null` was measuring a ramp that never happens

Seen on the rig: no more repeated ringing, but one overshoot, with the acceleration going
negative -> positive -> negative -> zero. Traced on a 77 mm step at 1 ms:

```
   t        err        v        a      x_null     d_zero     v_cmd
 0.641  +2.85295  +60.445  -800.00   +2.84453   +0.00842   +0.7077
 0.691  +0.72654  +26.695  -550.00   +0.71812   +0.00842   +0.7077
 0.735  +0.01337   +7.335  -330.00   +0.00495   +0.00842   +0.7077
 0.739  -0.01338   +6.055  -310.00      <- crosses the target still doing 6 mm/s with -310 on it
 0.763  -0.08094   +0.055  -190.00      <- worst overshoot
```

`d_zero` is pinned at **exactly** +0.00842 and `v_cmd` at +0.7077 for ninety-odd scans while the
brake is released at full jerk: a **sliding mode**. `x_null` depends on the acceleration, so
`vel_command` does too — the same self-reference that made `vel_stoppable` unusable, only milder.
Releasing the brake shrinks `x_null`, which grows `d_zero`, which raises the command, which justifies
the release; the loop balances `d_zero` at a constant instead of driving it to zero.

But the deeper error is that from t=0.641 onwards `v = 60.4` against `a^2/(2*jerk) = 64` — the axis is
**already over braking**, it will stop before the acceleration is nulled, so the ramp `x_null` measures
never happens. Fixed: in that regime `t_null` is the time to **standstill**, the smaller root of
`solveQuadratic(0.5*jerk, -acc_abs, vel_dir)` — the same solve the brake distance tree already uses —
and `x_null` is written with the full polynomial including the jerk term.

| 500 forward steps | before | after |
|---|---|---|
| overshoot @1 ms | 0.246 mm | **0.0168** |
| overshoot @0.2 ms | 0.0225 | **0.00006** |
| zero crossings, total @1 ms | 1121 | **807** |
| the same @0.2 ms | 379 | **28** |
| the rig's step, crossings | 3 | **1** |

Limits stay clean (0 over `max_acc`, 0 over `max_dec`, jerk 5.7e-10) and mirror symmetry is still
exactly 0. **It regressed two things:** the worst backwards velocity went 7.6 -> 17.2 mm/s at 1 ms
(the axis now stops slightly *short* and makes a small approach hunt instead of overshooting), the
worst single move went from 5 to 11 crossings, and 1 of 20000 states no longer settles (0.138 mm out).
Kept because the formula is simply correct where the old one was not, and overshoot is the more
serious failure for a motion controller.

**Removing `x_null` is not an option** — measured: overshoot goes from 0.246 mm to **32.9 mm** and
386 of 500 moves stop settling. The self-reference is load-bearing.

### `brake_distance` is correct — checked against integration and against Profilerold, 2026-09-13

Suspected on the rig, ruled out. The brake tree was compared with a 2e-7 s integration of the optimal
brake over 2991 random `(v, a)` states, and with `Profilerold::BrakeDistance`:

```
  GenerateTrajectory tree     : worst difference  0.0000 mm
  Profilerold BrakeDistance   : worst difference 24.1564 mm   (at v = 6.88, a = -993.86)
```

| v0 | a0 | integrated | tree | Profilerold |
|---|---|---|---|---|
| -199.971 | 0 | 40.9904 | **40.9904** | 39.9913 |
| -199.917 | +616 | 26.6431 | **26.6431** | 50.6644 |
| -199.135 | -997.66 | 132.6186 | **132.6186** | 127.4417 |
| +150 | -300 | 19.1606 | **19.1606** | 19.1606 |

So the copying would have to go the other way: **`Profilerold::BrakeDistance` is the wrong one.** It
breaks when the velocity and the acceleration have opposite signs — its `v_low = v_zero` choice and
the assumption that `f(v_peak)` is monotone do not hold for a state that has to reverse, so it plans
"null the acceleration first, then brake" where keeping the existing deceleration is shorter (26.64
against 50.66 above). This does not contradict the 200k verification recorded earlier in this file:
that measured `TotalDistance` landing on `TargetDistance`, not `BrakeDistance`, which the profiler
only uses to raise its `Overshoot` flag.

**And `brake_distance` is no longer in the control path.** Since the branch split was removed its only
use is the in-position park test — `diff_pos < InPositionWindow && brake_distance < InPositionWindow
&& acc_abs <= jerk*deltaTime`. The motion is driven by `vel_command` -> ramp law -> jerk walk, so even
a wrong brake distance could not produce the remaining overshoot. What is left is the `x_null` ->
`vel_command` -> `a` -> `x_null` feedback written up above.

### Root cause of the remaining overshoot, both phases — diagnosed 2026-09-13

**Every constraint is tested against the state at the start of the scan, but the jerk chosen binds
the state at the end of it.** One scan of lateness is one scan of excess, and a jerk limited axis
cannot give it back.

*Speeding up:* the ideal profile holds `a = max_acc` until `v = 100` and then applies full negative
jerk for 0.2 s, landing on `v = 200` with `a = 0`. The switch is noticed one scan late, so the axis
gains `max_acc * deltaTime = 1.0 mm/s` that can never be returned — and 1.0 mm/s is exactly the
measured excursion. In the trace, at `a = 240` the ideal has `v = 194.24` and the real axis `v = 195.24`.

*Slowing down:* the same with distance — the brake is started one scan late and the extra ground is
not recoverable, so the axis passes the target.

**Demonstrated cure.** A prototype that picks, every scan, the **largest jerk whose end-of-scan state
is still feasible**:

```
v' + a'|a'|/(2*jerk) <= max_vel        the velocity limit
brake_distance(v', a') <= d'           the position
-max_dec <= a' <= max_acc              the acceleration
```

and, when nothing is feasible, brakes the motion it has as hard as allowed (the fallback matters: an
earlier version fell back to `-jerk`, which accelerates an axis that is already running *away* from
the target — it diverged to 9300 mm/s).

| 200 moves, same seed | current code | prototype |
|---|---|---|
| overshoot @2 / 1 / 0.2 ms | 0.394377 / 0.016735 / 0.000001 mm | **0.000000 / 0.000000 / 0.000000** |
| backwards velocity @1 ms | 17.22 mm/s | **0.0295** |
| zero crossings @1 ms | 323 | **0** |
| over `max_vel` @1 ms | 1.0 mm/s | **4.3e-13** |
| distance left @1 ms | 0.0096 mm | **0.00006** |

Acceleration limit exceeded 0, jerk 5.6e-11. The overshoot disappears in **both** phases and not one
zero crossing is left.

This also puts `brake_distance` back in the control path — as a **feasibility test on the predicted
state**, which is what it is good for, not as a regulation target (that is the use that fed back on
itself and failed). It was already verified exact against integration.

**Two structural changes it needs, not made yet:** the brake tree has to become a callable function
(it is inline in `GenerateTrajectory` today) so the predicted state can be tested, and each scan needs
a fixed-step bisection on the jerk (50 steps in the prototype; `Profilerold` already uses a 100 step
bisection, so there is precedent, but it sits against this file's "do not extract formulas into
helpers" note).

### The end-of-scan feasibility search, applied 2026-09-13

The brake tree is now a function — `double BrakeDistance(double cur_vel, double cur_acc, double
max_dec, double jerk)` in `TrajectoryGenerator.h/.cpp` — because the scan loop has to evaluate it on a
*predicted* state. `BISECTION_STEPS = 50` sits next to `TOLERANCE`; `Profilerold` already carries a
100 step bisection, so there is precedent in the project. The "never extract a formula" style rule was
dropped on the user's call.

`GenerateTrajectory`'s motion section is now: park if in position, otherwise **search the jerk** —
the largest one whose end-of-scan state still satisfies

```
-acc_limit_down <= a' <= acc_limit_up                 (the speed-based limit rule, restored)
v' + a'|a'|/(2*jerk) <= max_vel
BrakeDistance(v', a') <= d'
```

falling back, when nothing is feasible, to braking the motion the axis has as hard as allowed.

**What the search is really finding is the switch instant, not a jerk level.** The jerk is a constant
`±Jerk`; what is unknown each scan is *when inside the scan* it should switch, and that instant almost
never lands on a scan boundary. Applying `+Jerk` for 0.4 of a scan and `0` for the rest raises the
acceleration by the same amount as applying `2000` for the whole scan — so an intermediate value is
how "the switch fell 40% into this scan" is written down, not a new jerk level. The bisection is a
stopwatch, not a control law: 15 halvings pin the switch instant to 1/32768 of a scan.

`BISECTION_STEPS` is **not** a lookahead or a buffer — nothing is precomputed and nothing is kept
between scans. It is the number of halvings of a binary search over **one scalar**, the jerk for this
scan: feasibility is monotone in it, so the feasible jerks form a single interval and the search finds
its upper end. Cost measured over 200k scans at -O2: **0.147 us** per `GenerateTrajectory` call at 15
steps, 0.165 us at 20 — 0.015% of a 1 ms cycle — and only **11** `BrakeDistance` calls per scan on
average, not one per step, because `acc_ok && vel_ok && pos_ok` short-circuits before the expensive
test in most iterations.

How many steps are actually needed, 200 moves at 1 ms (overshoot is 0 at every count):

| steps | backwards | distance left | over `max_vel` |
|---|---|---|---|
| 8 | 0.5103 mm/s | 0.00989 mm | 3.9e-05 |
| 12 | 0.0328 | 0.00170 | 4.9e-06 |
| **16** | **0.0289** | **0.00011** | **3.4e-13** |
| 20 | 0.0297 | 0.00007 | 6.5e-13 |
| 50 | 0.0295 | 0.00006 | 4.3e-13 |

It saturates at 16; below that the landing error and the limit excursions grow, above it nothing is
gained. 16-20 is the sweet spot.

**The search could not reach the limit (found 2026-09-13, fixed).** A bisection only ever tests
midpoints, so `jerk_lo` converged *towards* `+Jerk` without reaching it — at 15 steps the jerk output
read 4999.7 instead of 5000 in every saturated phase. One line fixes it: if no midpoint was ever
infeasible then the boundary lies above the limit, so the limit itself is feasible and is applied
exactly. With it, 15 steps now match what 50 used to give:

| 200 moves, 15 steps | before | after |
|---|---|---|
| scans reading exactly `\|jerk\| = Jerk` | 24.0% | **48.4%** |
| backwards velocity @1 ms | 0.1279 mm/s | **0.0295** |
| distance left @1 ms | 0.00679 mm | **0.00006** |
| over `max_vel` @1 ms | 4.4e-05 | **2.8e-13** |

**The jerk stays bang-bang, and the fractional scans are load-bearing.** Measured with a 1 mm/s^3
tolerance: ~48% of scans at exactly `±Jerk`, ~51% at zero, and **0.20%** carrying a genuinely
intermediate value — those are the switching scans, where the switch falls partway through the scan
and the fraction is how that is expressed. Snapping the jerk to `-Jerk / 0 / +Jerk` instead sends the
overshoot from 0.000000 to **3.156 mm** at 1 ms and the zero crossings from 0 to 3332. The jerk limit
itself is never exceeded either way (worst 5.6e-11).

Checked and not a problem: `vel_ok` is written one-sided (`<= max_vel`, no lower bound). Making it
two-sided changes nothing measurable, because the search always takes the *largest* feasible jerk and
so never chooses to accelerate away from the target. The 67.98 mm/s velocity excursion in the
bidirectional sweep is the documented start-state limit (`a0^2/(2*jerk)`, up to 100 mm/s at
`a0 = 1000`), not this.

**Moves from rest are now exact:**

| 200 moves | 2 ms | 1 ms | 0.2 ms | 0.1 ms |
|---|---|---|---|---|
| overshoot | 0.00000 | 0.00000 | 0.00000 | 0.00000 mm |
| zero crossings | 0 | 0 | 0 | 0 |
| backwards velocity | 0.117 | 0.030 | 0.001 | 0.000 mm/s |
| over `max_vel` | 8e-13 | 4e-13 | 0 | 1e-11 |

`max_acc` and `max_dec` never exceeded, jerk 5.7e-10, mirror symmetry exactly 0, and the rig's step
parks at 0.703 s with **no zero crossing at all**. The brake tree is unchanged by the extraction —
still 0.0000 mm against the integration.

**What is still open: the position test is direction-blind.** 189 of 20000 random `(target, v0, a0)`
states — the kind the handwheel produces — still ring. Traced:

```
   t        err   vel_dir   acc_dir |    jerk   feasible?   bd(end)   diff_end
 0.204  -5.54451   -5.272   +592.79 |   +5000        yes    0.01871    5.54948
 0.216  -5.56370   +2.157   +622.79 |   -5000         NO    7.19198    5.56123
```

While the axis runs *away* from the target, `BrakeDistance` reports the distance to standstill **in
the direction it is going** — 0.019 mm — so the test passes and the search keeps raising the
acceleration, to +622. The scan the velocity crosses zero, the axis is pointed at the target with
+622 on it, and a full stop from there needs 7.19 mm against the 5.56 that are left: the overshoot is
committed at the turnaround, one scan after the test last said "fine".

The quantity the test needs is not the one `BrakeDistance` returns. It needs the **signed displacement
to reach `(v = 0, a = 0)`**, not the distance until the velocity alone reaches zero. The two differ
exactly in the over-braking regime, where the acceleration survives standstill and picks the axis up
again — which is precisely the bucket where the tree and `Profilerold` disagreed when they were
compared (tree 0.0000, Profilerold 26.9493). So that disagreement was the two answering different
questions, and the feasibility test wants Profilerold's. (Profilerold is still wrong in the other
buckets.) Asked the user whether to add a second function or widen `BrakeDistance`; not done.

### The limits have to be ordered — guard and random button, 2026-09-13

`jerk >= max_acc, max_dec >= max_vel`. The other way round the profile degenerates: with the jerk
under the acceleration limits the acceleration ramp alone outlasts the move, with the acceleration
limits under the velocity one the velocity ramp does, and the axis never takes a proper step. Two
places now hold the rule:

- **`GenerateTrajectory`** orders them into its own locals (`max_acc = max(MaxAcceleration, max_vel)`
  and so on). The caller's `Axis` is **never written** — a parameter the user set stays set. That is
  deliberate: `motionold.cpp`'s documented bug was writing `MaxVelocity = 0` back into the caller and
  destroying it permanently. If `max_vel <= 0` the outputs are zeroed and the scan returns.
- **the rig's "Rastgele adım"** now randomises the four limits along with the target: `max_vel`
  10..500, `max_acc` and `max_dec` 1..10x that, `jerk` 1..10x the larger of the two. The four spin
  boxes are written with their signals blocked and `applyParams` is called once.

Verified with the limits randomised as well, 3000 moves per scan period, 300 s budget each:

| dt | overshoot | crossings | unsettled | distance left | over `max_vel` | over `max_acc` | over `max_dec` | over `Jerk` |
|---|---|---|---|---|---|---|---|---|
| 2 ms | 0.1033 mm | 27 | 0 | 0.0098 | 3.1e-3 | **0** | **0** | 2.2e-10 |
| 1 ms | **0.0000000** | **0** | 0 | 0.0087 | 1.0e-3 | **0** | **0** | 4.4e-10 |
| 0.2 ms | **0.0000000** | **0** | 0 | 0.0052 | 3.6e-5 | **0** | **0** | 2.2e-9 |

The single 2 ms overshoot is an extreme ratio — `max_dec` only 1.36x `max_vel` while `max_acc` is
7.3x — and it is gone by 1 ms. Earlier runs showed a handful of "unsettled" moves; those were only
the time budget (`max_vel` as low as 10 mm/s over a 400 mm target needs 40 s), not a failure.

### Feed override and software limits — added 2026-09-13

**`Axis.FeedOverride`** (default 1.0). It scales the **velocity limit only**: `vel_limit = max_vel *
max(FeedOverride, 0)`, used in the feasibility test's velocity check. The acceleration and jerk limits
are untouched, so a change in the override ramps in by itself under them — no special case, no
smoothing code. The limit ordering guard keeps using the machine `MaxVelocity`, not the scaled one.
Verified: peak velocity lands exactly on `200 * override` (2.8e-13 at 1.0, 1.1e-4 at 0.1), an override
of 0 holds the axis still, and stepping 1.00 -> 0.25 -> 1.00 in the middle of a move exceeds neither
the acceleration limits nor the jerk (both 0.0e+00).

**Software limits** are enforced by asking **where the axis would come to rest**, not where it is:
`stop_pos = cur_pos + dir * (pos_end +/- brake_end)` must be inside `[NegativeLimit, PositiveLimit]`.
The rule has a second half — *or at least closer to the band than the axis already is* — which is what
lets an axis that starts outside drive back in instead of being frozen. A first version gated the
whole test on `cur_pos` being inside; that version disabled itself the moment the axis breached by a
hair and turned a 0.19 mm breach into a **449 mm** one.

With the recovery rule, 2000 moves whose target is deliberately beyond the +/-100 mm limits, with the
dynamic limits randomised: worst breach **5.99 mm**, and **1995 of 2000** come to rest on the limit.
The residual breach is the same defect as the direction-blind position test above — `stop_pos` signs
`brake_end` by `vel_end`, which is meaningless when the velocity is near zero with an acceleration
still on it. Fixing that one quantity fixes both.

### Readiness for real motion blocks — assessment 2026-09-13

Asked whether motion function blocks could be built on this. Separating the engine from the layer:

**Trustworthy today.** The brake tree (0.0000 mm against a 2e-7 s integration over 2991 states, every
regime and both directions). The scan law for moves from standstill — with the limits randomised too,
3000 moves per scan period: zero overshoot and zero zero-crossings at 1 ms and below, `max_acc` and
`max_dec` never exceeded, jerk to 4e-10, the two directions bit-identical. Real-time behaviour is
sound: no allocation, no recursion, no unbounded loop in the scan path, the bisection is fixed-step so
the cost is deterministic at 0.15 us per scan, and nothing is kept between scans. The one
`solveQuadratic` call site now takes `a = 0.5 * jerk` with `jerk > 0` guaranteed by the limit guard, so
the missing `a == 0` check (open bug 5) is unreachable from here.

**Not ready.**

1. **The direction-blind position test** (written up above) — 189 of 20000 random `(v, a)` start states
   still ring. That state is exactly "the target moved while the axis was running", which is what
   `MC_MoveAbsolute` over `MC_MoveAbsolute`, blending and an intervening `MC_Stop` all produce. This is
   the blocker.
2. **Software limits are not consulted anywhere** — `NegativeLimit` / `PositiveLimit` appear only in
   `Axis.h`; grep finds no reference in the generator or the rig.
3. ~~**No block semantics.**~~ **Corrected by the user 2026-09-13: block semantics are deliberately
   out of scope.** This repo is the **generic core** a user builds their own blocks on — the state
   machine, `Done`/`Busy`/`CommandAborted`, buffer modes and `MC_Stop` priority belong to the caller.
   What the repo *does* owe is every motion feature the core needs: limits, feed override, software
   limits, and the kinematics.
4. ~~**No feed override.**~~ Added 2026-09-13, see above. A raw `MaxVelocity` change still applies on
   the next scan with no ramp on the command (the motion stays jerk limited, only the command steps).
5. **The verification is simulation only, and mostly against this file's own formulas.** The one
   independent reference is numerical integration. Nothing has run on a drive; scan jitter, a
   a drive that cannot follow the jerk command are unmodelled. **`deltaTime` stays a plain parameter
   by the user's decision (2026-09-13): calling the core deterministically is the integrator's
   responsibility, not the core's.**

**Remaining, in order:** (1) the signed `(v, a) -> (0, 0)` displacement — it fixes the direction-blind
position test *and* the residual software-limit breach, which are the same defect; (2) a test on real
hardware with real scan jitter. Software limits and feed override are done.

### Tried on the way and rejected

- **`vel_stoppable(d, a)` as the velocity command.** The inversion itself is right — verified against
  the tree over 270k states in every regime (`a > 0`, normal braking) to 8.6e-4 mm/s, the only gap
  being over-braking (`v < a^2/(2*jerk)`), where the axis stops before the acceleration is nulled and
  the profile has a different shape. But it **cannot be a regulation target**: it is a function of the
  very acceleration the law is choosing, so braking harder raises the allowed velocity, which asks for
  acceleration, which lowers it again. Measured: the axis parks 26-43 mm short of the target and
  500/500 moves never settle. It is a valid *test*, not a command.
- **`brake_distance >= diff_pos` as the guard** instead of `diff_pos_zero <= 0`. `brake_distance` is a
  magnitude with no direction, so it also fires while the axis is running *away* from the target and
  then forces more deceleration into the runaway: 11700 mm/s, 55 m of overshoot.
- Two weaker variants of the guard (`acc_target = min(acc_target, 0)`, and clamping `acc_exact` by the
  sign of `delta_vel`) changed nothing at all.

### Earlier options, prototyped and measured 2026-09-13

**Option A — one law, no `diff_pos > brake_distance` split.** Prototyped in full: a single velocity
command (the `a = 0` envelope asked at the `a = 0` point) feeding the `BuildRamp` peak-acceleration
law, `acc_exact`, the limit clamp and the jerk walk.

| | current | option A |
|---|---|---|
| worst backwards velocity @1 ms | 64.8 mm/s | **7.1** |
| strays off the surface (count) | 60 | **7** |
| overshoot @1 ms | 0.265 mm | **0.465** |
| overshoot @0.1 ms | 0.000 mm | **0.047** |

Backwards velocity now scales with the scan (11.8 / 7.1 / 2.3 / 1.46 at 2 / 1 / 0.2 / 0.1 ms) but the
overshoot doubles and does **not** collapse at a fine scan. Traced: once the envelope saturates at
`vel_command = 0` the distance is gone from the law, and `BuildRamp` — which only knows how to land a
*velocity* — reads `v = 63.96, a = -800` as "you are braking harder than needed, ease off", so the
jerk **releases** the brake from `-800` to `-455` while the axis is 0.03 mm from the target. A
velocity-regulating law cannot land on position. Two sub-variants (clamping `acc_exact` by the sign of
`delta_vel`, and refusing to release once `diff_pos_zero <= 0`) changed nothing.

**Option B — `vel_stoppable(d, a)`, recommended.** Both halves of the inverse are now derived and
verified against the tree in this file:

- trapezoidal, quadratic in `v`, with `A = |a|`, `D = max_dec`, `J = jerk`, `t1 = (D - A)/J`,
  `dv1 = A*t1 + 0.5*J*t1^2`, `dv3 = D^2/(2*J)`:
  `solveQuadratic(1/(2*D), t1 - dv1/D, -0.5*A*t1^2 - (1/6)*J*t1^3 + (dv1^2 - dv3^2)/(2*D) + D^3/(6*J^2) - d)`
  — agrees with the tree to **6.8e-7 mm**, inverts back to **8.4e-6 mm/s** over 10974 states.
- triangular, cubic in `u = acc_peak - A`: `u^3 + 2*A*u^2 + A^2*u + A^3/6 = d*J^2`, then
  `v = (2*(u+A)^2 - A^2) / (2*J)` — **6.8e-7 mm** / **6.8e-6 mm/s** over 20000 states. At `A = 0` it
  collapses to the `cbrt(d^2*jerk)` already in the file, so the present triangular envelope is its
  `a = 0` special case. Needs one real cubic root (Cardano; `std::cbrt` is already used here).

With `vel_command = min(max_vel, vel_stoppable(diff_pos, acc_dir))` there is no branch split left to
fight, and unlike option A the law carries the position, so the release-the-brake failure cannot
happen. **Not measured end to end** — only the two inversions are verified.

**Option C — hysteresis or a deadband on the branch test.** Cheapest, hides rather than fixes; the
measured analogue is raising `TOLERANCE` to 1e-1, which removes the ringing from the statistics
without changing a single ringing move.

**Superseded:** the earlier recommendation below is kept for the record but the `x_null` framing is
wrong for the braking side.

**Earlier (wrong) recommendation, 2026-09-13:** the `x_null` correction, not the other two.
The top level decision is already right — `brake_distance(v, a)` accounts for both. What is wrong is
that the braking branch regulates onto a *different* surface (`v == vel_allowed(diff_pos)`, the
`a = 0` envelope). Asking the envelope at the `a = 0` point instead —
`brake_velocity <= vel_allowed(diff_pos - x_null)` — is algebraically the same condition as
`brake_distance(v, a) <= diff_pos`, so the branch test and the clamp become the same surface: the two
branches stop fighting, the coasting sub-branch stops being dead code, and no hysteresis is needed.
Making `acc_exact` target `vel_threshold` instead (option 1) only makes the branch self-consistent on
a surface that is still too lenient by exactly the `x_null` term, trading the ringing for systematic
overshoot — the 1.7 mm per step already recorded in the Tracking notes. Hysteresis (option 3) hides
the gap between the two surfaces without closing it. `x_null = v*t_null + (1/3)*a*t_null^2` with
`t_null = |a|/jerk` is signed and needs no separate branch for speeding up or slowing down. It does
**not** replace the in-position window: the envelope's slope is still infinite at the target.

**Still open:** the `diff_pos <= TOLERANCE` park snaps the position onto the target and zeroes the
velocity, so if the axis happens to be fast on that scan it throws away up to 3.4 mm/s as a step.
The window branch in front makes it rare but it is still there.

## Working style notes

- The user writes the code themselves; do not change the logic unless asked, just do what
  was asked.
- When comments are requested, keep them **short**.
- Point out bugs, but do not fix them unprompted — ask first.

---
*Last update: 2026-09-13 (later the same day) — velocity mode and the position mode were merged into
**one `GenerateTrajectory` split by a single `if`**, with a small `MotionCommand` struct naming the
unit of `Target` at the call site; behaviour re-measured afterwards and unchanged. `docs/rig.mp4`
added: a 20 s offscreen recording of the rig (random commands, handwheel, random commands).*

*Previous update: 2026-09-13 (later the same day) — **velocity mode added**: `GenerateVelocityStep`, with
both modes sharing one `ScanStep` body so the open defect gets fixed once. 2000/2000 runs reach the
command exactly at every scan period, velocity limit held to 0.0e+00, software limits stop a velocity
command dead on the limit. Three real defects were found on the way — a clamp that could break the
jerk limit (pre-existing), a park that skipped the software limits, and a fallback pointing the wrong
way — all fixed and written up above, along with two that are still open. Position mode is unchanged.*

*Previous update: 2026-09-13 (later the same day) — `README.md` rewritten as the outward-facing
description (what it is, why scan-based, where it applies, the measured verification table, and an
explicit list of what is **not** verified), with `docs/rig.png` captured offscreen from the rig.*

*Previous update: 2026-09-13 (later the same day) — **the core was pulled out of the axis.** `Axis.h` is
gone; `GenerateTrajectory` is now a pure function over `MotionState` / `MotionLimits` returning a
`TrajectoryStep`, owning nothing and keeping nothing between calls. `FeedOverride` was added and then
removed — a feed override is a scaled `MaxVelocity` the caller hands in. Software limits stay in the
core (the caller cannot enforce them without re-deriving the brake logic); the rig got spin boxes for
them. `braketest` was deleted: it drove the core through `AxisState::Stopping` and waited for `Idle`,
neither of which had existed since the rewrite, so it had silently stopped testing anything. Behaviour
is unchanged by the refactor — same numbers through the new interface: zero overshoot and zero
crossings at 1 ms and below with the limits randomised, software limits 999/1000 resting on the limit.
Block semantics, buffering and blending are the caller's, by the user's decision.*

*Previous update: 2026-09-13 (later the same day) — the limit ordering rule (`jerk >= acc/dec >= vel`) is
enforced in two places: `GenerateTrajectory` orders them into locals without ever writing the caller's
`Axis`, and the rig's random button now randomises all four limits under that rule along with the
target. Re-verified with the limits randomised too — 3000 moves per scan period, zero overshoot and
zero crossings at 1 ms and below, acceleration limits never exceeded. Also fixed the bisection never
reaching `±Jerk` (it only converged towards it), which brought 15 steps up to the quality 50 used to
give.*

*Previous update: 2026-09-13 (later the same day) — **the end-of-scan feasibility search is applied**. The
brake tree is now the function `BrakeDistance(...)`, the motion section searches the jerk each scan,
and the old "never extract a formula" style rule is gone. Moves from rest are exact: zero overshoot,
zero crossings, limits held to 1e-10, the rig's step parks in 0.70 s without crossing the target once.
Still open: 189 of 20000 random `(v, a)` start states ring, because the position test uses
`BrakeDistance`, which is direction-blind — while the axis runs away from the target it reports the
distance to standstill the other way, so the search raises the acceleration until the turnaround
commits the overshoot. The test needs the signed displacement to `(v = 0, a = 0)` instead. Asked which
shape that should take.*

*Previous update: 2026-09-13 (later the same day) — **root cause of the overshoot in both phases found**:
every constraint is tested on the state at the start of the scan while the jerk chosen binds the state
at the end of it, so the switch is always one scan late and the excess (1.0 mm/s at the velocity limit,
the matching distance at the target) cannot be given back. A prototype that instead picks the largest
jerk whose end-of-scan state is still feasible removes the overshoot completely — 0.000000 mm at every
scan period, zero crossings, velocity limit held to 4e-13. Written up above with the two structural
changes it would need. No code changed.*

*Previous update: 2026-09-13 (later the same day) — the user suspected `brake_distance`; **checked and
ruled out**. The tree matches a 2e-7 s integration of the optimal brake to 0.0000 mm over 2991 random
states, while `Profilerold::BrakeDistance` is off by up to 24.16 mm on states whose velocity and
acceleration have opposite signs — so that file is the wrong one to copy from, and its own 200k
verification never covered `BrakeDistance`. `brake_distance` is also no longer in the control path at
all: its only remaining use is the in-position park test. No code touched.*

*Previous update: 2026-09-13 (later the same day) — the single overshoot left after the ringing fix was
traced and **fixed**: `x_null` assumed the jerk always finishes nulling the acceleration, but in the
over-braking regime (`v < a^2/(2*jerk)`) the axis stops first, so it was measuring a ramp that never
happens. `t_null` is now the time to standstill there, from the `solveQuadratic` the brake tree
already uses. Overshoot 0.246 -> 0.0168 mm at 1 ms, the rig's step down to a single zero crossing. Two
regressions recorded honestly above (worst backwards velocity, worst single move). The sliding mode
that produced the overshoot — `x_null` feeding back into its own command — is written up there too.*

*Previous update: 2026-09-13 (later the same day) — **the ringing fix is applied**. The
`diff_pos > brake_distance` branch split is gone; one law drives the motion (envelope at the `a = 0`
point -> `BuildRamp` peak acceleration -> `acc_exact` -> limits -> jerk walk), plus a guard that forces
full braking once the acceleration can no longer be nulled before the target. The rig's step goes from
12 zero crossings to 3 and parks in 0.83 s instead of 1.43 s; worst backwards velocity 64.8 -> 7.6 mm/s
at 1 ms. Fine-scan overshoot got worse (0.0009 -> 0.0225 mm at 0.2 ms) — the trade is written up above,
along with two approaches that were measured and rejected.*

*Previous update: 2026-09-13 (later the same day) — three fixes for the ringing prototyped and measured,
**none applied**. Removing the branch split (option A) cuts the worst backwards velocity from 64.8 to
7.1 mm/s but doubles the overshoot and it stops scaling with the scan, because a velocity-regulating
law releases the brake 0.03 mm from the target. The recommendation is option B, `vel_stoppable(d, a)`:
both halves of the inverse brake tree are now derived and verified to 6.8e-7 mm against the tree
(trapezoidal quadratic, triangular cubic). See the section above.*

*Previous update: 2026-09-13 (later the same day) — the user suspected `TOLERANCE`; **ruled out** by three
experiments (value swept over six orders of magnitude, the dimensional misuse removed, the value
raised until the ringing only hides). Two genuine but dormant `TOLERANCE` defects were recorded on the
way: everything except `diff_pos > TOLERANCE` compares a non-length against a 1e-6 that reads as mm,
and that same constant doubles as a position deadband far below one scan's resolution. No code
touched. The ringing diagnosis stands: the braking branch regulates onto the `a = 0` envelope while
the top level test uses the exact `brake_distance(v, a)` surface.*

*Previous update: 2026-09-13 (later the same day) — **tried the `x_null` fix and reverted it**: asking the
envelope at the `a = 0` point assumes the axis reaches `a = 0` before the target, which a hard brake
never does, so `diff_pos_zero` goes negative and the threshold tells the axis to release the brake.
Ringing halved but did not go. The quantity actually missing is `vel_stoppable(d, a)`, the inverse of
the brake tree at the *current* acceleration — derived, and verified against the tree to 6.8e-7 mm
over 10974 states. File is back to its pre-patch state.*

*Previous update: 2026-09-13 (later the same day) — investigated the ringing the user saw on the rig,
**no code touched**. It is the accelerating and braking branches swapping back and forth during the
approach: the braking branch regulates onto the zero-acceleration envelope, over-brakes, the margin
turns positive, the accelerating branch takes over and unwinds the deceleration at full jerk. 1998 of
2000 steps do it; worst backwards velocity on a forward move 64.8 mm/s. Root is the missing `x_null`
correction plus the decision and the clamp targeting two different velocities. Written up above.*

*Previous update: 2026-09-13 (later the same day) — **the online rig is ready to run**. `diff_pos` is
now a magnitude and the motion is solved in the direction of travel (`vel_dir` / `acc_dir`, `dir` put
back on the outputs), so one tree covers both directions — verified bit-identical over 5000 mirrored
pairs; the mirrored `else if` branch was written and then deleted. An in-position window branch was
added in front of the tree, without which the axis hunts around the target for ever, and the rig's
window default was raised to 1e-2 mm to match a 1 ms scan. `./build/onlinetest` builds; it has not
been run here (GUI).*

*Previous update: 2026-09-13 (later the same day) — the user's limit rule applied to both branches:
forward or backward, `max_acc` while the speed grows and `max_dec` while it shrinks, via
`acc_limit_up` / `acc_limit_down`. The accelerating side's floor became an explicit `cur_vel > max_vel`
branch so a deceleration limit no longer appears where it reads wrong. Sweep is clean on both limits
now. Traced the end of the move: the axis **hunts** around the target rather than settling, which is
the missing `InPositionWindow` — written up above, not added.*

*Previous update: 2026-09-13 (later the same day) — the **braking side** (`diff_pos <= brake_distance`)
was written on request: the remaining distance is inverted into `vel_allowed` and the rest of the
scan is the same target/clamp/walk/polynomial machinery as the accelerating side. Verified against
the real code; the landing error is `dt^2`. Two findings reported and left alone: a backwards start
uses `max_acc` where `max_dec` belongs, and the axis arrives with a few mm/s still on it because
`InPositionWindow` is not used yet. See the section above.*

*Previous update: 2026-09-13 (later the same day) — the accelerating side of the motion block was
**fixed on request**: the `max_acc` gate that froze the acceleration, the velocity latch and the
Euler integration are gone, and the acceleration now walks towards a clamped target so the jerk
limit holds. The user then caught the `-max_dec` target on the easing-off side — an accelerating
block never asks for a deceleration, the target there is `0`; that alone removed the parked-at-
`max_vel` chatter that had been written up as an open design problem. `max_dec` now only floors
the clamp, for a start state above `max_vel`. Both binaries build.*

*Previous update: 2026-09-13 (later the same day) — the hand written motion block at the bottom of
`TrajectoryGenerator.cpp` was reviewed twice on request, no code touched. The key finding is that
`cur_vel < brake_velocity` cannot decide anything (it reduces to `sign(cur_acc)`); the full list
is in the new "Motion block at the bottom" section above.*

*Previous update: 2026-09-13 (later the same day) — bug 11 fixed on request: the two unqualified
`abs(...)` calls at lines 270/273 are `std::abs` now, builds clean. The faded `lcpp_x:` text that
led there was a clangd **inlay hint**, not code (libc++ names `abs`'s parameter `__lcpp_x`,
`SDKs/MacOSX.sdk/usr/include/c++/v1/stdlib.h:121`); inlay hints turned off in the user's VS Code
settings. Note for next time: the clangd extension (0.6.0) has **no** `clangd.inlayHints.enabled`
setting — only the `clangd.inlayHints.toggle` command — so that key is a no-op. What works is
VS Code's own `editor.inlayHints.enabled: "off"`, set under the `[cpp]` / `[c]` language overrides.*

*Previous update: 2026-09-13 (later the same day) — Qt6 installed on the user's macOS machine
(`brew install qt`, 6.11.2 with QtCharts) and the build reproduced there; the Build section now
carries the macOS variant and its `CMAKE_PREFIX_PATH`. Both `onlinetest` and `braketest` compile
and link on Apple clang — the unfinished `if (diff_pos > TOLERANCE)` block at the bottom of
`TrajectoryGenerator.cpp` is syntactically valid, so it no longer blocks the build (the note below
saying it does not compile is out of date on that point; the logic itself is still unfinished).
No code was touched.*

*Previous update: 2026-09-13 — **new architecture attempt**. The user started rewriting the online
decision tree; `diff_pos` and `brake_velocity` added as top level locals and `brake_velocity`'s
first draft fixed on request (const assignment, `Jerk` typo, wrong branch, dimensionally wrong
formula). See the section above. The motion section at the bottom of the file is unfinished.*

*Previous update: 2026-09-06 (later the same day) — **the online generator**.
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
