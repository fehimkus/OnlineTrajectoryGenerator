# OnlineTrajectoryGenerator — Claude working notes

## Rules for this file

- **Read this file at the start of every session.** Look here before reading any code.
- **Keep it about purpose and method, not history.** Record what the core is, how it works and why it
  is built that way. Do not turn it back into a changelog — dated post-mortems, superseded
  recommendations and measurement dumps do not belong here.
- **Update it at the end of every prompt.** New decisions, new formulas, changes of approach go into
  the matching section. If there is nothing to update, say "no update" rather than skipping silently.
- Everything in this repo is written in **English**: this file, code comments, docs, README.

## What this repo is

**A trajectory core, not a motion controller.** It answers one question, once per scan: given where
the axis is, what it may not exceed, and what it is told to do, what is the state one scan later.

It owns nothing. There is no axis object, nothing is kept between calls, nothing is written back. The
caller holds the axis and calls this every cycle.

**Out of scope, deliberately** — these belong to the motion layer written on top: the state machine,
`Done` / `Busy` / `Active` / `CommandAborted` / `Error`, buffer modes, blending, `MC_Stop` priority.
Blending needs nothing extra from the core: handing in a fresh command on any scan is what it is built
for. A feed override is just a scaled `MaxVelocity` handed in.

**`deltaTime` is a plain parameter.** Calling the core on a deterministic cycle is the integrator's
responsibility, not the core's.

## Interface

```cpp
struct MotionState    { double Position, Velocity, Acceleration; };
struct MotionLimits   { double MaxVelocity, MaxAcceleration, MaxDeceleration, Jerk,
                               NegativeLimit, PositiveLimit, InPositionWindow, InVelocityWindow; };
enum class MotionCommandKind { Position, Velocity };
struct MotionCommand  { MotionCommandKind Type; double Target; };   // mm, or mm/s
struct TrajectoryStep { MotionState State; double Jerk, BrakeDistance; bool InPosition, InVelocity; };

TrajectoryStep GenerateTrajectory(const MotionState&, const MotionLimits&,
                                  const MotionCommand&, double deltaTime);
double BrakeDistance(double cur_vel, double cur_acc, double max_dec, double jerk);
```

The command kind is a named enum rather than a bare `double` so the **unit of `Target` is written at
the call site**. Both kinds run through one function split by a single `if`; everything after the
split is shared.

`MotionLimits` holds whatever constraints are in force for this scan. A motion block folds the
*command's* dynamics and the *axis's* limits into these fields — the core does not care which is
which. (A rename to `MotionConstraints` was proposed for that reason and is still undecided.)

## How a scan works

1. **Order the limits.** `jerk >= max_acc, max_dec >= max_vel`, into local copies. The other way round
   the profile degenerates and the axis never takes a proper step. **Never write the ordered values
   back** — the caller's parameters stay as it set them.
2. **Brake distance** of the state the scan was entered with, handed out before anything moves.
3. **Park check.** In position (or at velocity) → zero the motion and return. Without a window the
   axis can never settle: the envelope's slope runs away at the target, so a finite scan only ever
   hunts around it. A park branch returns early, so **anything it skips is not checked at all** — it
   must not bypass the software limits.
4. **Search the jerk.** A bisection over `[-Jerk, +Jerk]` for the largest value whose **end-of-scan**
   state still satisfies every constraint:
   - `-acc_limit_down <= a' <= acc_limit_up`
   - `v' + a'|a'| / (2*jerk) <= vel_bound` — where the velocity settles once that acceleration is nulled
   - position commands: `BrakeDistance(v', a') <= d'` — can still stop on the target
   - `stop_pos` inside the software limits, or at least closer to them than the axis already is
5. **Fallback** when nothing is feasible: relieve the constraint that is actually broken. If the
   settling velocity is already past its bound, pull that down; otherwise brake the motion the axis
   has. Getting this backwards diverges instead of recovering.
6. **Clamp and integrate.** Clamp the acceleration to the limits *and* to `+/- jerk*deltaTime` of
   where it was, so a state handed in past a limit is walked back rather than jumped. Then integrate
   `x/v/a` with the full polynomials using the jerk actually applied.

### Why end-of-scan, and why a search

Every limit is a statement about the state the axis will be in at the **end** of the scan. Tested on
the state it is in now, every switching point lands one cycle late, and what one late cycle costs —
`max_acc * deltaTime` of velocity at the limit, the matching ground at the target — a jerk-limited
axis can never give back.

The search is **not** a lookahead or a buffer, and nothing is precomputed. The jerk is a constant
`±Jerk`; what is unknown is *when inside the scan* it should switch, and that instant almost never
lands on a scan boundary. Applying `+Jerk` for 0.4 of a scan and `0` for the rest raises the
acceleration by the same amount as applying `2000` for the whole scan — so an intermediate value is
how "the switch fell 40 % into this scan" is written down, not a new jerk level. The bisection is a
stopwatch, not a control law. Feasibility is monotone in the jerk, so the feasible set is one interval
and the search finds its upper end. ~48 % of scans come out at exactly `±Jerk`, ~51 % at zero, and
only ~0.2 % carry a fractional value — the switching scans. Snapping those to bang-bang brings the
overshoot back, so they are load-bearing.

`BISECTION_STEPS` is the number of halvings. It saturates around 16; the cost is ~0.15 µs per scan
(0.015 % of a 1 ms cycle) with no allocation, no recursion and no unbounded loop, so the cost is the
same every cycle. Because a bisection only tests midpoints it can never *reach* `+Jerk`; when no
midpoint failed, the limit itself is feasible and is applied exactly.

### Direction frame

The scan is solved in one direction and the sign is put back on the way out. A position command picks
the frame from the position error, a velocity command from the **velocity** error, and passes "no
target" so only the software limits bound the travel — in velocity mode they are the only thing that
ever stops the axis. Which acceleration limit applies follows the **speed**, not the sign of the
acceleration: `max_acc` while the speed grows, `max_dec` while it shrinks, in both directions.

## Brake distance

An if/else tree over the state, on magnitudes, returning a positive distance. Branches:
`cur_acc * cur_vel > 0` (speeding up: null the acceleration first, then brake from there), `< 0`
(slowing down: raise or lower the deceleration to `max_dec`, hold, release), and the cases where
either is zero. Each branch splits trapezoidal / triangular on whether `max_dec` is reached.

Phase naming: 1 = jerk zeroes the current acceleration, 2 = jerk takes it to `-max_dec`, 3 = constant
`-max_dec`, 4 = jerk brings it back to zero with the velocity landing on zero.

Useful closed forms: the `a = 0` trapezoid collapses to `v^2/(2*max_dec) + v*max_dec/(2*jerk)`; the
triangular case to `v^1.5 / sqrt(jerk)`; the split sits at `max_dec^3 / jerk^2`. Triangular peak
deceleration is `sqrt(jerk*v + 0.5*a^2)`, and a final jerk phase always collapses to `a*t^2/6`.

**It is exact** — 0.0000 mm against a 2e-7 s integration of the optimal brake over 2991 random states
in every regime and both directions. Treat it as the reference, not as something to re-derive.

**But it carries no direction**, and that is the one known defect (below).

## How to verify a change

Claims here are measured, never estimated. Several plausible-sounding fixes were adopted and then
backed out because measurement contradicted them — assume the same will happen again.

- **Reference**: integrate the optimal brake numerically at ~1e-7 s and compare. That is the only
  independent check in the project.
- **Sweeps**: a few thousand moves with `MaxVelocity`, `MaxAcceleration`, `MaxDeceleration` and `Jerk`
  all randomised (keeping them ordered), at several scan periods. Watch overshoot, target crossings,
  each limit, and whether every move settles.
- **Scan scaling**: a residual that falls with `deltaTime` is granularity; one that does not is a
  formula error. This distinction has settled most arguments here.
- **Mirror check**: run a move and its exact negative; `|forward + backward|` must be 0.
- **Exclude what physics forbids**: a start state whose own acceleration already carries the velocity
  past `MaxVelocity` cannot be honoured — nulling it costs `a0^2/(2*jerk)` and nothing prevents that.
  Filter those out or the numbers are meaningless.

Current headline: zero overshoot and zero target crossings at 1 ms and below with the limits
randomised; velocity commands land exactly (2000/2000); acceleration limits never exceeded; jerk
within 1e-9.

## Open

- **The signed stop displacement — the one real defect.** `BrakeDistance` answers "how far do I
  travel, in the direction I am going, before the velocity reaches zero". The feasibility test needs
  "where do I come to rest, signed, on the target's axis, with **both** `v = 0` and `a = 0`". The two
  differ when the axis runs *away* from the target (the brake distance then measures the wrong
  direction, so the test passes right up to the turnaround) and in the over-braking case (the velocity
  reaches zero with an acceleration still on it, which picks the axis up again). It costs ringing on
  ~1 % of random start states and the residual software-limit breach. One quantity fixes both; it is
  the next piece of work.
- **Velocity mode at a zero crossing**: an acceleration that was legally `max_dec` while slowing a
  backward motion becomes an *acceleration* the moment the velocity crosses zero, and is briefly over
  `max_acc` if that is smaller. Walked back at the jerk limit — dropping it instantly would break
  jerk, so the excursion is bounded by `max_dec - max_acc` and unavoidable.
- **No hardware.** Everything is simulation; scan jitter and a drive that cannot follow the jerk
  command are unmodelled.
- **No rotary / modulo axes.** Position is linear; the shortest-path decision and an unwrapped target
  are the caller's job.
- **Non-zero target velocity** is not supported — the core always arrives at rest. This is what would
  let it guarantee a corner velocity instead of leaving blending to the caller.

## Multi-axis, if it comes up

A **coordinated straight-line** move reduces to this core exactly: run it on the path parameter `s`,
project with `x_i = A_i + (s/L)*(B_i - A_i)`, `v_i = s_dot * u_i`, and map the limits down with
`min_i(MaxVelocity_i / |u_i|)` and the same for acceleration and jerk. Every axis is a scaled copy of
one profile, so **time synchronisation is free** — there is only one time base and no prescribed
duration to hit. That part is pure wrapper work.

It breaks on: a start velocity not parallel to the new direction (online, the direction turns every
scan, and the perpendicular component cannot be expressed by one scalar — this is the serious one);
independent per-axis targets, which have no common direction to project onto; and curved paths, where
a `kappa * s_dot^2` term makes the limit mapping state dependent.

## Build

```bash
cmake -B build -S .                        # macOS: add -DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt
cmake --build build -j
./build/onlinetest                         # the only binary: the test rig
```

The core is `src/TrajectoryGenerator.h/.cpp` and depends on nothing but the standard library. Qt6
(Widgets + Charts) is only for the rig, `src/OnlineWindow.cpp/.h` + `src/online_main.cpp`: a handwheel
slider sampled *inside* the scan loop, four charts against their limits, an adjustable scan period, and
a random button that randomises the target and all four dynamic limits at once.

`docs/rig.png` and `docs/rig.mp4` were captured **without touching the user's screen**, by running the
binary under `QT_QPA_PLATFORM=offscreen` with a temporary flag in `online_main.cpp` that clicks the
random button, drives the slider along a sine, calls `QWidget::grab()` per frame and quits; the frames
were encoded with a throwaway Swift program using `AVAssetWriter` (no ffmpeg, imageio or Pillow on this
machine, and Qt cannot write GIF). Capture runs slower than the timer asks, so encode at the measured
frame rate or the video plays fast. **Never use `screencapture` on the user's desktop.**

## Code style

- Comments are **English, lowercase, short**, at the end of the line with `//`. One line is enough —
  no derivation blocks.
- Allman braces, 4 space indent.
- Formulas written out **fully parenthesized and unsimplified**, inline where they are used. Pulling a
  reusable *quantity* into its own function is fine when something else has to evaluate it.
- Literal constants like `0.1666667` rather than `1.0/6.0`.
- Cases laid out as a decision tree of `if / else if / else`, each branch commented with the condition
  it handles.
- Intermediate results go into clearly named locals.

## Working style

- **The user writes the code.** Do not change logic unless asked; do what was asked.
- **Point out bugs, do not fix them unprompted** — ask first.
- Measure before claiming. Report what the measurement says, including when it contradicts the
  suggestion that was just made.

## Context

- **Prior art**: jerk-limited online trajectory generation is a worked field. **Ruckig** (open source,
  multi-axis, time-synchronised, arbitrary target states) and its predecessor **Reflexxes** are the
  reference points; every large CNC and robot vendor has an in-house equivalent. This core is the
  single-axis, zero-target-velocity case — an easier problem. The value here is ownership,
  auditability and the verification trail, not novelty.
- **Licence**: the repo is **GPL-3.0**, which is copyleft — anyone distributing a product built on it
  must release that product under GPL-3.0 too. Wording like "free for anyone to build on" reads as
  permissive and is misleading under it. If the intent is genuinely "use it anywhere, commercial
  included", the licence has to change (MIT / Apache-2.0, what Ruckig uses). If it changes, the
  README's framing changes with it.
