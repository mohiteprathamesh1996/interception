# Missile Intercept Lab

An interactive R Shiny application that simulates a missile interceptor
pursuing a manoeuvring target in three dimensions.

The app is designed for students and researchers who want to explore how
guidance algorithms work — you can change the physics, tune the sensors,
and watch the engagement play out in real time.

---

## Quick Start

```r
install.packages(c("shiny", "plotly", "shinycssloaders", "shinyjs"))
shiny::runApp("missile_app/")
```

---

## What the App Shows

When you open the app, a simulation runs immediately with default settings.
The main view is a 3D plot showing three paths:

- **Red line** — the target, flying and evading
- **Blue line** — the interceptor, guided toward the target
- **Purple dashed line** — what the Kalman filter *thinks* the target is doing,
  based on noisy radar data

Above the plot, nine KPI cards update as you scrub through the engagement:

| Card | What it means |
|------|--------------|
| Interceptor speed | How fast the missile is moving right now |
| Target speed | How fast the target is moving right now |
| Closing range | Distance between the two objects |
| Accel (actual) | How hard the interceptor is currently turning, in g |
| Interceptor Mach | Speed as a multiple of the speed of sound |
| Zero-effort miss | If guidance stopped now, how far would it miss by |
| KF error | How wrong the Kalman filter estimate currently is |
| Min miss distance | Closest the interceptor has come to the target |
| Intercept time | When (if) the interceptor reached the target |

---

## Controls

### Physics Settings

**Gravity** — when on, both objects fall under 9.81 m/s. Without this,
everything flies in straight lines regardless of altitude.

**Aerodynamic drag** — when on, the air slows both objects down based on
altitude and their shape (set by the ballistic coefficient sliders).
Higher ballistic coefficient = sleeker, less drag.

### Guidance Law

Four algorithms are available. They differ in how they calculate which
direction to steer:

- **Pure PN** — the standard approach. Steer to cancel the rotation of the
  line between you and the target. Works well for non-manoeuvring targets.
- **Augmented PN** — same as Pure PN, but also compensates for the target's
  current acceleration. Better against targets that are actively turning.
- **Optimal (OGL)** — computes the minimum-energy path to intercept.
  Best theoretical performance, especially at long range.
- **Time-Varying PN** — like Pure PN, but the gain increases automatically
  as the interceptor closes in. Helps in the final seconds of the engagement.

**Navigation constant N** — how aggressively the interceptor steers. Higher
values mean sharper turns but can overshoot. Three to five is typical.

**Actuator lag (tau)** — real fins and control surfaces take time to move.
This adds that delay. Set to zero for an ideal instantaneous response;
increase it to see how sluggish actuation degrades accuracy.

### Target

**Speed** — target speed in metres per second.

**Evasive manoeuvre** — how the target tries to avoid the interceptor:

- **None** — flies straight
- **Sinusoidal weave** — oscillates sideways at a set frequency
- **Bang-bang** — snaps hard left, then hard right, alternating
- **3D spiral** — weaves in two planes simultaneously
- **Stochastic** — random direction changes each time step

**Initial position and velocity** — where the target starts and which
direction it is heading.

### Interceptor

**Max speed** — the interceptor's speed cap. Must be faster than the target
to have a chance.

**Max lateral acceleration** — the structural g-limit. Higher values allow
tighter turns but are physically demanding.

**Max thrust** — forward thrust to compensate for drag and gravity.

**Initial position** — where the interceptor launches from.

### Radar / Sensor

**Position noise** — how inaccurate the radar is when measuring where the
target is. Larger values make tracking harder.

**Noise correlation time** — real radar errors are not random every
millisecond; they drift slowly. This controls how quickly the noise changes.
Zero means fully random; higher values mean the error persists longer.

**Range-dependent noise scale** — accuracy gets worse at longer range.
This controls how quickly.

**Doppler measurement** — if on, the radar also measures the target's
velocity directly (with some noise), which helps the Kalman filter converge
faster.

### Kalman Filter

The Kalman filter is the "brain" that turns noisy radar data into a clean
estimate of where the target is and where it is going.

**Process noise (sigma_q)** — how much the filter expects the target to
manoeuvre. Too low, and the filter ignores sudden turns. Too high, and it
becomes jumpy. Match this roughly to the target's actual manoeuvre amplitude.

**Initial velocity uncertainty** — how uncertain the filter is about the
target's speed at the start. Lower values mean it trusts its initial guess
more.

### Monte Carlo

Runs the same simulation many times with different random noise seeds.
This shows the spread of possible outcomes — some runs will intercept,
some will miss — and gives a realistic picture of system reliability.

### Animation

The **animation frame** slider lets you scrub through the engagement step
by step. **Auto-play** steps through automatically.

The toggles show or hide: the noisy radar track, the Kalman filter estimated
path, the line-of-sight vector, and the predicted intercept point.

---

## Tabs

| Tab | What is in it |
|-----|--------------|
| 3D Engagement | The main 3D view with all trajectories |
| Guidance Metrics | Closing range, ZEM, acceleration, LOS rate, speed — all vs time |
| Kinematics | Altitude and Mach number over time; drag deceleration if enabled |
| Kalman Filter | How accurate the filter estimate is, and the measurement residuals |
| Monte Carlo | Overlaid trajectories from all runs; miss distance histogram |
| Physics Reference | The equations behind every model in plain text |

---

## File Structure

```
missile_app/
  app.R           Shiny UI and server logic
  R/
    physics.R     Simulation engine — runs independently of Shiny
    plots.R       All Plotly chart functions
  www/
    style.css     Stylesheet
  README.md       This file
```

The physics engine has no Shiny dependency. You can call it directly
in scripts for batch runs or parameter sweeps:

```r
source("R/physics.R")

p              <- default_params()
p$v_target     <- 600
p$guidance_law <- "ogl"

result <- run_simulation(p)

# result$closing      — range at each time step
# result$int_pos      — interceptor position matrix (n x 3)
# result$tgt_pos      — target position matrix (n x 3)
# result$miss_distance — closest approach
```

---

## Tips for Exploration

**Why is the interceptor missing?**
Check the closing range plot. If range is not decreasing, the interceptor is
too slow or the initial heading is wrong. Try increasing interceptor speed
or switching to OGL guidance.

**Why is the Kalman filter lagging behind the target?**
The process noise (sigma_q) is probably set too low for the target's
manoeuvre level. Increase it to make the filter more responsive.

**How does actuator lag affect performance?**
Compare the Commanded vs Actuator output lines on the Guidance Metrics tab.
At high tau, there is a clear gap — the interceptor cannot respond quickly
enough, which matters most in the final seconds.

**What is a good N value?**
For a non-manoeuvring target, N = 3 is standard. For a bang-bang target,
try N = 4 or 5, or switch to Augmented PN which accounts for the target
acceleration directly.

---

## Known Limitations

- The target manoeuvre is a pure lateral force and is not coupled to the
  drag model — in reality, hard turns increase aerodynamic drag significantly.
- The Kalman filter uses a constant-velocity process model. It will lag
  noticeably during a bang-bang manoeuvre. An interacting multiple model (IMM)
  filter would handle this better.
- Integration is RK4 at a fixed time step. Variable-step integration would
  be more efficient for engagements with very different timescales.
- No seeker field-of-view limit — the interceptor can always see the target
  regardless of angle.