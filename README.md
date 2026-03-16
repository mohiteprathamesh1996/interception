# Missile Intercept Lab

An R Shiny application simulating a missile-interceptor engagement in three
dimensions.  The simulation implements realistic physics and guidance algorithms
suitable for educational and research purposes.

---

## Contents

```
missile_app/
  app.R           Main Shiny application (UI + server)
  R/
    physics.R     Simulation engine (pure R, no Shiny dependency)
    plots.R       Plotly visualisation functions
  www/
    style.css     Application stylesheet
  README.md       This file
```

---

## Requirements

R (>= 4.0) with the following packages:

```r
install.packages(c("shiny", "plotly", "shinycssloaders", "shinyjs"))
```

---

## Running the Application

```r
shiny::runApp("missile_app/")
```

Or open `app.R` in RStudio and click **Run App**.

---

## Physics Overview

### Numerical Integration

All state variables are integrated using the classical fourth-order Runge-Kutta
method (RK4).  This gives a global truncation error of O(dt^4), compared with
O(dt) for the Euler method used in most introductory implementations.  The
default time step is dt = 0.02 s, which is conservative and accurate even for
fast-turning engagements.

### Guidance Laws

Four guidance laws are implemented:

| Law | Formula | Notes |
|-----|---------|-------|
| Pure PN | a = N * Vc * (omega_LOS x r_hat) | Standard proportional navigation |
| Augmented PN | a = N*Vc*(omega_LOS x r_hat) + (N/2)*a_tgt_perp | Adds target acceleration feedforward |
| Optimal (OGL) | a = 6 * ZEM / t_go^2 | Minimises miss distance (LQR solution) |
| Time-Varying PN | N_eff = N*(1 + 1/t_go) | Boosts gain as engagement closes |

Where:
- `r_hat` is the unit line-of-sight (LOS) vector from interceptor to target
- `Vc` is the closing speed: `Vc = -r_hat . v_rel`
- `omega_LOS` is the LOS angular rate: `omega_LOS = (r_hat x v_rel) / |r|`
- `ZEM` is the zero-effort miss vector: `ZEM = r_rel + v_rel * t_go`
- `t_go` is estimated time to go: `t_go = |r| / Vc`

### Kalman Filter

A 6-state linear Kalman filter tracks the target's position and velocity.  The
guidance laws use this filtered estimate rather than the raw (noisy) radar
measurement.

- State vector: `x = [px, py, pz, vx, vy, vz]`
- Process model: constant velocity with discrete Wiener process noise
- Measurement model: position only, or position + velocity (Doppler mode)
- The process noise parameter `sigma_q` (m/s^2) represents the expected target
  manoeuvre acceleration; increase it for agile targets

### Actuator Dynamics

Guidance commands are passed through a first-order lag:

```
tau * a_dot + a_actual = a_cmd
```

Discretised as:

```
a(k+1) = a(k) + (dt / tau) * (a_cmd - a(k))
```

Setting `tau = 0` gives ideal (instantaneous) response.  A realistic fin
actuator has `tau` in the range 0.05 to 0.25 s.

### Atmosphere and Drag

The ISA (International Standard Atmosphere) model is used for the troposphere
(0 to 11 km altitude):

```
T(z)   = 288.15 - 0.0065 * z   (K)
rho(z) = 1.225 * (T / 288.15)^4.256   (kg/m^3)
```

Aerodynamic drag is modelled using the ballistic coefficient beta = m / (Cd * A)
in kg/m^2.  The deceleration is:

```
a_drag = -rho(z) * v^2 / (2 * beta) * v_hat
```

Typical values: cruise missile 2000-5000 kg/m^2, interceptor 1500-4000 kg/m^2.
Drag is disabled by default; enable it in Physics Settings.

### Gravity

Gravity acts as a constant downward acceleration in the ENU (East-North-Up)
frame: `g = [0, 0, -9.81]` m/s^2.  Both bodies experience gravity.  Powered
flight models (enabled by default) apply forward thrust to counteract drag and
gravity along the velocity vector.

### Radar Noise

Radar measurements use a first-order autoregressive AR(1) noise model:

```
n(k) = rho * n(k-1) + sigma * sqrt(1 - rho^2) * w,   w ~ N(0,1)
rho  = exp(-dt / tau_c)
```

This generates temporally correlated noise that is more realistic than
independent Gaussian samples.  The effective noise standard deviation also
scales with range:

```
sigma_eff(R) = sigma_0 * (1 + k_R * R / 10000)
```

### Target Manoeuvres

| Profile | Description |
|---------|-------------|
| None | Straight and level flight |
| Sinusoidal weave | Lateral oscillation at configurable amplitude and frequency |
| Bang-bang | Hard lateral reversal at maximum g every half period |
| 3D spiral | Simultaneous sinusoidal manoeuvre in two perpendicular planes |
| Stochastic | Random lateral direction each time step |

---

## Controls

### Physics Settings
- **Gravity** - toggles gravitational acceleration (ENU frame, +Z up)
- **Aerodynamic drag** - toggles ISA atmosphere + ballistic coefficient drag model
- **Ballistic coefficients** - higher value means less drag (heavier / more streamlined)

### Guidance Law
- **Law selector** - choose from PN, APN, OGL, or TVPN
- **Navigation constant N** - proportional navigation gain; higher value gives more
  aggressive steering but can cause instability; 3-5 is typical
- **Actuator lag tau** - first-order lag on the guidance command in seconds

### Target
- **Speed** - target speed in m/s (direction is set by the Vx/Vy inputs)
- **Evasive manoeuvre** - manoeuvre profile and parameters
- **Initial position** - X, Y, Z coordinates of target at t = 0
- **Initial velocity direction** - Vx, Vy components (Vz defaults to 0)

### Interceptor
- **Max speed** - speed cap on interceptor in m/s
- **Max lateral acceleration** - structural g-limit
- **Max thrust** - forward thrust available to maintain speed
- **Initial position** - launch site coordinates

### Radar / Sensor
- **Position noise sigma** - 1-sigma Gaussian noise on position measurements (m)
- **Noise correlation time** - AR(1) time constant; 0 = white noise
- **Range-dependent noise scale** - how quickly noise grows with range
- **Doppler measurement** - also observe target velocity (noisily)

### Kalman Filter
- **Process noise sigma_q** - expected target manoeuvre acceleration (m/s^2);
  increase for agile targets, decrease for smooth trajectories
- **Initial velocity uncertainty** - prior uncertainty on target velocity

### Animation
- **Animation frame** - scrub through the engagement step by step
- **Auto-play** - step through at approximately 18 frames per second
- **Display toggles** - show/hide radar track, KF estimate, LOS vector,
  predicted intercept point

---

## Tabs

| Tab | Content |
|-----|---------|
| 3D Engagement | Animated 3D view with true trajectories, KF estimate, noisy radar track, LOS vector, predicted and actual intercept points |
| Guidance Metrics | Closing range + ZEM, commanded vs actuator acceleration, LOS rate + time-to-go, speed vs time |
| Kinematics | Altitude vs time, Mach number, aerodynamic drag deceleration |
| Kalman Filter | Estimation error vs 1-sigma uncertainty, innovation (measurement residual), filter consistency check |
| Monte Carlo | Overlaid interceptor trajectories across all runs, miss distance histogram, intercept probability |
| Physics Reference | Equations and variable definitions for all models used |

---

## Colour Coding

| Colour | Element |
|--------|---------|
| Red | Target trajectory |
| Blue | Interceptor trajectory |
| Purple (dashed) | Kalman filter estimate |
| Green | Intercept point, kill radius, positive status |
| Amber | LOS vector, closing range, ZEM |
| Faint red dots | Noisy radar track |

---

## Extending the Simulation

The physics engine (`R/physics.R`) has no Shiny dependency and can be called
independently for batch simulation or scripted analysis.

Key functions:

```r
# Single run
result <- run_simulation(params)

# Monte Carlo
results <- run_monte_carlo(params, n_runs = 50)

# Default parameters as a starting point
p <- default_params()
p$v_target <- 600
p$guidance_law <- "ogl"
result <- run_simulation(p)
```

Possible extensions:

- Replace Euler speed cap with proper 6-DOF thrust integration
- Add roll dynamics and bank-to-turn steering
- Implement an extended Kalman filter for nonlinear target models (e.g. coordinated turn)
- Add a seeker field-of-view constraint (the guidance command should be gated by
  whether the target is inside the seeker gimbal limit)
- Model radar clutter or electronic countermeasures as additional noise sources
- Add a second interceptor for salvo engagement analysis

---

## Known Limitations

- Euler speed cap: when propulsion is on, speed is clamped rather than integrated
  through a proper thrust-drag energy equation; this is a minor approximation
- Constant-velocity Kalman process model will lag during hard target manoeuvres;
  an interacting multiple model (IMM) filter would handle manoeuvre detection better
- No gravity gradient above 11 km; ISA stratosphere model is included but not
  validated against real range test data
- The target manoeuvre is applied as a pure lateral force with no aerodynamic
  coupling to the drag model
