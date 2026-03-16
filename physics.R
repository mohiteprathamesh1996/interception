# =============================================================================
# physics_v2.R  —  High-Fidelity Missile Interceptor Simulation Engine
#
# Improvements over v1:
#   1. RK4 integration (vs Euler)
#   2. ISA atmosphere + aerodynamic drag (ballistic coefficient model)
#   3. Full 3D gravity (ENU frame, Z = up)
#   4. Actuator dynamics: first-order lag on guidance commands
#   5. 6-state Extended Kalman Filter (position + velocity, CV + Singer noise)
#   6. Correlated AR(1) radar noise (vs white noise)
#   7. Range-dependent radar accuracy (σ ∝ R)
#   8. Optional Doppler velocity measurement
#   9. 4 guidance laws: Pure PN, Augmented PN, Optimal (OGL/ZEM), Time-Varying PN
#  10. 4 target maneuver models: None, Sinusoidal, Bang-Bang, Spiral
#  11. Speed-maintenance propulsion (powered missile model)
#  12. Predicted intercept point (collision triangle)
#  13. Zero-Effort Miss (ZEM) vector throughout engagement
#  14. Mach number and drag diagnostics per time step
# =============================================================================

# ── Physical constants ─────────────────────────────────────────────────────
G_ACCEL  <- 9.81              # m/s²
G_VEC    <- c(0, 0, -G_ACCEL) # ENU coordinates (Z = up)
R_GAS    <- 287.05            # J/(kg·K), dry air
GAMMA    <- 1.4               # adiabatic index, air

# =============================================================================
# ATMOSPHERE  —  International Standard Atmosphere (ISA)
# Valid: sea level to 20 km
# =============================================================================
isa_density <- function(z_m) {
  z_m <- max(0, z_m)
  if (z_m <= 11000) {
    T   <- 288.15 - 0.0065 * z_m
    p   <- 101325 * (T / 288.15)^5.2561
  } else {
    T   <- 216.65
    p   <- 22632 * exp(-0.0001577 * (z_m - 11000))
  }
  p / (R_GAS * T)
}

isa_temperature <- function(z_m) {
  z_m <- max(0, z_m)
  if (z_m <= 11000) 288.15 - 0.0065 * z_m else 216.65
}

speed_of_sound <- function(z_m) {
  sqrt(GAMMA * R_GAS * isa_temperature(z_m))
}

mach_number <- function(speed_ms, z_m) {
  speed_ms / speed_of_sound(z_m)
}

# =============================================================================
# AERODYNAMIC DRAG
# Model: a_drag = -ρ(z) * v² / (2β) * v̂
# β = m/(Cd * A_ref) [kg/m²] — ballistic coefficient
#   High β → low drag (heavy / aerodynamic body)
#   Low  β → high drag (light / blunt body)
# Typical values:
#   Cruise missile:     β ≈ 2000–5000 kg/m²
#   Fighter aircraft:   β ≈ 1000–3000 kg/m²
#   Interceptor rocket: β ≈ 1500–4000 kg/m²
# =============================================================================
drag_accel <- function(vel, z_m, beta_kg_m2) {
  spd <- sqrt(sum(vel^2))
  if (spd < 0.01 || beta_kg_m2 <= 0) return(rep(0, 3))
  rho <- isa_density(z_m)
  (-0.5 * rho * spd^2 / beta_kg_m2) * (vel / spd)
}

# =============================================================================
# RK4 INTEGRATOR
# f(t, y, ...) → dy/dt
# =============================================================================
rk4_step <- function(f, y, t, dt, ...) {
  k1 <- f(t,          y,           ...)
  k2 <- f(t + dt/2,   y + dt/2*k1, ...)
  k3 <- f(t + dt/2,   y + dt/2*k2, ...)
  k4 <- f(t + dt,     y + dt  *k3, ...)
  y + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)
}

# =============================================================================
# DYNAMICS DERIVATIVE FUNCTIONS  (for RK4)
# State vector: [x, y, z, vx, vy, vz]  (6 elements)
# =============================================================================

# ── Target dynamics ──────────────────────────────────────────────────────────
tgt_deriv <- function(t_val, state, params, a_maneuver) {
  pos <- state[1:3]
  vel <- state[4:6]
  spd <- sqrt(sum(vel^2))

  a_grav <- if (params$enable_gravity) G_VEC else rep(0, 3)
  a_drag <- if (params$enable_drag && spd > 0.01)
               drag_accel(vel, pos[3], params$tgt_beta)
            else rep(0, 3)

  # Speed-maintenance propulsion: thrust along velocity to counteract drag
  # (simulates a powered missile maintaining cruise speed)
  a_prop <- rep(0, 3)
  if (params$tgt_powered && spd > 0.01) {
    v_hat        <- vel / spd
    a_loss       <- sum((a_drag + a_grav) * v_hat)  # negative → speed-reducing
    thrust_needed <- max(0, min(-a_loss, params$tgt_max_thrust_g * G_ACCEL))
    a_prop       <- thrust_needed * v_hat
  }

  d_vel <- a_grav + a_drag + a_prop + a_maneuver
  c(vel, d_vel)
}

# ── Interceptor dynamics ─────────────────────────────────────────────────────
int_deriv <- function(t_val, state, params, a_guidance_actual) {
  pos <- state[1:3]
  vel <- state[4:6]
  spd <- sqrt(sum(vel^2))

  a_grav <- if (params$enable_gravity) G_VEC else rep(0, 3)
  a_drag <- if (params$enable_drag && spd > 0.01)
               drag_accel(vel, pos[3], params$int_beta)
            else rep(0, 3)

  # Interceptor propulsion: maintain speed against drag
  a_thrust <- rep(0, 3)
  if (params$int_powered && spd > 0.01) {
    v_hat        <- vel / spd
    a_loss       <- sum((a_drag + a_grav) * v_hat)
    thrust_needed <- max(0, min(-a_loss, params$max_thrust_g * G_ACCEL))
    a_thrust     <- thrust_needed * v_hat
  }

  d_vel <- a_grav + a_drag + a_thrust + a_guidance_actual
  c(vel, d_vel)
}

# =============================================================================
# GUIDANCE LAWS
# All return a 3D acceleration command vector [m/s²]
# Inputs use KF-estimated relative state (r_est, v_est)
# =============================================================================

# ── Pure Proportional Navigation ────────────────────────────────────────────
# a_cmd = N · Vc · (ω_LOS × r̂)
# Derivation: nulls the LOS angular rate → zero-lag intercept for non-maneuvering targets
guidance_pn <- function(r_rel, v_rel, N) {
  R <- sqrt(sum(r_rel^2))
  if (R < 1e-3) return(rep(0, 3))
  r_hat   <- r_rel / R
  Vc      <- -sum(r_hat * v_rel)           # closing speed [m/s], positive = approaching
  omega   <- cross3(r_hat, v_rel) / R      # LOS angular rate [rad/s]
  N * Vc * cross3(omega, r_hat)
}

# ── Augmented Proportional Navigation ───────────────────────────────────────
# a_cmd = N·Vc·(ω_LOS × r̂) + (N/2)·a_target_perp
# Adds feedforward for known/estimated target acceleration → handles maneuvering targets
guidance_apn <- function(r_rel, v_rel, a_tgt_est, N) {
  a_base <- guidance_pn(r_rel, v_rel, N)
  R <- sqrt(sum(r_rel^2))
  if (R < 1e-3) return(a_base)
  r_hat    <- r_rel / R
  a_t_perp <- a_tgt_est - sum(a_tgt_est * r_hat) * r_hat   # component ⊥ LOS
  a_base + (N / 2) * a_t_perp
}

# ── Optimal Guidance Law (ZEM-based) ─────────────────────────────────────────
# a_cmd = 6 · ZEM / t_go²
# Gravity-compensated ZEM: ZEM = r_rel + v_rel·t_go - 0.5·g·t_go²
# Without gravity compensation the missile wastes lateral authority
# fighting the trajectory curvature induced by gravity over t_go.
# Ref: Zarchan, "Tactical and Strategic Missile Guidance", Ch. 4
guidance_ogl <- function(r_rel, v_rel, Vc_est = NULL, g_vec = NULL) {
  R <- sqrt(sum(r_rel^2))
  if (R < 1e-3) return(rep(0, 3))
  r_hat  <- r_rel / R
  Vc     <- if (!is.null(Vc_est)) Vc_est else max(-sum(r_hat * v_rel), 0.1)
  t_go   <- max(R / max(Vc, 1.0), 0.1)
  grav_corr <- if (!is.null(g_vec)) 0.5 * g_vec * t_go^2 else rep(0, 3)
  ZEM    <- r_rel + v_rel * t_go - grav_corr
  6 * ZEM / t_go^2
}

# ── Time-Varying PN (biased for endgame) ─────────────────────────────────────
# N_eff = N_base · (1 + k / t_go) → increases as t_go decreases
# Compensates for increasing guidance sensitivity at terminal phase
guidance_tvpn <- function(r_rel, v_rel, N_base) {
  R <- sqrt(sum(r_rel^2))
  if (R < 1e-3) return(rep(0, 3))
  r_hat  <- r_rel / R
  Vc     <- max(-sum(r_hat * v_rel), 0.1)
  t_go   <- R / Vc
  N_eff  <- N_base * min(3.0, 1 + 1.0 / max(t_go, 0.5))    # bias factor
  guidance_pn(r_rel, v_rel, N_eff)
}

# =============================================================================
# ACTUATOR DYNAMICS  —  First-Order Lag
# Models fin/TVC actuation bandwidth
# τ · ȧ_actual + a_actual = a_cmd
# Discrete: a(k+1) = a(k) + (dt/τ) · (a_cmd - a(k))
# τ = 0: instantaneous (ideal) · τ = 0.2 s: typical fin actuator
# =============================================================================
actuator_step <- function(a_actual, a_cmd, dt, tau) {
  if (tau < 1e-6) return(a_cmd)
  a_actual + (dt / tau) * (a_cmd - a_actual)
}

# =============================================================================
# CORRELATED RADAR NOISE  —  AR(1) Process
# n(k) = ρ · n(k-1) + σ · √(1−ρ²) · w,  w ~ N(0,1)
# ρ = exp(−dt / τ_c),  τ_c = correlation time [s]
# τ_c → 0: white noise · τ_c → ∞: constant bias
# =============================================================================
ar1_step <- function(n_prev, sigma, tau_c, dt) {
  if (tau_c < 1e-6) return(rnorm(length(n_prev), 0, sigma))
  rho <- exp(-dt / tau_c)
  rho * n_prev + sigma * sqrt(max(1 - rho^2, 0)) * rnorm(length(n_prev))
}

# =============================================================================
# 6-STATE KALMAN FILTER  (Position + Velocity, Constant-Velocity model)
#
# State:    x = [px, py, pz, vx, vy, vz]^T
# Process:  x(k+1) = F·x(k) + w,  w ~ N(0, Q)
#           F = [I3  dt·I3 ]   (constant velocity)
#               [0   I3   ]
# Measurement: z = H·x + v,  v ~ N(0, R_cov)
#   Position only: H = [I3 | 0]
#   + Doppler:     H = [I3 | I3]  (also measures velocity)
#
# Process noise Q: discrete Wiener model
#   Accounts for unknown target acceleration up to σ_q [m/s²]
# =============================================================================
kf_init <- function(pos_init, vel_init, sigma_p = 200, sigma_v = 80) {
  list(
    x = c(pos_init, vel_init),
    P = diag(c(rep(sigma_p^2, 3), rep(sigma_v^2, 3)))
  )
}

kf_predict <- function(kf, dt, sigma_q = 40) {
  # State transition
  I3    <- diag(3)
  O3    <- matrix(0, 3, 3)
  F_mat <- rbind(cbind(I3, dt * I3), cbind(O3, I3))

  # Discrete Wiener process noise (Singer-like acceleration model)
  q     <- sigma_q^2
  dt2   <- dt^2;  dt3 <- dt^3;  dt4 <- dt^4
  Q_mat <- q * rbind(
    cbind(dt4/4 * I3, dt3/2 * I3),
    cbind(dt3/2 * I3, dt2   * I3)
  )

  list(
    x = as.vector(F_mat %*% kf$x),
    P = F_mat %*% kf$P %*% t(F_mat) + Q_mat
  )
}

kf_update <- function(kf_pred, z_meas, R_meas_cov) {
  n_meas <- length(z_meas)

  # Measurement matrix H
  if (n_meas == 3) {
    H_mat <- cbind(diag(3), matrix(0, 3, 3))         # position only
  } else {
    H_mat <- rbind(cbind(diag(3), matrix(0, 3, 3)),  # position + velocity
                   cbind(matrix(0, 3, 3), diag(3)))
  }

  # Innovation
  innov <- z_meas - as.vector(H_mat %*% kf_pred$x)

  # Innovation covariance  (needed for NIS)
  S_mat <- H_mat %*% kf_pred$P %*% t(H_mat) + R_meas_cov

  # Kalman gain (regularized inversion)
  S_reg <- S_mat + 1e-9 * diag(nrow(S_mat))
  K_mat <- tryCatch(
    kf_pred$P %*% t(H_mat) %*% solve(S_reg),
    error = function(e) kf_pred$P %*% t(H_mat) %*% diag(1 / diag(S_reg))
  )

  # Updated state and covariance
  x_upd <- kf_pred$x + as.vector(K_mat %*% innov)
  P_upd <- (diag(6) - K_mat %*% H_mat) %*% kf_pred$P

  # Ensure symmetry and positive-definiteness
  P_upd <- (P_upd + t(P_upd)) / 2 + 1e-8 * diag(6)

  # Normalised Innovation Squared (NIS)
  # Under correct filter, NIS ~ chi^2(n_meas).  For 3-D position: 95% bounds [0.35, 7.81]
  S3    <- S_mat[1:3, 1:3, drop = FALSE]
  nu3   <- innov[1:3]
  nis   <- tryCatch(as.numeric(t(nu3) %*% solve(S3 + 1e-9 * diag(3)) %*% nu3),
                    error = function(e) NA_real_)

  list(x = x_upd, P = P_upd, innov = innov, nis = nis)
}

# =============================================================================
# TARGET MANEUVER MODELS
# Returns instantaneous maneuver acceleration [m/s²] perpendicular to velocity
# =============================================================================
target_maneuver_accel <- function(t_val, tgt_state, params) {
  vel <- tgt_state[4:6]
  spd <- sqrt(sum(vel^2))
  if (spd < 0.1) return(rep(0, 3))

  v_hat    <- vel / spd
  max_a    <- params$tgt_max_g * G_ACCEL

  # Lateral directions perpendicular to velocity
  # lat1: horizontal lateral · lat2: vertical lateral
  up      <- c(0, 0, 1)
  lat1    <- cross3(v_hat, up)
  lat1_n  <- sqrt(sum(lat1^2))
  if (lat1_n < 1e-6) lat1 <- c(1, 0, 0) else lat1 <- lat1 / lat1_n
  lat2    <- cross3(v_hat, lat1)

  switch(params$maneuver_type,

    "none" = rep(0, 3),

    # Sinusoidal weave in lat1 direction
    "weave" = {
      params$weave_amp_g * G_ACCEL * sin(params$weave_freq * t_val) * lat1
    },

    # Bang-bang: hard turn alternating direction every half-period
    "bangbang" = {
      phase  <- (t_val %% params$bangbang_period) / params$bangbang_period
      sign_v <- if (phase < 0.5) 1.0 else -1.0
      sign_v * params$bangbang_g * G_ACCEL * lat1
    },

    # 3D spiral: combined lateral maneuver in two perpendicular planes
    "spiral" = {
      params$weave_amp_g * G_ACCEL * (
        sin(params$weave_freq * t_val) * lat1 +
        cos(params$weave_freq * t_val) * lat2
      )
    },

    # Stochastic: random acceleration with specified magnitude
    "random" = {
      a_rand <- rnorm(3)
      a_rand_perp <- a_rand - sum(a_rand * v_hat) * v_hat   # project out along-velocity
      a_n <- sqrt(sum(a_rand_perp^2))
      if (a_n > 0) params$weave_amp_g * G_ACCEL * a_rand_perp / a_n else rep(0,3)
    },

    rep(0, 3)
  )
}

# =============================================================================
# PREDICTED INTERCEPT POINT  (Collision Triangle)
# Solves for the earliest time t_f > 0 such that the interceptor
# (flying at v_int_mag from pos_i) can reach the target's predicted position.
# Quadratic: |pos_t + vel_t·t - pos_i|² = (v_int_mag·t)²
# =============================================================================
compute_predicted_intercept <- function(pos_i, vel_i, pos_t_est, vel_t_est,
                                         v_int_mag) {
  r   <- pos_t_est - pos_i
  a_q <- sum(vel_t_est^2) - v_int_mag^2
  b_q <- 2 * sum(r * vel_t_est)
  c_q <- sum(r^2)

  if (abs(a_q) < 1e-6) {
    if (abs(b_q) < 1e-6) return(NULL)
    t_f <- -c_q / b_q
  } else {
    disc <- b_q^2 - 4 * a_q * c_q
    if (disc < 0) return(NULL)
    t1 <- (-b_q + sqrt(disc)) / (2 * a_q)
    t2 <- (-b_q - sqrt(disc)) / (2 * a_q)
    pos_t <- c(t1, t2)[c(t1, t2) > 0.01]
    if (length(pos_t) == 0) return(NULL)
    t_f <- min(pos_t)
  }

  list(pos = pos_t_est + vel_t_est * t_f, t_go = t_f)
}

# =============================================================================
# MAIN SIMULATION LOOP
# =============================================================================
run_simulation <- function(params, seed = 42) {
  set.seed(seed)

  dt       <- params$dt
  t_max    <- params$t_max
  miss_thr <- params$miss_threshold
  max_a    <- params$max_accel_g * G_ACCEL

  # ── Initialize target state ────────────────────────────────────────────────
  tgt_v0  <- c(params$tgt_vx, params$tgt_vy, params$tgt_vz)
  tgt_spd <- sqrt(sum(tgt_v0^2))
  if (tgt_spd > 0) tgt_v0 <- tgt_v0 / tgt_spd * params$v_target
  tgt_state <- c(params$tgt_x0, params$tgt_y0, params$tgt_z0, tgt_v0)

  # ── Initialize interceptor state ──────────────────────────────────────────
  int_v0  <- c(params$int_vx0, params$int_vy0, params$int_vz0)
  int_spd <- sqrt(sum(int_v0^2))
  if (int_spd > 0) int_v0 <- int_v0 / int_spd * params$v_interceptor
  int_state <- c(params$int_x0, params$int_y0, params$int_z0, int_v0)

  # ── Initialize actuator state ──────────────────────────────────────────────
  a_actual  <- rep(0, 3)    # actual achieved acceleration

  # ── Initialize Kalman filter ───────────────────────────────────────────────
  kf <- kf_init(tgt_state[1:3], tgt_state[4:6],
                sigma_p = params$noise_pos * 5,
                sigma_v = params$kf_sigma_init_vel)
  kf_prev_vel <- kf$x[4:6]   # for finite-difference acceleration estimate

  # ── Correlated noise state ────────────────────────────────────────────────
  noise_p <- rep(0, 3)

  # ── Storage allocation ────────────────────────────────────────────────────
  times  <- seq(0, t_max, by = dt)
  n_max  <- length(times)

  S <- list(
    int_pos    = matrix(NA, n_max, 3),
    tgt_pos    = matrix(NA, n_max, 3),
    tgt_pos_m  = matrix(NA, n_max, 3),  # noisy radar
    tgt_pos_kf = matrix(NA, n_max, 3),  # KF position estimate
    tgt_vel_kf = matrix(NA, n_max, 3),  # KF velocity estimate
    int_vel    = matrix(NA, n_max, 3),
    tgt_vel    = matrix(NA, n_max, 3),
    a_cmd      = matrix(NA, n_max, 3),  # guidance command
    a_actual   = matrix(NA, n_max, 3),  # actuator output
    a_maneuver = matrix(NA, n_max, 3),  # target maneuver
    kf_innov   = matrix(NA, n_max, 3),  # KF innovation (measurement residual)
    pred_ip    = matrix(NA, n_max, 3),  # predicted intercept point
    kf_P_trace   = numeric(n_max),
    closing      = numeric(n_max),
    int_speed    = numeric(n_max),
    tgt_speed    = numeric(n_max),
    a_cmd_mag    = numeric(n_max),
    a_actual_mag = numeric(n_max),
    los_rate     = numeric(n_max),
    zem          = numeric(n_max),      # gravity-compensated ZEM magnitude [m]
    vc           = numeric(n_max),      # closing speed [m/s]
    mach_int     = numeric(n_max),
    mach_tgt     = numeric(n_max),
    drag_int_g   = numeric(n_max),
    drag_tgt_g   = numeric(n_max),
    alt_int      = numeric(n_max),
    alt_tgt      = numeric(n_max),
    tgo          = numeric(n_max),
    nis          = numeric(n_max),      # normalised innovation squared
    seeker_angle = numeric(n_max),      # angle between velocity and LOS [deg]
    seeker_track = numeric(n_max)       # 1 = tracking, 0 = open-loop
  )

  intercept_step <- NA_integer_
  intercept_pos  <- NULL

  # ──────────────────────────────────────────────────────────────────────────
  # MAIN LOOP
  # ──────────────────────────────────────────────────────────────────────────
  for (i in seq_along(times)) {
    t <- times[i]

    tgt_pos <- tgt_state[1:3]
    tgt_vel <- tgt_state[4:6]
    int_pos <- int_state[1:3]
    int_vel <- int_state[4:6]

    # ── True miss distance ─────────────────────────────────────────────────
    r_true  <- tgt_pos - int_pos
    R_true  <- sqrt(sum(r_true^2))

    # ── Store kinematics ───────────────────────────────────────────────────
    S$int_pos[i, ]   <- int_pos
    S$tgt_pos[i, ]   <- tgt_pos
    S$int_vel[i, ]   <- int_vel
    S$tgt_vel[i, ]   <- tgt_vel
    S$closing[i]     <- R_true
    S$int_speed[i]   <- sqrt(sum(int_vel^2))
    S$tgt_speed[i]   <- sqrt(sum(tgt_vel^2))
    S$alt_int[i]     <- int_pos[3]
    S$alt_tgt[i]     <- tgt_pos[3]

    # ── Mach numbers ──────────────────────────────────────────────────────
    S$mach_int[i]    <- mach_number(S$int_speed[i], int_pos[3])
    S$mach_tgt[i]    <- mach_number(S$tgt_speed[i], tgt_pos[3])

    # ── Drag diagnostics ──────────────────────────────────────────────────
    if (params$enable_drag) {
      a_di <- drag_accel(int_vel, int_pos[3], params$int_beta)
      a_dt <- drag_accel(tgt_vel, tgt_pos[3], params$tgt_beta)
      S$drag_int_g[i] <- sqrt(sum(a_di^2)) / G_ACCEL
      S$drag_tgt_g[i] <- sqrt(sum(a_dt^2)) / G_ACCEL
    }

    # ── KF state ──────────────────────────────────────────────────────────
    S$kf_P_trace[i]   <- sum(diag(kf$P)[1:3])
    S$tgt_pos_kf[i, ] <- kf$x[1:3]
    S$tgt_vel_kf[i, ] <- kf$x[4:6]
    S$a_actual[i, ]   <- a_actual
    S$a_actual_mag[i] <- sqrt(sum(a_actual^2)) / G_ACCEL

    # ── Relative geometry from KF ─────────────────────────────────────────
    r_est <- kf$x[1:3] - int_pos
    v_est <- kf$x[4:6] - int_vel
    R_est <- sqrt(sum(r_est^2))

    if (R_est > 0.1) {
      r_hat_est     <- r_est / R_est
      Vc_est        <- max(-sum(r_hat_est * v_est), 0.1)
      omega_est     <- cross3(r_hat_est, v_est) / R_est
      S$los_rate[i] <- sqrt(sum(omega_est^2))
      S$vc[i]       <- Vc_est

      t_go_est  <- R_est / Vc_est
      S$tgo[i]  <- t_go_est

      # Gravity-compensated ZEM
      grav_corr_zem <- if (params$enable_gravity) 0.5 * G_VEC * t_go_est^2 else rep(0, 3)
      ZEM_vec   <- r_est + v_est * t_go_est - grav_corr_zem
      S$zem[i]  <- sqrt(sum(ZEM_vec^2))

      # Seeker field-of-view check
      int_spd_now  <- sqrt(sum(int_vel^2))
      v_int_hat    <- if (int_spd_now > 0.1) int_vel / int_spd_now else r_hat_est
      cos_ang      <- pmax(-1, pmin(1, sum(v_int_hat * r_hat_est)))
      seeker_ang_r <- acos(cos_ang)
      seeker_ok    <- seeker_ang_r <= params$seeker_fov_rad
      S$seeker_angle[i] <- seeker_ang_r * (180 / pi)
      S$seeker_track[i] <- as.numeric(seeker_ok)

      # Predicted intercept point
      pip <- compute_predicted_intercept(
        int_pos, int_vel, kf$x[1:3], kf$x[4:6], params$v_interceptor)
      if (!is.null(pip)) S$pred_ip[i, ] <- pip$pos
    } else {
      seeker_ok <- TRUE
    }

    # ── Check intercept ────────────────────────────────────────────────────
    if (R_true < miss_thr) {
      intercept_step <- i
      intercept_pos  <- int_pos
      break
    }

    # ── Estimate target acceleration (finite diff of KF velocity) ─────────
    a_tgt_est <- (kf$x[4:6] - kf_prev_vel) / dt
    kf_prev_vel <- kf$x[4:6]

    # ── GUIDANCE COMMAND ──────────────────────────────────────────────────
    g_arg     <- if (params$enable_gravity) G_VEC else NULL
    a_cmd_vec <- switch(params$guidance_law,
      "pn"   = guidance_pn(r_est, v_est, params$N_pn),
      "apn"  = guidance_apn(r_est, v_est, a_tgt_est, params$N_pn),
      "ogl"  = guidance_ogl(r_est, v_est, Vc_est, g_vec = g_arg),
      "tvpn" = guidance_tvpn(r_est, v_est, params$N_pn),
      guidance_pn(r_est, v_est, params$N_pn)
    )

    # Gravity bias: add upward component to cancel gravity on the LOS
    # Prevents interceptor wasting lateral authority fighting gravity curvature
    if (params$enable_gravity) {
      a_cmd_vec <- a_cmd_vec - G_VEC   # +9.81 upward
    }

    # Seeker gate: zero guidance command if target outside seeker FOV
    if (!seeker_ok) a_cmd_vec <- rep(0, 3)

    # Clamp to max structural acceleration
    a_cmd_mag <- sqrt(sum(a_cmd_vec^2))
    if (a_cmd_mag > max_a) a_cmd_vec <- a_cmd_vec / a_cmd_mag * max_a
    S$a_cmd[i, ]    <- a_cmd_vec
    S$a_cmd_mag[i]  <- sqrt(sum(a_cmd_vec^2)) / G_ACCEL

    # ── ACTUATOR DYNAMICS ─────────────────────────────────────────────────
    a_actual <- actuator_step(a_actual, a_cmd_vec, dt, params$actuator_tau)

    # ── TARGET MANEUVER ───────────────────────────────────────────────────
    a_maneuver        <- target_maneuver_accel(t, tgt_state, params)
    S$a_maneuver[i, ] <- a_maneuver

    # ── RADAR MEASUREMENT (correlated noise, range-dependent) ─────────────
    noise_p <- ar1_step(noise_p, params$noise_pos, params$noise_tau_c, dt)
    range_scale <- 1 + params$noise_range_scale * R_true / 10000
    z_pos <- tgt_pos + noise_p * range_scale
    S$tgt_pos_m[i, ] <- z_pos

    # Measurement noise covariance (range-dependent)
    sigma_meas <- params$noise_pos * range_scale
    R_cov      <- diag(sigma_meas^2, 3)

    if (params$radar_doppler) {
      # Doppler: also observe velocity with noise
      noise_v <- rnorm(3, 0, params$noise_vel)
      z_vel   <- tgt_vel + noise_v
      z_meas  <- c(z_pos, z_vel)
      R_cov   <- rbind(cbind(R_cov,         matrix(0,3,3)),
                       cbind(matrix(0,3,3), diag(params$noise_vel^2, 3)))
    } else {
      z_meas <- z_pos
    }

    # ── KALMAN FILTER UPDATE ──────────────────────────────────────────────
    kf_pred    <- kf_predict(kf, dt, sigma_q = params$kf_sigma_q)
    kf_upd     <- kf_update(kf_pred, z_meas, R_cov)
    kf         <- list(x = kf_upd$x, P = kf_upd$P)
    S$kf_innov[i, ] <- kf_upd$innov[1:3]
    S$nis[i]        <- kf_upd$nis %||% NA_real_

    # ── RK4 INTEGRATION ───────────────────────────────────────────────────
    tgt_state <- rk4_step(tgt_deriv, tgt_state, t, dt,
                          params = params, a_maneuver = a_maneuver)
    int_state <- rk4_step(int_deriv, int_state, t, dt,
                          params = params, a_guidance_actual = a_actual)

    # Ground constraint
    if (tgt_state[3] < 0) { tgt_state[3] <- 0; if(tgt_state[6]<0) tgt_state[6]<-0 }
    if (int_state[3] < 0) { int_state[3] <- 0; if(int_state[6]<0) int_state[6]<-0 }

    # Speed cap (interceptor) — allow 5% overspeed from gravity/thrust
    i_spd <- sqrt(sum(int_state[4:6]^2))
    if (i_spd > params$v_interceptor * 1.05) {
      int_state[4:6] <- int_state[4:6] / i_spd * params$v_interceptor
    }
  }

  # ── Package results ───────────────────────────────────────────────────────
  n_v <- if (!is.na(intercept_step)) intercept_step else n_max

  result <- list(
    times          = times[1:n_v],
    intercept_step = intercept_step,
    intercept_pos  = intercept_pos,
    intercept_time = if (!is.na(intercept_step)) times[intercept_step] else NA_real_,
    miss_distance  = min(S$closing[1:n_v], na.rm = TRUE),
    naf            = compute_naf(params$N_pn),
    params         = params
  )
  for (nm in names(S)) {
    x <- S[[nm]]
    result[[nm]] <- if (is.matrix(x)) x[1:n_v, , drop = FALSE] else x[1:n_v]
  }
  result
}

# =============================================================================
# MONTE CARLO  —  varied noise seeds
# =============================================================================
run_monte_carlo <- function(params, n_runs = 20) {
  lapply(seq_len(n_runs), function(i) run_simulation(params, seed = i * 7 + 13))
}

# =============================================================================
# UTILITY
# =============================================================================
cross3 <- function(a, b) c(a[2]*b[3]-a[3]*b[2], a[3]*b[1]-a[1]*b[3], a[1]*b[2]-a[2]*b[1])

`%||%` <- function(a, b) if (!is.null(a) && length(a) > 0 && !is.na(a[1])) a else b

# =============================================================================
# NAF  —  Navigation Accuracy Factor
# Measures sensitivity of miss distance to initial heading error and sensor lag.
# For pure PN with N > 2:  NAF = N(N-2)/2
# Lower NAF = less sensitive to errors.  N=3 gives NAF=1.5; N=5 gives NAF=7.5.
# Ref: Zarchan, "Tactical and Strategic Missile Guidance", 6th ed., eq. 5.40
# =============================================================================
compute_naf <- function(N_pn) {
  if (N_pn <= 2) return(NA_real_)
  N_pn * (N_pn - 2) / 2
}

# =============================================================================
# VALIDATION  —  N sweep
# For each value of N run two simulations:
#   clean: zero noise, zero actuator lag, no target manoeuvre  (ideal PN)
#   full:  nominal noise and lag from base_params
# This lets the user verify that the simulation converges with the analytical
# trend: miss distance should decrease as N increases (up to about N=5),
# and NAF should increase (degrading tolerance to initial heading error).
# =============================================================================
run_n_sweep <- function(base_params, n_values = c(2, 3, 4, 5, 6)) {
  lapply(n_values, function(n) {
    p_clean <- base_params
    p_clean$N_pn          <- n
    p_clean$noise_pos     <- 0.01
    p_clean$noise_tau_c   <- 0
    p_clean$actuator_tau  <- 0
    p_clean$maneuver_type <- "none"

    p_full        <- base_params
    p_full$N_pn   <- n

    s_clean <- run_simulation(p_clean, seed = 1)
    s_full  <- run_simulation(p_full,  seed = 1)

    list(
      N          = n,
      NAF        = compute_naf(n),
      miss_clean = s_clean$miss_distance,
      miss_full  = s_full$miss_distance,
      it_clean   = s_clean$intercept_time,
      it_full    = s_full$intercept_time
    )
  })
}

# =============================================================================
# DEFAULT PARAMETERS
# =============================================================================
default_params <- function() {
  list(
    dt             = 0.02,   # s  (RK4 is accurate even at 0.05, 0.02 is excellent)
    t_max          = 25.0,   # s
    miss_threshold = 50,     # m

    # Physics toggles
    enable_gravity = TRUE,
    enable_drag    = FALSE,  # off by default; enable to see aerodynamic effects

    # Target
    v_target        = 500,
    tgt_x0 = 9000, tgt_y0 = 4000, tgt_z0 = 3000,
    tgt_vx = -350,  tgt_vy = -150, tgt_vz = 0,
    tgt_beta        = 4000,  # kg/m²
    tgt_powered     = TRUE,
    tgt_max_thrust_g= 5,
    tgt_max_g       = 5,     # g max maneuver

    # Target maneuver
    maneuver_type   = "weave",
    weave_amp_g     = 3,     # g
    weave_freq      = 0.25,  # rad/s
    bangbang_period = 4.0,   # s
    bangbang_g      = 3.0,   # g

    # Interceptor
    v_interceptor  = 700,
    int_x0 = 0, int_y0 = 0, int_z0 = 0,
    int_vx0 = 700, int_vy0 = 200, int_vz0 = 100,
    max_accel_g    = 30,
    int_beta       = 3000,   # kg/m²
    int_powered    = TRUE,
    max_thrust_g   = 8,
    actuator_tau   = 0.15,   # s  first-order lag

    # Guidance
    guidance_law   = "pn",
    N_pn           = 3.0,
    seeker_fov_rad = pi / 3,   # 60 deg half-angle (typical RF seeker)

    # Sensor
    noise_pos        = 15,    # m 1-sigma
    noise_vel        = 4,     # m/s (for Doppler mode)
    noise_tau_c      = 0.3,   # s  AR(1) correlation time
    noise_range_scale= 0.2,   # range-dependent noise factor
    radar_doppler    = FALSE, # Doppler velocity measurement

    # Kalman filter
    kf_sigma_q        = 40,   # m/s²  process noise (maneuver uncertainty)
    kf_sigma_init_vel = 60    # m/s   initial velocity uncertainty
  )
}
