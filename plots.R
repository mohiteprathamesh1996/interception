# ============================================================
# plots.R  -  Plotly Visualization Engine
# Clean, professional style with no decorative characters
# ============================================================

library(plotly)

# ------------------------------------------------------------
# Colour palette  (restrained: blue / red / green / amber)
# ------------------------------------------------------------
COL <- list(
  target      = "#e74c3c",
  target_fill = "rgba(231,76,60,0.08)",
  target_noise= "rgba(231,76,60,0.18)",
  interceptor = "#2980b9",
  int_fill    = "rgba(41,128,185,0.08)",
  kf          = "#8e44ad",
  kf_fill     = "rgba(142,68,173,0.10)",
  intercept_pt= "#27ae60",
  los         = "#e67e22",
  accel       = "#c0392b",
  drag        = "#d35400",
  mach        = "#16a085",
  grid        = "#ecf0f1",
  border      = "#dde3ea",
  text        = "#2c3e50",
  subtext     = "#7f8c8d",
  bg          = "#ffffff",
  bg_plot     = "#fafbfc",
  font        = "Helvetica Neue, Helvetica, Arial, sans-serif",
  font_mono   = "Courier New, monospace"
)

# ------------------------------------------------------------
# Shared layout builder
# ------------------------------------------------------------
base_layout <- function(title = "", xlab = "Time (s)", ylab = "",
                         showlegend = TRUE) {
  list(
    title        = list(text  = title,
                        font  = list(family = COL$font, size = 12,
                                     color  = COL$text),
                        x     = 0),
    paper_bgcolor = COL$bg,
    plot_bgcolor  = COL$bg_plot,
    font          = list(family = COL$font, color = COL$subtext, size = 11),
    xaxis         = list(title     = xlab,
                         gridcolor = COL$grid,
                         linecolor = COL$border,
                         zerolinecolor = COL$border,
                         tickfont  = list(size = 10)),
    yaxis         = list(title     = ylab,
                         gridcolor = COL$grid,
                         linecolor = COL$border,
                         zerolinecolor = COL$border,
                         tickfont  = list(size = 10)),
    legend        = list(bgcolor     = "rgba(255,255,255,0.9)",
                         bordercolor = COL$border,
                         borderwidth = 1,
                         font        = list(size = 10, family = COL$font)),
    showlegend    = showlegend,
    margin        = list(l = 55, r = 20, t = 44, b = 44),
    hoverlabel    = list(bgcolor    = "#ffffff",
                         bordercolor= COL$border,
                         font       = list(family = COL$font_mono,
                                           size   = 11,
                                           color  = COL$text))
  )
}

scene_3d <- function() {
  ax <- list(
    backgroundcolor = COL$bg,
    gridcolor       = COL$grid,
    showbackground  = TRUE,
    zerolinecolor   = COL$border,
    tickfont        = list(size = 9, family = COL$font),
    titlefont       = list(size = 11, family = COL$font, color = COL$text)
  )
  list(
    xaxis      = c(ax, list(title = "X (m)")),
    yaxis      = c(ax, list(title = "Y (m)")),
    zaxis      = c(ax, list(title = "Z  altitude (m)")),
    bgcolor    = COL$bg,
    camera     = list(eye = list(x = 1.5, y = 1.4, z = 0.75)),
    aspectmode = "data"
  )
}

# ============================================================
# PLOT 1  -  3D Engagement Geometry
# ============================================================
plot_3d_trajectory <- function(sim, show_noise = TRUE, show_kf = TRUE,
                                show_los = TRUE, show_pred_ip = TRUE,
                                anim_frame = NULL) {
  ti <- sim$tgt_pos
  ii <- sim$int_pos
  tm <- sim$tgt_pos_m
  tk <- sim$tgt_pos_kf
  tt <- sim$times
  n  <- nrow(ti)

  fr <- if (!is.null(anim_frame)) max(2L, min(as.integer(anim_frame), n)) else n
  ti <- ti[1:fr, , drop = FALSE]
  ii <- ii[1:fr, , drop = FALSE]
  tm <- tm[1:fr, , drop = FALSE]
  tk <- tk[1:fr, , drop = FALSE]
  tt <- tt[1:fr]

  th <- sprintf("Target<br>t = %.2f s<br>X: %.0f m  Y: %.0f m  Z: %.0f m<br>Speed: %.0f m/s  Mach: %.2f",
                tt, ti[,1], ti[,2], ti[,3],
                sim$tgt_speed[1:fr], sim$mach_tgt[1:fr])
  ih <- sprintf("Interceptor<br>t = %.2f s<br>X: %.0f m  Y: %.0f m  Z: %.0f m<br>Speed: %.0f m/s  Mach: %.2f",
                tt, ii[,1], ii[,2], ii[,3],
                sim$int_speed[1:fr], sim$mach_int[1:fr])

  p <- plot_ly()

  # Noisy radar track
  if (show_noise) {
    p <- p |> add_trace(
      x = tm[,1], y = tm[,2], z = tm[,3],
      type = "scatter3d", mode = "lines",
      line = list(color = COL$target_noise, width = 1, dash = "dot"),
      name = "Radar track (noisy)", hoverinfo = "skip"
    )
  }

  # KF estimated track
  if (show_kf) {
    kh <- sprintf("KF Estimate<br>t = %.2f s<br>X: %.0f  Y: %.0f  Z: %.0f",
                  tt, tk[,1], tk[,2], tk[,3])
    p <- p |> add_trace(
      x = tk[,1], y = tk[,2], z = tk[,3],
      type = "scatter3d", mode = "lines",
      line = list(color = COL$kf, width = 2, dash = "dash"),
      name = "KF estimate", text = kh, hoverinfo = "text"
    )
  }

  # True target trajectory
  p <- p |>
    add_trace(
      x = ti[,1], y = ti[,2], z = ti[,3],
      type = "scatter3d", mode = "lines",
      line   = list(color = COL$target, width = 3),
      name   = "Target (true)", text = th, hoverinfo = "text"
    ) |>
    # Interceptor trajectory
    add_trace(
      x = ii[,1], y = ii[,2], z = ii[,3],
      type = "scatter3d", mode = "lines",
      line   = list(color = COL$interceptor, width = 3),
      name   = "Interceptor", text = ih, hoverinfo = "text"
    ) |>
    # Current positions
    add_trace(
      x = tail(ti[,1], 1), y = tail(ti[,2], 1), z = tail(ti[,3], 1),
      type   = "scatter3d", mode = "markers",
      marker = list(size = 9, color = COL$target, symbol = "diamond",
                    line = list(color = "#ffffff", width = 1)),
      name = "Target (current)", showlegend = FALSE, hoverinfo = "skip"
    ) |>
    add_trace(
      x = tail(ii[,1], 1), y = tail(ii[,2], 1), z = tail(ii[,3], 1),
      type   = "scatter3d", mode = "markers",
      marker = list(size = 9, color = COL$interceptor, symbol = "circle",
                    line = list(color = "#ffffff", width = 1)),
      name = "Interceptor (current)", showlegend = FALSE, hoverinfo = "skip"
    ) |>
    # Launch site
    add_trace(
      x = 0, y = 0, z = 0,
      type   = "scatter3d", mode = "markers",
      marker = list(size = 7, color = COL$interceptor, symbol = "square",
                    line = list(color = "#ffffff", width = 1)),
      name      = "Launch site",
      hovertext = "Launch site [0, 0, 0]", hoverinfo = "text"
    )

  # LOS vector
  if (show_los && fr >= 2) {
    ip      <- ii[fr, ]
    tp      <- ti[fr, ]
    los_len <- sqrt(sum((tp - ip)^2))
    los_dir <- if (los_len > 0) (tp - ip) / los_len else c(1, 0, 0)
    tip     <- ip + los_dir * min(los_len, 1500)
    p <- p |> add_trace(
      x = c(ip[1], tip[1]), y = c(ip[2], tip[2]), z = c(ip[3], tip[3]),
      type = "scatter3d", mode = "lines",
      line = list(color = COL$los, width = 2, dash = "dash"),
      name = "LOS vector",
      hovertext = sprintf("LOS distance: %.0f m", los_len), hoverinfo = "text"
    )
  }

  # Predicted intercept point
  if (show_pred_ip && fr >= 2) {
    pip <- sim$pred_ip[fr, ]
    if (all(!is.na(pip))) {
      p <- p |> add_trace(
        x = pip[1], y = pip[2], z = pip[3],
        type   = "scatter3d", mode = "markers",
        marker = list(size = 9, color = COL$los, symbol = "x",
                      line = list(color = "#ffffff", width = 1.5)),
        name      = "Predicted intercept",
        hovertext = sprintf("Predicted intercept<br>X: %.0f  Y: %.0f  Z: %.0f",
                            pip[1], pip[2], pip[3]),
        hoverinfo = "text"
      )
    }
  }

  # True intercept marker
  if (!is.na(sim$intercept_time) && !is.null(sim$intercept_pos)) {
    ip2 <- sim$intercept_pos
    p <- p |> add_trace(
      x = ip2[1], y = ip2[2], z = ip2[3],
      type   = "scatter3d", mode = "markers",
      marker = list(size = 16, color = COL$intercept_pt, symbol = "x",
                    line = list(color = "#ffffff", width = 2)),
      name = sprintf("Intercept  t = %.2f s", sim$intercept_time),
      hovertext = sprintf("Intercept achieved<br>t = %.2f s<br>Miss distance: %.1f m",
                          sim$intercept_time, sim$miss_distance),
      hoverinfo = "text"
    )
  }

  p |> layout(
    paper_bgcolor = COL$bg,
    scene         = scene_3d(),
    title = list(
      text = "3D Engagement Geometry",
      font = list(family = COL$font, size = 13, color = COL$text),
      x    = 0.02
    ),
    legend = list(bgcolor = "rgba(255,255,255,0.92)", bordercolor = COL$border,
                  borderwidth = 1,
                  font = list(size = 10, family = COL$font, color = COL$text)),
    margin     = list(l = 0, r = 0, t = 44, b = 0),
    hoverlabel = list(bgcolor = "#ffffff", bordercolor = COL$border,
                      font = list(family = COL$font_mono, size = 10,
                                  color = COL$text))
  )
}

# ============================================================
# PLOT 2  -  Closing Distance + ZEM
# ============================================================
plot_closing_and_zem <- function(sim) {
  t  <- sim$times
  n  <- length(t)

  p <- plot_ly() |>
    add_trace(x = t, y = sim$closing,
              type = "scatter", mode = "lines",
              line = list(color = COL$los, width = 2),
              fill = "tozeroy", fillcolor = "rgba(230,126,34,0.07)",
              name = "True range",
              hovertemplate = "t = %{x:.2f} s<br>Range = %{y:.1f} m<extra></extra>") |>
    add_trace(x = t, y = sim$zem,
              type = "scatter", mode = "lines",
              line = list(color = COL$kf, width = 1.8, dash = "dash"),
              name = "Zero-effort miss (ZEM)",
              hovertemplate = "t = %{x:.2f} s<br>ZEM = %{y:.1f} m<extra></extra>") |>
    add_trace(x = t, y = rep(sim$params$miss_threshold, n),
              type = "scatter", mode = "lines",
              line = list(color = COL$intercept_pt, width = 1, dash = "dot"),
              name = sprintf("Kill radius (%.0f m)", sim$params$miss_threshold),
              hoverinfo = "skip")

  if (!is.na(sim$intercept_time)) {
    p <- p |> add_trace(
      x = c(sim$intercept_time, sim$intercept_time),
      y = c(0, max(sim$closing, na.rm = TRUE)),
      type = "scatter", mode = "lines",
      line = list(color = COL$intercept_pt, width = 1, dash = "dash"),
      name = "Intercept time", hoverinfo = "skip"
    )
  }

  p |> layout(base_layout("Closing Range and Zero-Effort Miss",
                           ylab = "Distance (m)"))
}

# ============================================================
# PLOT 3  -  Speed vs Time
# ============================================================
plot_speeds <- function(sim) {
  t <- sim$times
  plot_ly() |>
    add_trace(x = t, y = sim$tgt_speed,
              type = "scatter", mode = "lines",
              line = list(color = COL$target, width = 2),
              name = "Target speed",
              hovertemplate = "t = %{x:.2f} s<br>V_tgt = %{y:.0f} m/s<extra></extra>") |>
    add_trace(x = t, y = sim$int_speed,
              type = "scatter", mode = "lines",
              line = list(color = COL$interceptor, width = 2),
              name = "Interceptor speed",
              hovertemplate = "t = %{x:.2f} s<br>V_int = %{y:.0f} m/s<extra></extra>") |>
    add_trace(x = t, y = sim$vc,
              type = "scatter", mode = "lines",
              line = list(color = COL$intercept_pt, width = 1.5, dash = "dot"),
              name = "Closing speed Vc",
              hovertemplate = "t = %{x:.2f} s<br>Vc = %{y:.0f} m/s<extra></extra>") |>
    layout(base_layout("Speed vs Time", ylab = "Speed (m/s)"))
}

# ============================================================
# PLOT 4  -  Guidance Acceleration  (commanded vs actuator)
# ============================================================
plot_acceleration <- function(sim) {
  t   <- sim$times
  n_a <- length(sim$a_cmd_mag)

  plot_ly() |>
    add_trace(x = t[1:n_a], y = sim$a_cmd_mag[1:n_a],
              type = "scatter", mode = "lines",
              line = list(color = COL$accel, width = 2),
              fill = "tozeroy", fillcolor = "rgba(192,57,43,0.06)",
              name = "Commanded (g)",
              hovertemplate = "t = %{x:.2f} s<br>a_cmd = %{y:.2f} g<extra></extra>") |>
    add_trace(x = t, y = sim$a_actual_mag,
              type = "scatter", mode = "lines",
              line = list(color = COL$los, width = 1.8, dash = "dash"),
              name = sprintf("Actuator output  (tau = %.2f s)", sim$params$actuator_tau),
              hovertemplate = "t = %{x:.2f} s<br>a_actual = %{y:.2f} g<extra></extra>") |>
    add_trace(x = t, y = rep(sim$params$max_accel_g, length(t)),
              type = "scatter", mode = "lines",
              line = list(color = "#bdc3c7", width = 1, dash = "dot"),
              name = sprintf("Max (%.0f g)", sim$params$max_accel_g),
              hoverinfo = "skip") |>
    layout(base_layout("Guidance Acceleration: Commanded vs Actuator",
                       ylab = "Acceleration (g)"))
}

# ============================================================
# PLOT 5  -  LOS Rate + Time-to-Go
# ============================================================
plot_los_and_tgo <- function(sim) {
  t <- sim$times
  plot_ly() |>
    add_trace(x = t, y = sim$los_rate * (180 / pi),
              type = "scatter", mode = "lines",
              line = list(color = COL$los, width = 2),
              fill = "tozeroy", fillcolor = "rgba(230,126,34,0.06)",
              name = "LOS rate (deg/s)",
              hovertemplate = "t = %{x:.2f} s<br>omega = %{y:.4f} deg/s<extra></extra>") |>
    add_trace(x = t, y = sim$tgo,
              type = "scatter", mode = "lines",
              line = list(color = COL$kf, width = 1.8, dash = "dash"),
              name = "Time-to-go (s)",
              yaxis = "y2",
              hovertemplate = "t = %{x:.2f} s<br>t_go = %{y:.2f} s<extra></extra>") |>
    layout(c(
      base_layout("LOS Angular Rate and Time-to-Go", ylab = "LOS rate (deg/s)"),
      list(yaxis2 = list(
        title         = "t_go (s)",
        overlaying    = "y",
        side          = "right",
        gridcolor     = "transparent",
        titlefont     = list(color = COL$kf),
        tickfont      = list(color = COL$kf, size = 10)
      ))
    ))
}

# ============================================================
# PLOT 6  -  Altitude + Mach Number
# ============================================================
plot_altitude_mach <- function(sim) {
  t <- sim$times
  plot_ly() |>
    add_trace(x = t, y = sim$alt_tgt,
              type = "scatter", mode = "lines",
              line = list(color = COL$target, width = 2),
              fill = "tozeroy", fillcolor = COL$target_fill,
              name = "Target altitude",
              hovertemplate = "t = %{x:.2f} s<br>Alt_tgt = %{y:.0f} m<extra></extra>") |>
    add_trace(x = t, y = sim$alt_int,
              type = "scatter", mode = "lines",
              line = list(color = COL$interceptor, width = 2),
              fill = "tozeroy", fillcolor = COL$int_fill,
              name = "Interceptor altitude",
              hovertemplate = "t = %{x:.2f} s<br>Alt_int = %{y:.0f} m<extra></extra>") |>
    add_trace(x = t, y = sim$mach_int,
              type = "scatter", mode = "lines",
              line = list(color = COL$mach, width = 1.8, dash = "dot"),
              name = "Interceptor Mach",
              yaxis = "y2",
              hovertemplate = "t = %{x:.2f} s<br>M_int = %{y:.2f}<extra></extra>") |>
    add_trace(x = t, y = sim$mach_tgt,
              type = "scatter", mode = "lines",
              line = list(color = COL$drag, width = 1.5, dash = "dot"),
              name = "Target Mach",
              yaxis = "y2",
              hovertemplate = "t = %{x:.2f} s<br>M_tgt = %{y:.2f}<extra></extra>") |>
    layout(c(
      base_layout("Altitude and Mach Number", ylab = "Altitude (m)"),
      list(yaxis2 = list(
        title      = "Mach",
        overlaying = "y", side = "right",
        gridcolor  = "transparent",
        titlefont  = list(color = COL$mach),
        tickfont   = list(color = COL$mach, size = 10)
      ))
    ))
}

# ============================================================
# PLOT 7  -  Aerodynamic Drag
# ============================================================
plot_drag <- function(sim) {
  t <- sim$times
  if (all(sim$drag_int_g == 0, na.rm = TRUE)) {
    return(
      plot_ly() |>
        add_annotations(
          x = 0.5, y = 0.5, xref = "paper", yref = "paper",
          text       = "Drag is disabled.  Enable it in Physics Settings.",
          font       = list(size = 12, color = COL$subtext, family = COL$font),
          showarrow  = FALSE
        ) |>
        layout(base_layout("Aerodynamic Drag Deceleration",
                            ylab = "Drag (g)"))
    )
  }
  plot_ly() |>
    add_trace(x = t, y = sim$drag_int_g,
              type = "scatter", mode = "lines",
              line = list(color = COL$drag, width = 2),
              fill = "tozeroy", fillcolor = "rgba(211,84,0,0.07)",
              name = sprintf("Interceptor drag  (beta = %.0f kg/m^2)",
                             sim$params$int_beta),
              hovertemplate = "t = %{x:.2f} s<br>D_int = %{y:.2f} g<extra></extra>") |>
    add_trace(x = t, y = sim$drag_tgt_g,
              type = "scatter", mode = "lines",
              line = list(color = COL$accel, width = 1.8, dash = "dash"),
              name = sprintf("Target drag  (beta = %.0f kg/m^2)",
                             sim$params$tgt_beta),
              hovertemplate = "t = %{x:.2f} s<br>D_tgt = %{y:.2f} g<extra></extra>") |>
    layout(base_layout("Aerodynamic Drag Deceleration", ylab = "Drag (g)"))
}

# ============================================================
# PLOT 8  -  Kalman Filter Estimation Quality
# ============================================================
plot_kf_estimate <- function(sim) {
  t         <- sim$times
  err       <- sqrt(rowSums((sim$tgt_pos_kf - sim$tgt_pos)^2))
  err[is.na(err)] <- NA
  cov_sigma <- sqrt(pmax(sim$kf_P_trace, 0))

  plot_ly() |>
    add_trace(x = t, y = cov_sigma,
              type = "scatter", mode = "lines",
              line = list(color = COL$kf, width = 1.5, dash = "dash"),
              fill = "tozeroy", fillcolor = COL$kf_fill,
              name = "1-sigma uncertainty (KF)",
              hovertemplate = "t = %{x:.2f} s<br>sigma = %{y:.1f} m<extra></extra>") |>
    add_trace(x = t, y = err,
              type = "scatter", mode = "lines",
              line = list(color = COL$interceptor, width = 2),
              name = "True estimation error",
              hovertemplate = "t = %{x:.2f} s<br>error = %{y:.1f} m<extra></extra>") |>
    layout(base_layout("Kalman Filter: Estimation Error vs 1-Sigma Uncertainty",
                       ylab = "Position error (m)"))
}

# ============================================================
# PLOT 9  -  KF Innovation (measurement residual)
# ============================================================
plot_kf_innovation <- function(sim) {
  t     <- sim$times
  innov <- sim$kf_innov
  if (is.null(innov) || all(is.na(innov))) {
    return(plot_ly() |> layout(base_layout("KF Innovation", ylab = "Residual (m)")))
  }
  innov_mag <- sqrt(rowSums(innov^2, na.rm = TRUE))

  plot_ly() |>
    add_trace(x = t, y = innov[, 1], type = "scatter", mode = "lines",
              line = list(color = COL$target,      width = 1.2),
              name = "Innovation X", hoverinfo = "skip") |>
    add_trace(x = t, y = innov[, 2], type = "scatter", mode = "lines",
              line = list(color = COL$interceptor, width = 1.2),
              name = "Innovation Y", hoverinfo = "skip") |>
    add_trace(x = t, y = innov[, 3], type = "scatter", mode = "lines",
              line = list(color = COL$mach,        width = 1.2),
              name = "Innovation Z", hoverinfo = "skip") |>
    add_trace(x = t, y = innov_mag, type = "scatter", mode = "lines",
              line = list(color = COL$los, width = 2),
              name = "Innovation magnitude",
              hovertemplate = "t = %{x:.2f} s<br>|v| = %{y:.1f} m<extra></extra>") |>
    layout(base_layout("Kalman Filter Innovation (Measurement Residual)",
                       ylab = "Residual (m)"))
}

# ============================================================
# PLOT 10  -  Monte Carlo 3D overlay
# ============================================================
plot_monte_carlo_3d <- function(mc_results) {
  miss_dists  <- sapply(mc_results, function(r) r$miss_distance)
  int_times   <- sapply(mc_results, function(r) r$intercept_time %||% NA_real_)
  n_intercept <- sum(!is.na(int_times))
  n_total     <- length(mc_results)

  blue_shades <- c("#2980b9","#3498db","#5dade2","#85c1e9","#aed6f1",
                   "#1a5276","#2471a3","#1f618d","#154360","#d6eaf8")

  p <- plot_ly()
  for (i in seq_along(mc_results)) {
    r   <- mc_results[[i]]
    ii  <- r$int_pos
    col <- blue_shades[((i - 1) %% length(blue_shades)) + 1]
    p <- p |> add_trace(
      x = ii[, 1], y = ii[, 2], z = ii[, 3],
      type = "scatter3d", mode = "lines",
      line    = list(color = col, width = 1.5),
      opacity = 0.5,
      name    = sprintf("Run %02d  miss = %.0f m", i, r$miss_distance),
      hovertemplate = sprintf("Run %d  miss distance: %.1f m<extra></extra>",
                               i, r$miss_distance)
    )
  }

  ti <- mc_results[[1]]$tgt_pos
  p <- p |> add_trace(
    x = ti[, 1], y = ti[, 2], z = ti[, 3],
    type = "scatter3d", mode = "lines",
    line = list(color = COL$target, width = 3),
    name = "Target (nominal)", hoverinfo = "skip"
  )

  p |> layout(
    paper_bgcolor = COL$bg,
    scene = scene_3d(),
    title = list(
      text = sprintf("Monte Carlo  -  %d runs  -  Intercepts: %d / %d  -  Median miss: %.1f m",
                     n_total, n_intercept, n_total,
                     median(miss_dists, na.rm = TRUE)),
      font = list(family = COL$font, size = 12, color = COL$text),
      x    = 0.02
    ),
    legend = list(bgcolor     = "rgba(255,255,255,0.9)",
                  bordercolor = COL$border,
                  borderwidth = 1,
                  font        = list(size = 9, family = COL$font)),
    margin = list(l = 0, r = 0, t = 46, b = 0)
  )
}

# ============================================================
# PLOT 11  -  Miss Distance Histogram
# ============================================================
plot_miss_histogram <- function(mc_results) {
  miss_dists  <- sapply(mc_results, function(r) r$miss_distance)
  kill_radius <- mc_results[[1]]$params$miss_threshold
  p_kill      <- mean(miss_dists <= kill_radius, na.rm = TRUE) * 100

  plot_ly() |>
    add_trace(
      x    = miss_dists,
      type = "histogram",
      nbinsx = 15,
      marker = list(
        color = COL$interceptor,
        line  = list(color = "#ffffff", width = 1)
      ),
      name = "Miss distance"
    ) |>
    add_segments(
      x = kill_radius, xend = kill_radius, y = 0, yend = 1,
      yref = "paper",
      line = list(color = COL$intercept_pt, dash = "dash", width = 1.5),
      showlegend = FALSE, hoverinfo = "skip"
    ) |>
    add_annotations(
      x = kill_radius, y = 0.95, yref = "paper",
      xanchor = "left",
      text    = sprintf("  Kill radius  Pk = %.1f%%", p_kill),
      font    = list(color  = COL$intercept_pt, family = COL$font, size = 11),
      showarrow = FALSE
    ) |>
    layout(base_layout(
      sprintf("Miss Distance Distribution  -  Pk = %.1f%%", p_kill),
      xlab = "Miss Distance (m)", ylab = "Count"
    ))
}
