# ============================================================
# app.R  -  Missile Intercept Lab
#
# A self-contained R Shiny application demonstrating
# proportional navigation guidance against a manoeuvring
# target, with full physics:
#
#   - RK4 numerical integration
#   - ISA atmosphere and aerodynamic drag
#   - Gravity (ENU frame)
#   - First-order actuator lag
#   - 6-state Kalman filter (position + velocity)
#   - AR(1) correlated, range-dependent radar noise
#   - Four guidance laws (PN, APN, OGL, TVPN)
#   - Five target manoeuvre profiles
#   - Monte Carlo capability
#
# Requirements:
#   install.packages(c("shiny", "plotly", "shinycssloaders", "shinyjs"))
#
# Run:
#   shiny::runApp("missile_app/")
# ============================================================

library(shiny)
library(plotly)
library(shinycssloaders)
library(shinyjs)

source("R/physics.R")
source("R/plots.R")

options(spinner.color = "#2980b9", spinner.type = 6, spinner.size = 0.7)

# ============================================================
# UI
# ============================================================
ui <- fluidPage(
  useShinyjs(),
  tags$head(
    tags$link(rel = "stylesheet", href = "style.css"),
    tags$title("Missile Intercept Lab")
  ),

  # Page header
  div(class = "page-header",
    h1("Missile Intercept Lab"),
    p("Proportional Navigation Guidance  |  RK4 Integration  |  Kalman Filter  |  ISA Atmosphere")
  ),

  sidebarLayout(

    # ==========================================================
    # SIDEBAR
    # ==========================================================
    sidebarPanel(
      width = 3,
      class = "sidebar",

      actionButton("run_sim", "Run Simulation",  class = "btn btn-primary-custom"),
      actionButton("run_mc",  "Monte Carlo",      class = "btn btn-secondary-custom"),

      # ----------------------------------------------------------
      div(class = "section-label", "Physics"),

      checkboxInput("enable_gravity", "Gravity  (9.81 m/s, ENU +Z up)", value = TRUE),
      checkboxInput("enable_drag",    "Aerodynamic drag  (ISA atmosphere)", value = FALSE),

      conditionalPanel("input.enable_drag == true",
        sliderInput("tgt_beta", "Target ballistic coefficient  (kg/m^2)",
                    min = 500, max = 10000, value = 4000, step = 100),
        sliderInput("int_beta", "Interceptor ballistic coefficient  (kg/m^2)",
                    min = 500, max = 10000, value = 3000, step = 100)
      ),

      # ----------------------------------------------------------
      div(class = "section-label", "Guidance Law"),

      selectInput("guidance_law", NULL,
        choices = c(
          "Pure PN"                = "pn",
          "Augmented PN  (APN)"   = "apn",
          "Optimal / ZEM  (OGL)"  = "ogl",
          "Time-Varying PN"        = "tvpn"
        ),
        selected = "pn"
      ),

      uiOutput("guidance_eq_ui"),

      sliderInput("N_pn", "Navigation constant  N",
                  min = 1, max = 7, value = 3, step = 0.5),

      # ----------------------------------------------------------
      div(class = "section-label", "Actuator Dynamics"),

      sliderInput("actuator_tau",
                  "Lag time constant  tau  (s)   [0 = ideal]",
                  min = 0, max = 0.5, value = 0.15, step = 0.01),

      # ----------------------------------------------------------
      div(class = "section-label", "Target"),

      sliderInput("v_target", "Speed  (m/s)",
                  min = 200, max = 900, value = 500, step = 10),

      selectInput("maneuver_type", "Evasive manoeuvre",
        choices = c(
          "None"             = "none",
          "Sinusoidal weave" = "weave",
          "Bang-bang"        = "bangbang",
          "3D spiral"        = "spiral",
          "Stochastic"       = "random"
        ),
        selected = "weave"
      ),

      conditionalPanel("input.maneuver_type != 'none'",
        sliderInput("weave_amp_g", "Manoeuvre amplitude  (g)",
                    min = 0, max = 10, value = 3, step = 0.5),
        conditionalPanel(
          "input.maneuver_type == 'weave' || input.maneuver_type == 'spiral'",
          sliderInput("weave_freq", "Weave frequency  (rad/s)",
                      min = 0.05, max = 2.0, value = 0.25, step = 0.05)
        ),
        conditionalPanel("input.maneuver_type == 'bangbang'",
          sliderInput("bangbang_period", "Bang-bang period  (s)",
                      min = 1, max = 10, value = 4, step = 0.5)
        )
      ),

      fluidRow(
        column(6, numericInput("tgt_x0", "Target X0 (m)", value = 9000, step = 500)),
        column(6, numericInput("tgt_y0", "Target Y0 (m)", value = 4000, step = 500))
      ),
      fluidRow(
        column(6, numericInput("tgt_z0", "Target Z0 (m)", value = 3000, step = 200)),
        column(6, numericInput("tgt_max_g", "Max manoeuvre (g)", value = 5, step = 1))
      ),
      fluidRow(
        column(6, numericInput("tgt_vx", "Target Vx (m/s)", value = -350, step = 50)),
        column(6, numericInput("tgt_vy", "Target Vy (m/s)", value = -150, step = 50))
      ),

      # ----------------------------------------------------------
      div(class = "section-label", "Interceptor"),

      sliderInput("v_interceptor", "Max speed  (m/s)",
                  min = 300, max = 1500, value = 700, step = 25),
      sliderInput("max_accel_g", "Max lateral acceleration  (g)",
                  min = 5, max = 60, value = 30, step = 5),
      sliderInput("max_thrust_g", "Max thrust  (g, forward)",
                  min = 0, max = 20, value = 8, step = 1),

      fluidRow(
        column(4, numericInput("int_x0", "Int X0", value = 0,   step = 100)),
        column(4, numericInput("int_y0", "Int Y0", value = 0,   step = 100)),
        column(4, numericInput("int_z0", "Int Z0", value = 0,   step = 100))
      ),

      # ----------------------------------------------------------
      div(class = "section-label", "Radar / Sensor"),

      sliderInput("noise_pos", "Position noise sigma  (m)",
                  min = 0, max = 100, value = 15, step = 1),
      sliderInput("noise_tau_c", "Noise correlation time tau_c  (s)",
                  min = 0, max = 2, value = 0.3, step = 0.05),
      sliderInput("noise_range_scale", "Range-dependent noise scale",
                  min = 0, max = 1, value = 0.2, step = 0.05),

      checkboxInput("radar_doppler", "Doppler velocity measurement", value = FALSE),
      conditionalPanel("input.radar_doppler == true",
        sliderInput("noise_vel", "Velocity noise sigma  (m/s)",
                    min = 0, max = 20, value = 4, step = 0.5)
      ),

      # ----------------------------------------------------------
      div(class = "section-label", "Kalman Filter"),

      sliderInput("kf_sigma_q", "Process noise sigma_q  (m/s^2)",
                  min = 5, max = 200, value = 40, step = 5),
      sliderInput("kf_sigma_init_vel",
                  "Initial velocity uncertainty sigma_v0  (m/s)",
                  min = 10, max = 200, value = 60, step = 5),

      # ----------------------------------------------------------
      div(class = "section-label", "Monte Carlo"),

      sliderInput("mc_runs", "Number of runs",
                  min = 5, max = 50, value = 20, step = 5),

      # ----------------------------------------------------------
      div(class = "section-label", "Animation"),

      uiOutput("anim_slider_ui"),

      fluidRow(
        column(6, checkboxInput("auto_play",        "Auto-play",      value = FALSE)),
        column(6, checkboxInput("show_noise_track", "Radar track",    value = TRUE))
      ),
      fluidRow(
        column(6, checkboxInput("show_kf_track",  "KF track",         value = TRUE)),
        column(6, checkboxInput("show_los_arrow", "LOS vector",       value = TRUE))
      ),
      checkboxInput("show_pred_ip", "Show predicted intercept point", value = TRUE)
    ),

    # ==========================================================
    # MAIN PANEL
    # ==========================================================
    mainPanel(
      width = 9,

      uiOutput("metrics_bar"),

      tabsetPanel(
        id = "main_tabs",

        # --------------------------------------------------------
        tabPanel("3D Engagement",
          withSpinner(plotlyOutput("plot_3d", height = "520px")),
          uiOutput("status_bar")
        ),

        # --------------------------------------------------------
        tabPanel("Guidance Metrics",
          fluidRow(
            column(6, withSpinner(plotlyOutput("plot_closing", height = "270px"))),
            column(6, withSpinner(plotlyOutput("plot_accel",   height = "270px")))
          ),
          fluidRow(
            column(6, withSpinner(plotlyOutput("plot_los",    height = "270px"))),
            column(6, withSpinner(plotlyOutput("plot_speeds", height = "270px")))
          )
        ),

        # --------------------------------------------------------
        tabPanel("Kinematics",
          fluidRow(
            column(6, withSpinner(plotlyOutput("plot_altitude", height = "270px"))),
            column(6, withSpinner(plotlyOutput("plot_drag",     height = "270px")))
          )
        ),

        # --------------------------------------------------------
        tabPanel("Kalman Filter",
          fluidRow(
            column(6, withSpinner(plotlyOutput("plot_kf_est",   height = "270px"))),
            column(6, withSpinner(plotlyOutput("plot_kf_innov", height = "270px")))
          ),
          uiOutput("kf_summary_ui")
        ),

        # --------------------------------------------------------
        tabPanel("Monte Carlo",
          withSpinner(plotlyOutput("plot_mc_3d",   height = "420px")),
          withSpinner(plotlyOutput("plot_mc_hist", height = "220px"))
        ),

        # --------------------------------------------------------
        tabPanel("Physics Reference",
          div(style = "padding: 20px; max-width: 860px; overflow-y: auto;",
            uiOutput("physics_ref_ui")
          )
        )
      )
    )
  )
)

# ============================================================
# SERVER
# ============================================================
server <- function(input, output, session) {

  # Build parameter list from current UI inputs
  sim_params <- reactive({
    p <- default_params()
    p$enable_gravity    <- input$enable_gravity
    p$enable_drag       <- input$enable_drag
    p$tgt_beta          <- input$tgt_beta
    p$int_beta          <- input$int_beta
    p$guidance_law      <- input$guidance_law
    p$N_pn              <- input$N_pn
    p$actuator_tau      <- input$actuator_tau
    p$v_target          <- input$v_target
    p$v_interceptor     <- input$v_interceptor
    p$max_accel_g       <- input$max_accel_g
    p$max_thrust_g      <- input$max_thrust_g
    p$maneuver_type     <- input$maneuver_type
    p$weave_amp_g       <- input$weave_amp_g
    p$weave_freq        <- input$weave_freq
    p$bangbang_period   <- input$bangbang_period
    p$tgt_max_g         <- input$tgt_max_g
    p$tgt_x0            <- input$tgt_x0
    p$tgt_y0            <- input$tgt_y0
    p$tgt_z0            <- input$tgt_z0
    p$tgt_vx            <- input$tgt_vx
    p$tgt_vy            <- input$tgt_vy
    p$int_x0            <- input$int_x0
    p$int_y0            <- input$int_y0
    p$int_z0            <- input$int_z0
    p$noise_pos         <- input$noise_pos
    p$noise_vel         <- input$noise_vel
    p$noise_tau_c       <- input$noise_tau_c
    p$noise_range_scale <- input$noise_range_scale
    p$radar_doppler     <- input$radar_doppler
    p$kf_sigma_q        <- input$kf_sigma_q
    p$kf_sigma_init_vel <- input$kf_sigma_init_vel
    p
  })

  rv <- reactiveValues(sim = NULL, mc = NULL, frame = 2L)

  # Run single simulation
  observeEvent(input$run_sim, {
    withProgress(message = "Running simulation...", value = 0.5, {
      rv$sim   <- run_simulation(sim_params())
      rv$frame <- max(2L, round(length(rv$sim$times) * 0.12))
      rv$mc    <- NULL
    })
  })

  # Default run at startup
  observe({
    isolate({
      if (is.null(rv$sim)) {
        rv$sim   <- run_simulation(default_params())
        rv$frame <- max(2L, round(length(rv$sim$times) * 0.12))
      }
    })
  })

  # Monte Carlo
  observeEvent(input$run_mc, {
    n <- input$mc_runs
    withProgress(message = sprintf("Monte Carlo: %d runs...", n), value = 0.3, {
      rv$mc <- run_monte_carlo(sim_params(), n_runs = n)
      updateTabsetPanel(session, "main_tabs", selected = "Monte Carlo")
    })
  })

  # Auto-play
  observe({
    if (isTRUE(input$auto_play) && !is.null(rv$sim)) {
      invalidateLater(55)
      isolate({
        n_max    <- length(rv$sim$times)
        rv$frame <- if (rv$frame >= n_max) 2L else rv$frame + 2L
      })
    }
  })

  observeEvent(input$anim_frame, { rv$frame <- input$anim_frame })

  # Animation slider
  output$anim_slider_ui <- renderUI({
    n <- if (!is.null(rv$sim)) length(rv$sim$times) else 100L
    sliderInput("anim_frame", "Animation frame",
                min = 2, max = n, value = isolate(rv$frame),
                step = 1, ticks = FALSE, animate = FALSE)
  })

  # ----------------------------------------------------------
  # 3D plot
  output$plot_3d <- renderPlotly({
    req(rv$sim)
    plot_3d_trajectory(rv$sim,
      show_noise   = isTRUE(input$show_noise_track),
      show_kf      = isTRUE(input$show_kf_track),
      show_los     = isTRUE(input$show_los_arrow),
      show_pred_ip = isTRUE(input$show_pred_ip),
      anim_frame   = rv$frame
    )
  })

  # Guidance metrics
  output$plot_closing <- renderPlotly({ req(rv$sim); plot_closing_and_zem(rv$sim) })
  output$plot_accel   <- renderPlotly({ req(rv$sim); plot_acceleration(rv$sim) })
  output$plot_los     <- renderPlotly({ req(rv$sim); plot_los_and_tgo(rv$sim) })
  output$plot_speeds  <- renderPlotly({ req(rv$sim); plot_speeds(rv$sim) })

  # Kinematics
  output$plot_altitude <- renderPlotly({ req(rv$sim); plot_altitude_mach(rv$sim) })
  output$plot_drag     <- renderPlotly({ req(rv$sim); plot_drag(rv$sim) })

  # Kalman filter
  output$plot_kf_est   <- renderPlotly({ req(rv$sim); plot_kf_estimate(rv$sim) })
  output$plot_kf_innov <- renderPlotly({ req(rv$sim); plot_kf_innovation(rv$sim) })

  # Monte Carlo
  output$plot_mc_3d   <- renderPlotly({ req(rv$mc); plot_monte_carlo_3d(rv$mc) })
  output$plot_mc_hist <- renderPlotly({ req(rv$mc); plot_miss_histogram(rv$mc) })

  # ----------------------------------------------------------
  # Metrics bar
  output$metrics_bar <- renderUI({
    req(rv$sim)
    s <- rv$sim
    f <- min(rv$frame, length(s$times))

    gv <- function(vec) if (!is.null(vec) && length(vec) >= f) vec[f] else NA_real_
    fmt <- function(x, d = 0) if (is.na(x)) "-" else sprintf(paste0("%.", d, "f"), x)

    kf_err <- tryCatch(
      sqrt(sum((s$tgt_pos_kf[f, ] - s$tgt_pos[f, ])^2)),
      error = function(e) NA_real_
    )

    div(class = "metric-row",
      div(class = "metric-card",
        div(class = "metric-label", "Interceptor speed"),
        p(class = "metric-value",   fmt(gv(s$int_speed))),
        span(class = "metric-unit", "m/s")),
      div(class = "metric-card red",
        div(class = "metric-label", "Target speed"),
        p(class = "metric-value",   fmt(gv(s$tgt_speed))),
        span(class = "metric-unit", "m/s")),
      div(class = "metric-card amber",
        div(class = "metric-label", "Closing range"),
        p(class = "metric-value",   fmt(gv(s$closing))),
        span(class = "metric-unit", "m")),
      div(class = "metric-card amber",
        div(class = "metric-label", "Accel (actual)"),
        p(class = "metric-value",   fmt(gv(s$a_actual_mag), 1)),
        span(class = "metric-unit", "g")),
      div(class = "metric-card",
        div(class = "metric-label", "Interceptor Mach"),
        p(class = "metric-value",   fmt(gv(s$mach_int), 2)),
        span(class = "metric-unit", "M")),
      div(class = "metric-card purple",
        div(class = "metric-label", "ZEM"),
        p(class = "metric-value",   fmt(gv(s$zem))),
        span(class = "metric-unit", "m")),
      div(class = "metric-card grey",
        div(class = "metric-label", "KF error"),
        p(class = "metric-value",   fmt(kf_err)),
        span(class = "metric-unit", "m")),
      div(class = "metric-card green",
        div(class = "metric-label", "Min miss dist"),
        p(class = "metric-value",
          if (!is.na(s$miss_distance)) fmt(s$miss_distance, 1) else "-"),
        span(class = "metric-unit", "m")),
      div(class = "metric-card green",
        div(class = "metric-label", "Intercept time"),
        p(class = "metric-value",
          if (!is.na(s$intercept_time)) fmt(s$intercept_time, 2) else "-"),
        span(class = "metric-unit", "s"))
    )
  })

  # Status bar
  output$status_bar <- renderUI({
    req(rv$sim)
    s <- rv$sim
    if (!is.na(s$intercept_time)) {
      div(class = "status-bar intercept",
        sprintf("INTERCEPT ACHIEVED  |  t = %.2f s  |  Miss distance = %.1f m  |  Guidance = %s  |  Actuator tau = %.2f s",
                s$intercept_time, s$miss_distance,
                toupper(s$params$guidance_law), s$params$actuator_tau))
    } else {
      div(class = "status-bar miss",
        sprintf("NO INTERCEPT  |  Closest approach = %.1f m  |  Try increasing N, interceptor speed, or switching to OGL",
                s$miss_distance))
    }
  })

  # Guidance equation
  output$guidance_eq_ui <- renderUI({
    eq <- switch(input$guidance_law,
      "pn"   = "a = N * Vc * (omega_LOS x r_hat)",
      "apn"  = "a = N*Vc*(omega_LOS x r_hat) + (N/2)*a_tgt_perp",
      "ogl"  = "a = 6 * ZEM / t_go^2   where ZEM = r + v * t_go",
      "tvpn" = "N_eff = N * (1 + 1/t_go)   (gain increases at terminal phase)",
      ""
    )
    div(class = "eq-box", eq)
  })

  # KF summary
  output$kf_summary_ui <- renderUI({
    req(rv$sim)
    s  <- rv$sim
    n  <- sum(!is.na(s$kf_P_trace))
    i0 <- sqrt(max(s$kf_P_trace[1], 0, na.rm = TRUE))
    i1 <- sqrt(max(s$kf_P_trace[n], 0, na.rm = TRUE))
    fe <- tryCatch(
      sqrt(sum((tail(s$tgt_pos_kf, 1) - tail(s$tgt_pos, 1))^2)),
      error = function(e) NA_real_
    )
    consistent <- !is.na(fe) && !is.na(i1) && fe < i1 * 2

    div(class = "kf-summary",
      div(class = "metric-card purple", style = "min-width: 180px;",
        div(class = "metric-label", "Initial 1-sigma (position)"),
        p(class   = "metric-value",  sprintf("%.1f", i0)),
        span(class = "metric-unit", "m")),
      div(class = "metric-card purple", style = "min-width: 180px;",
        div(class = "metric-label", "Final 1-sigma (position)"),
        p(class   = "metric-value",  sprintf("%.1f", i1)),
        span(class = "metric-unit", "m")),
      div(class = "metric-card purple", style = "min-width: 180px;",
        div(class = "metric-label", "Final estimation error"),
        p(class   = "metric-value",  if (!is.na(fe)) sprintf("%.1f", fe) else "-"),
        span(class = "metric-unit", "m")),
      div(class = if (consistent) "metric-card green" else "metric-card red",
          style = "min-width: 180px;",
        div(class = "metric-label", "Filter consistent (err < 2-sigma)?"),
        p(class   = "metric-value",  style = "font-size: 14px;",
          if (consistent) "Yes" else "No  (tune sigma_q)"))
    )
  })

  # Physics reference
  output$physics_ref_ui <- renderUI({
    tagList(
      tags$h4(style = "color: #2980b9; border-bottom: 1px solid #dde3ea; padding-bottom: 6px;",
              "Guidance Laws"),
      div(class = "eq-box",
        HTML("Pure PN:<br>
          a_cmd = N * Vc * (omega_LOS x r_hat)<br><br>
          r_hat    = unit LOS vector (interceptor to target)<br>
          Vc       = closing speed = -r_hat . v_rel  (m/s)<br>
          omega_LOS = (r_hat x v_rel) / |r|  (rad/s)<br>
          N        = navigation constant (3-5 typical)")),
      div(class = "eq-box",
        HTML("Augmented PN:<br>
          a_cmd = N*Vc*(omega_LOS x r_hat) + (N/2)*a_target_perp<br><br>
          a_target_perp estimated from KF velocity: a_est(k) = [v_kf(k) - v_kf(k-1)] / dt")),
      div(class = "eq-box",
        HTML("Optimal Guidance Law (ZEM-based):<br>
          a_cmd = 6 * ZEM / t_go^2<br>
          ZEM = r_rel + v_rel * t_go   (zero-effort miss vector)<br>
          t_go = |r| / Vc   (estimated time-to-go)<br><br>
          Minimises miss distance under energy-optimal (LQR) control")),
      div(class = "eq-box",
        HTML("Time-Varying PN:<br>
          N_eff(t) = N * min(3, 1 + 1/t_go)<br><br>
          Increases gain as engagement closes; better endgame performance")),

      tags$h4(style = "color: #8e44ad; border-bottom: 1px solid #dde3ea; padding-bottom: 6px; margin-top: 20px;",
              "Kalman Filter  (6-state, constant velocity)"),
      div(class = "eq-box",
        HTML("State: x = [px, py, pz, vx, vy, vz]<br><br>
          Predict:  x- = F*x,   P- = F*P*F' + Q<br>
          F = [I3  dt*I3]   (constant-velocity model)<br>
              [0   I3   ]<br>
          Q = discrete Wiener process noise  (sigma_q = manoeuvre uncertainty)<br><br>
          Update:   K = P-*H'*(H*P-*H' + R)^-1<br>
                    x = x- + K*(z - H*x-)<br>
                    P = (I - K*H)*P-<br><br>
          H = [I3 | 0]  position only, or  [I3 | I3]  with Doppler")),

      tags$h4(style = "color: #e67e22; border-bottom: 1px solid #dde3ea; padding-bottom: 6px; margin-top: 20px;",
              "Actuator Dynamics"),
      div(class = "eq-box",
        HTML("First-order lag:<br>
          tau * a_dot + a_actual = a_cmd<br>
          Discrete: a(k+1) = a(k) + (dt/tau) * (a_cmd - a(k))<br><br>
          tau = 0:    ideal (instantaneous)<br>
          tau = 0.1 s: fast fin actuator<br>
          tau = 0.3 s: slow actuator")),

      tags$h4(style = "color: #c0392b; border-bottom: 1px solid #dde3ea; padding-bottom: 6px; margin-top: 20px;",
              "Aerodynamics  (ISA + Ballistic Coefficient)"),
      div(class = "eq-box",
        HTML("ISA troposphere (0 to 11 km):<br>
          T(z) = 288.15 - 0.0065*z  (K)<br>
          rho(z) = 1.225 * (T/288.15)^4.256  (kg/m^3)<br><br>
          Drag deceleration:<br>
          a_drag = -rho(z)*v^2 / (2*beta) * v_hat<br>
          beta = m / (Cd * A)  (kg/m^2) - ballistic coefficient<br>
          Higher beta means less drag")),

      tags$h4(style = "color: #7f8c8d; border-bottom: 1px solid #dde3ea; padding-bottom: 6px; margin-top: 20px;",
              "Radar Noise Model"),
      div(class = "eq-box",
        HTML("AR(1) correlated noise:<br>
          n(k) = rho * n(k-1) + sigma * sqrt(1 - rho^2) * w,  w ~ N(0,1)<br>
          rho = exp(-dt / tau_c)<br><br>
          Range-dependent sigma:<br>
          sigma_eff(R) = sigma_0 * (1 + k_R * R / 10000)<br><br>
          tau_c = 0:  white noise<br>
          tau_c large: near-constant bias")),

      tags$h4(style = "color: #2c3e50; border-bottom: 1px solid #dde3ea; padding-bottom: 6px; margin-top: 20px;",
              "Numerical Integration  (RK4)"),
      div(class = "eq-box",
        HTML("4th-order Runge-Kutta:<br>
          y(t+dt) = y(t) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)<br>
          k1 = f(t, y)<br>
          k2 = f(t + dt/2, y + dt/2*k1)<br>
          k3 = f(t + dt/2, y + dt/2*k2)<br>
          k4 = f(t + dt,   y + dt*k3)<br><br>
          Global truncation error: O(dt^4)  vs  O(dt) for Euler"))
    )
  })
}

# ============================================================
# Launch
# ============================================================
shinyApp(ui = ui, server = server)
