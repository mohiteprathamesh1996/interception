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
      sliderInput("seeker_fov_deg", "Seeker FOV half-angle  (deg)",
                  min = 10, max = 90, value = 60, step = 5),

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

      # Engagement parameters bar
      uiOutput("engagement_bar"),

      # KPI telemetry table
      uiOutput("metrics_bar"),

      tabsetPanel(
        id = "main_tabs",

        # --------------------------------------------------------
        tabPanel("3D Engagement",
          withSpinner(plotlyOutput("plot_3d", height = "460px")),
          uiOutput("status_bar")
        ),

        # --------------------------------------------------------
        tabPanel("Guidance Metrics",
          fluidRow(
            column(6, withSpinner(plotlyOutput("plot_closing", height = "240px"))),
            column(6, withSpinner(plotlyOutput("plot_accel",   height = "240px")))
          ),
          fluidRow(
            column(6, withSpinner(plotlyOutput("plot_los",    height = "240px"))),
            column(6, withSpinner(plotlyOutput("plot_speeds", height = "240px")))
          )
        ),

        # --------------------------------------------------------
        tabPanel("Kinematics",
          fluidRow(
            column(6, withSpinner(plotlyOutput("plot_altitude", height = "300px"))),
            column(6, withSpinner(plotlyOutput("plot_drag",     height = "300px")))
          )
        ),

        # --------------------------------------------------------
        tabPanel("Kalman Filter",
          fluidRow(
            column(6, withSpinner(plotlyOutput("plot_kf_est",   height = "260px"))),
            column(6, withSpinner(plotlyOutput("plot_kf_innov", height = "260px")))
          ),
          uiOutput("kf_summary_ui")
        ),

        # --------------------------------------------------------
        tabPanel("Validation",
          div(style = "padding: 10px 14px 4px;",
            actionButton("run_validation", "Run N Sweep",
                         class = "btn btn-secondary-custom",
                         style = "width: auto; padding: 6px 20px;")
          ),
          fluidRow(
            column(6, withSpinner(plotlyOutput("plot_nis",       height = "280px"))),
            column(6, withSpinner(plotlyOutput("plot_seeker",    height = "280px")))
          ),
          withSpinner(plotlyOutput("plot_n_sweep", height = "300px")),
          uiOutput("validation_summary_ui")
        ),

        # --------------------------------------------------------
        tabPanel("Monte Carlo",
          withSpinner(plotlyOutput("plot_mc_3d",   height = "380px")),
          withSpinner(plotlyOutput("plot_mc_hist", height = "200px"))
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
    p$seeker_fov_rad    <- input$seeker_fov_deg * pi / 180
    p$bangbang_g        <- input$weave_amp_g     # use same amplitude slider
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

  rv <- reactiveValues(sim = NULL, mc = NULL, frame = 2L, n_sweep = NULL)

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
  # ----------------------------------------------------------
  # Engagement parameters bar
  output$engagement_bar <- renderUI({
    req(rv$sim)
    s   <- rv$sim
    p   <- s$params
    naf <- s$naf
    div(class = "engagement-bar",
      span(class = "eng-item",
        span(class = "eng-key", "Guidance"),
        span(class = "eng-val", toupper(p$guidance_law))),
      span(class = "eng-sep", "|"),
      span(class = "eng-item",
        span(class = "eng-key", "N"),
        span(class = "eng-val", p$N_pn)),
      span(class = "eng-sep", "|"),
      span(class = "eng-item",
        span(class = "eng-key", "NAF"),
        span(class = "eng-val",
             if (!is.na(naf)) sprintf("%.2f", naf) else "N/A")),
      span(class = "eng-sep", "|"),
      span(class = "eng-item",
        span(class = "eng-key", "Actuator tau"),
        span(class = "eng-val", sprintf("%.2f s", p$actuator_tau))),
      span(class = "eng-sep", "|"),
      span(class = "eng-item",
        span(class = "eng-key", "Seeker FOV"),
        span(class = "eng-val", sprintf("%.0f deg", p$seeker_fov_rad * 180 / pi))),
      span(class = "eng-sep", "|"),
      span(class = "eng-item",
        span(class = "eng-key", "Manoeuvre"),
        span(class = "eng-val", toupper(p$maneuver_type)))
    )
  })

  # ----------------------------------------------------------
  # KPI telemetry table  (3 x 3 grid)
  output$metrics_bar <- renderUI({
    req(rv$sim)
    s <- rv$sim
    f <- min(rv$frame, length(s$times))

    gv  <- function(vec) if (!is.null(vec) && length(vec) >= f) vec[f] else NA_real_
    fmt <- function(x, d = 0) if (is.na(x)) "-" else sprintf(paste0("%.", d, "f"), x)

    kf_err <- tryCatch(
      sqrt(sum((s$tgt_pos_kf[f, ] - s$tgt_pos[f, ])^2)),
      error = function(e) NA_real_
    )

    cell <- function(label, value, unit, cls = "") {
      div(class = paste("kpi-cell", cls),
        span(class = "kpi-label", label),
        div(class = "kpi-number-row",
          span(class = "kpi-value", value),
          span(class = "kpi-unit",  unit)
        )
      )
    }

    div(class = "kpi-table",
      cell("Interceptor speed",  fmt(gv(s$int_speed)),        "m/s"),
      cell("Target speed",       fmt(gv(s$tgt_speed)),        "m/s",  "status-alert"),
      cell("Closing range",      fmt(gv(s$closing)),          "m",    "status-warn"),
      cell("Accel (actual)",     fmt(gv(s$a_actual_mag), 1),  "g"),
      cell("Interceptor Mach",   fmt(gv(s$mach_int), 2),     "M"),
      cell("Zero-effort miss",   fmt(gv(s$zem)),              "m",    "status-warn"),
      cell("KF error",           fmt(kf_err),                 "m"),
      cell("Min miss distance",
           if (!is.na(s$miss_distance)) fmt(s$miss_distance, 1) else "-",
           "m",
           if (!is.na(s$miss_distance) &&
               s$miss_distance < s$params$miss_threshold) "status-good" else ""),
      cell("Intercept time",
           if (!is.na(s$intercept_time)) fmt(s$intercept_time, 2) else "-",
           "s",
           if (!is.na(s$intercept_time)) "status-good" else "")
    )
  })

  # ----------------------------------------------------------
  # Validation: N sweep
  observeEvent(input$run_validation, {
    withProgress(message = "Running N sweep (5 x 2 simulations)...", value = 0.4, {
      rv$n_sweep <- run_n_sweep(sim_params())
    })
  })

  output$plot_nis     <- renderPlotly({ req(rv$sim); plot_nis(rv$sim) })
  output$plot_seeker  <- renderPlotly({ req(rv$sim); plot_seeker_angle(rv$sim) })
  output$plot_n_sweep <- renderPlotly({
    req(rv$n_sweep)
    plot_n_sweep(rv$n_sweep)
  })

  output$validation_summary_ui <- renderUI({
    req(rv$sim)
    s   <- rv$sim
    nis <- s$nis
    nis_ok   <- mean(nis >= 0.352 & nis <= 7.815, na.rm = TRUE) * 100
    nis_hi   <- mean(nis > 7.815, na.rm = TRUE) * 100
    sk_ok    <- mean(s$seeker_track == 1, na.rm = TRUE) * 100
    naf_val  <- s$naf

    div(class = "kf-summary",
      div(class = "metric-card purple", style = "min-width: 180px;",
        div(class = "metric-label", "NIS within bounds"),
        p(class = "metric-value", style = "font-size:16px;",
          sprintf("%.0f%%", nis_ok)),
        span(class = "metric-unit", "of timesteps in chi^2(3) 95% interval")),
      div(class = if (nis_hi < 10) "metric-card green" else "metric-card red",
          style = "min-width: 180px;",
        div(class = "metric-label", "NIS exceedance"),
        p(class = "metric-value", style = "font-size:16px;",
          sprintf("%.0f%%", nis_hi)),
        span(class = "metric-unit",
             if (nis_hi < 10) "filter consistent" else "increase sigma_q")),
      div(class = "metric-card", style = "min-width: 180px;",
        div(class = "metric-label", "Seeker tracking"),
        p(class = "metric-value", style = "font-size:16px;",
          sprintf("%.0f%%", sk_ok)),
        span(class = "metric-unit", "of engagement")),
      div(class = "metric-card", style = "min-width: 180px;",
        div(class = "metric-label", "NAF  =  N(N-2)/2"),
        p(class = "metric-value", style = "font-size:16px;",
          if (!is.na(naf_val)) sprintf("%.2f", naf_val) else "N/A"),
        span(class = "metric-unit",
             sprintf("N = %.1f  |  sensitivity to heading error", s$params$N_pn)))
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

    ref_head <- function(text, col = "#2980b9") {
      tags$h4(style = sprintf(
        "color:%s; border-bottom:1px solid #dde3ea; padding-bottom:6px; margin-top:20px;", col),
        text)
    }

    tagList(

      ref_head("Guidance Laws"),
      div(class = "eq-box",
        HTML("Pure PN:<br>
          a_cmd = N * Vc * (omega_LOS x r_hat)  +  gravity_bias<br><br>
          r_hat      = unit LOS vector (interceptor to target)<br>
          Vc         = closing speed  =  -r_hat . v_rel  (m/s)<br>
          omega_LOS  = LOS angular rate  =  (r_hat x v_rel) / |r|  (rad/s)<br>
          gravity_bias = -g  (upward, cancels trajectory curvature due to gravity)<br>
          N          = navigation constant (3-5 typical)")),

      div(class = "eq-box",
        HTML("Augmented PN (APN):<br>
          a_cmd = N*Vc*(omega_LOS x r_hat) + (N/2)*a_target_perp  +  gravity_bias<br><br>
          a_target_perp estimated from KF velocity finite difference:<br>
          a_est(k)  =  [v_kf(k) - v_kf(k-1)] / dt")),

      div(class = "eq-box",
        HTML("Optimal Guidance Law (OGL)  -  gravity-compensated ZEM:<br>
          a_cmd = 6 * ZEM_gc / t_go^2<br><br>
          ZEM_gc = r_rel + v_rel*t_go - 0.5*g*t_go^2   (gravity-compensated)<br>
          t_go   = |r| / Vc<br><br>
          Without gravity compensation, the missile wastes lateral authority<br>
          fighting the ~0.5*g*t_go^2 downward deflection over t_go seconds.<br>
          At t_go = 5 s this is ~122 m of uncompensated miss.<br>
          Ref: Zarchan, Tactical and Strategic Missile Guidance, Ch. 4")),

      div(class = "eq-box",
        HTML("Time-Varying PN:<br>
          N_eff(t) = N * min(3, 1 + 1/t_go)<br><br>
          Gain ramps up automatically in the terminal phase.")),

      ref_head("Navigation Accuracy Factor (NAF)", "#c0392b"),
      div(class = "eq-box",
        HTML("NAF  =  N * (N - 2) / 2<br><br>
          Measures sensitivity of miss distance to heading errors, sensor lag,<br>
          and target manoeuvre.  Derived from the adjoint solution to the<br>
          linearised engagement equations.<br><br>
          N = 2:  NAF = 0   (insensitive, but marginal stability)<br>
          N = 3:  NAF = 1.5  (standard choice)<br>
          N = 4:  NAF = 4<br>
          N = 5:  NAF = 7.5  (high performance, but error-sensitive)<br><br>
          Higher NAF = better against maneuvering targets,<br>
          but worse miss distance if sensor noise is large.<br>
          Ref: Zarchan, eq. 5.40")),

      ref_head("Seeker Field-of-View Model", "#e67e22"),
      div(class = "eq-box",
        HTML("Gimbal angle check at each step:<br>
          alpha = acos( v_hat_interceptor . r_hat_LOS )<br><br>
          If alpha > FOV_half_angle:  guidance command = 0 (open-loop flight)<br>
          If alpha <= FOV_half_angle: guidance command active<br><br>
          Typical RF seeker:  FOV = 60 deg half-angle<br>
          Typical IR seeker:  FOV = 30-45 deg half-angle<br><br>
          Seeker dropout in the terminal phase is a real miss distance driver.<br>
          Monitor the Seeker Angle plot in the Validation tab.")),

      ref_head("Kalman Filter  (6-state, constant-velocity)", "#8e44ad"),
      div(class = "eq-box",
        HTML("State:  x = [px, py, pz, vx, vy, vz]<br><br>
          Predict:  x- = F*x,   P- = F*P*F' + Q<br>
          F = [I3  dt*I3]   (constant-velocity transition)<br>
              [0   I3   ]<br>
          Q = discrete Wiener process noise  (sigma_q = manoeuvre uncertainty)<br>
          Q_ij = sigma_q^2 * [dt^4/4  dt^3/2]   (position-velocity block)<br>
                              [dt^3/2  dt^2  ]<br><br>
          Update:   K = P-*H'*(H*P-*H' + R)^-1<br>
                    x = x- + K*(z - H*x-)<br>
                    P = (I - K*H)*P-<br><br>
          H = [I3 | 0]  position only<br>
          H = [I3 | I3] with Doppler velocity<br><br>
          Limitation: CV model lags during hard manoeuvres.<br>
          Increase sigma_q to compensate, or use an IMM filter.")),

      ref_head("Normalised Innovation Squared (NIS)", "#8e44ad"),
      div(class = "eq-box",
        HTML("NIS(k)  =  nu(k)' * S(k)^-1 * nu(k)<br><br>
          nu = innovation (measurement residual)<br>
          S  = innovation covariance  =  H*P-*H' + R<br><br>
          Under a consistent, well-tuned filter, NIS ~ chi^2(n_meas).<br>
          For 3-D position measurements:  95% bounds are [0.352, 7.815].<br><br>
          NIS consistently above 7.815:  filter over-confident -> increase sigma_q<br>
          NIS consistently below 0.352:  filter over-conservative -> decrease sigma_q<br><br>
          Ref: Bar-Shalom, Estimation with Applications to Tracking, Sec. 5.4")),

      ref_head("Actuator Dynamics"),
      div(class = "eq-box",
        HTML("First-order lag:<br>
          tau * a_dot + a_actual = a_cmd<br>
          Discrete:  a(k+1) = a(k) + (dt/tau) * (a_cmd - a(k))<br><br>
          tau = 0:     ideal (instantaneous)<br>
          tau = 0.1 s: fast fin actuator<br>
          tau = 0.3 s: slow actuator<br><br>
          The guidance-actuator lag product (tau * omega_LOS) is a key<br>
          driver of miss distance in the terminal phase.")),

      ref_head("Aerodynamics  (ISA + Ballistic Coefficient)", "#c0392b"),
      div(class = "eq-box",
        HTML("ISA troposphere (0 to 11 km):<br>
          T(z)   = 288.15 - 0.0065*z  (K)<br>
          rho(z) = 1.225 * (T/288.15)^4.256  (kg/m^3)<br><br>
          Drag deceleration:<br>
          a_drag = -rho(z)*v^2 / (2*beta) * v_hat<br>
          beta   = m / (Cd * A)  (kg/m^2)  ballistic coefficient<br><br>
          Typical values:<br>
          Cruise missile:     beta ~ 2000-5000 kg/m^2<br>
          Interceptor rocket: beta ~ 1500-4000 kg/m^2")),

      ref_head("Radar Noise Model", "#7f8c8d"),
      div(class = "eq-box",
        HTML("AR(1) correlated noise (more realistic than white noise):<br>
          n(k) = rho * n(k-1) + sigma * sqrt(1-rho^2) * w,  w ~ N(0,1)<br>
          rho  = exp(-dt / tau_c)<br><br>
          Range-dependent measurement noise:<br>
          sigma_eff(R) = sigma_0 * (1 + k_R * R / 10000)<br><br>
          tau_c = 0:  white noise (uncorrelated)<br>
          tau_c large: slowly-drifting bias")),

      ref_head("Numerical Integration  (RK4)", "#2c3e50"),
      div(class = "eq-box",
        HTML("4th-order Runge-Kutta:<br>
          y(t+dt) = y(t) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)<br>
          k1 = f(t,          y)<br>
          k2 = f(t + dt/2,   y + dt/2*k1)<br>
          k3 = f(t + dt/2,   y + dt/2*k2)<br>
          k4 = f(t + dt,     y + dt*k3)<br><br>
          Global truncation error: O(dt^4)  vs  O(dt) for Euler.<br>
          At dt = 0.02 s and v = 700 m/s, Euler accumulates ~3 m<br>
          per second of trajectory error.  RK4 accumulates < 0.01 m."))
    )
  })
}

# ============================================================
# Launch
# ============================================================
shinyApp(ui = ui, server = server)
