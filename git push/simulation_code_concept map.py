# =============================================================
# TESLA MODEL 3 CIRCUIT SIMULATION — EXPLANATION & CONCEPT MAP
# =============================================================
#
# OVERVIEW:
# This simulation models how an electric vehicle (EV) — similar to a Tesla Model 3 —
# completes laps around a given track using real-world vehicle physics.
# It simulates power usage, acceleration, cornering, and energy efficiency over time.
#
# The system uses two main files:
#   1. simulation_code.py → handles all physics, motion, and visualization
#   2. simulation_ai.py   → (optional) runs a genetic algorithm optimizer
#       that finds the most efficient throttle/coast strategy for the straights
#
# =============================================================
# SECTION 1 — IMPORTS AND INITIAL SETTINGS
# =============================================================
# Import all Python libraries needed for data handling, math, and plotting.
# Matplotlib’s animation module updates the vehicle position in real-time.
# simulation_ai provides optional AI-based strategy optimization.
#
# SIMULATION_MODE chooses whether to run only the physics ("SIM"),
# optimization only ("GA"), or both ("BOTH").
# The track data is read from a CSV file that lists GPS coordinates.
#
# POLICY_SAVE_FILE is where we store or load the best throttle strategy
# (AI learns it once → then we can reuse it later).
#
# =============================================================
# SECTION 2 — VEHICLE PHYSICAL PARAMETERS
# =============================================================
# These constants define how the simulated vehicle behaves.
# - car_mass_kg: total mass affects inertia (Newton’s 2nd Law)
# - drag_coefficient and car_frontal_area_m2: determine aerodynamic drag
# - rolling_resistance_coefficient: represents tyre-road friction
# - motor_max_power_watts and motor_max_drive_force_newton: motor output limits
# - drivetrain_efficiency: energy lost in transmission and electronics
# - brake_max_force_newton: limit for braking system power
#
# NOTE:
# These values define how the car interacts with the environment,
# and are critical for realistic simulation results.
#
# =============================================================
# SECTION 3 — SPEED TARGETS AND LIMITS
# =============================================================
# These determine the lap targets for maintaining average speed:
# - target_average_speed_lower/upper define the acceptable range (in km/h and m/s)
# - safe_corner_speed_mps is the desired max speed in corners
# - maximum_vehicle_speed_mps caps the simulation at realistic limits
# - total_laps_to_simulate decides how many laps to run before stopping
# - stop_after_finish ensures animation halts automatically
#
# =============================================================
# SECTION 4 — SIMULATION TIME CONTROL
# =============================================================
# timeStep (simulation_time_step_seconds): how often we update physics equations.
# Smaller = smoother but slower simulation.
# frameInterval (animation_refresh_interval_ms): how often to refresh visuals.
# Decrease for faster animation, increase for smoother performance.
#
# =============================================================
# SECTION 5 — FORCE CALCULATION HELPERS
# =============================================================
# aerodynamic_drag(v): computes Fd = 0.5 * ρ * Cd * A * v²
# rolling_resistance(m): computes Frr = m * g * Cr
# These forces resist motion and are recalculated every frame.
#
# =============================================================
# SECTION 6 — LOAD AND PROCESS TRACK DATA
# =============================================================
# 1. Read the track coordinates (latitude, longitude) from CSV.
# 2. Convert them into local X-Y meters using Earth radius and trigonometry.
# 3. Compute segment lengths and directions.
# 4. Calculate total track distance.
#
# The resulting arrays:
#   track_points[x,y] → each coordinate on track
#   segment_lengths → meters between two points
#   segment_directions → normalized direction vectors
#   segment_distance_markers → cumulative distance for lookup
#
# =============================================================
# SECTION 7 — CORNER DETECTION (CURVATURE METHOD)
# =============================================================
# Calculates turning angle across each segment using a “window” of nearby points.
# If direction change exceeds a threshold (e.g., 9°), that segment is marked as a corner.
# This helps the control logic decide when to brake or coast.
#
# =============================================================
# SECTION 8 — STRAIGHT SECTION DETECTION
# =============================================================
# Groups all consecutive non-corner segments into “straights”.
# Each straight section stores:
#   start_distance, end_distance, and length.
# This allows us to assign different throttle strategies for each straight.
#
# =============================================================
# SECTION 9 — LOAD OR GENERATE THROTTLE POLICY
# =============================================================
# - If running in GA mode, we use the genetic algorithm to find an efficient throttle pattern.
# - Otherwise, it loads a pre-trained JSON file.
# - If no file exists, it creates a default policy with 70% throttle and 40% pulse fraction.
#
# POLICY STRUCTURE:
#   Each straight = (throttle_strength, pulse_fraction)
#   throttle_strength → 0.0 to 1.0 (how much power used)
#   pulse_fraction → portion of straight where throttle is applied before coasting.
#
# =============================================================
# SECTION 10 — INITIALIZE STATE VARIABLES
# =============================================================
# These store the current simulation state:
#   distance_m  → total meters driven
#   speed_mps   → current velocity
#   energy_used_wh → total energy consumption in Watt-hours
#   elapsed_time_s → total time elapsed
#   lap_counter → completed laps
#
# Telemetry arrays keep record for later analysis and plotting.
#
# =============================================================
# SECTION 11 — POSITION INTERPOLATION FUNCTION
# =============================================================
# get_position_from_distance(distance):
# Converts traveled distance into (x, y) coordinates on track.
# Uses segment interpolation (linear between two points).
# Used every frame to update the car’s position visually.
#
# =============================================================
# SECTION 12 — MAIN CONTROL STRATEGY FUNCTION
# =============================================================
# control_strategy(distance, speed, time):
# Core logic that decides whether to accelerate, coast, or brake.
#
# STEP 1: Compute current average speed.
# STEP 2: Identify current track segment (straight or corner).
# STEP 3: Estimate if a corner is coming soon using “lookahead distance”.
# STEP 4: Choose drive/brake forces:
#   - Launch mode: full power at low speed.
#   - Straights: throttle based on policy (AI or manual).
#   - Pre-corner: slow down early if corner detected ahead.
#   - In-corner: brake gently if above safe speed.
#   - Else: coast to save energy.
#
# STEP 5: Adjust throttle to maintain target average speed.
# STEP 6: Return computed driveForce, brakeForce, resistance, and descriptive state.
#
# =============================================================
# SECTION 13 — PHYSICS UPDATE LOOP (ANIMATION)
# =============================================================
# Every frame:
#   - Call control_strategy() to get forces and state.
#   - Compute acceleration = F_net / mass
#   - Update velocity and distance using timeStep (dt)
#   - Compute power draw = driveForce * velocity
#   - Update energy usage = (Power * dt) / 3600 → Wh
#   - Update lap count.
#
# =============================================================
# SECTION 14 — VISUALIZATION (MATPLOTLIB ANIMATION)
# =============================================================
# - The static track is drawn once.
# - A moving dot (car_dot) represents the vehicle position.
# - Color indicates behavior:
#     green = accelerating or throttle
#     blue  = coasting
#     red   = braking
#
# - A floating HUD displays:
#     lap count, speed, avg speed, power, energy, and state.
#
# - When total laps are reached, the animation stops automatically
#   and prints summary stats in the console.
#
# =============================================================
# SECTION 15 — AI OPTIMIZER (OPTIONAL)
# =============================================================
# The simulation_ai.py module can be called through run_ga().
# It runs a genetic algorithm that:
#   - Randomly mutates throttle policies.
#   - Simulates each one for fitness (time + energy balance).
#   - Evolves toward the most efficient lap strategy.
# The result is saved as a JSON policy file for future runs.
#
# =============================================================
# CONCEPTUAL FLOW PER FRAME
# =============================================================
# 1. Read current distance, velocity, and state.
# 2. Use strategy function to determine throttle or braking.
# 3. Compute acceleration and new position.
# 4. Update energy usage and lap progress.
# 5. Refresh visualization and HUD.
# 6. Repeat until all laps are complete.
#
# =============================================================
# END OF CODE CONCEPT MAP
# =============================================================
