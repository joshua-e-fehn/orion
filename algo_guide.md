# Prediction & Planning Guide — PX4 (ROS2, no MAVROS)

**Purpose**: Practical, implementation-focused guide for *prediction* (target motion prediction) and *planning/guidance* (interceptor guidance laws) from the paper **“Towards Safe Mid-Air Drone Interception: Strategies for Tracking & Capture”** (Pliska et al., IEEE RA-L 2024). This guide intentionally **omits perception/state-estimation** (you said you don't care about that) and focuses only on prediction of target motion and planning approaches (algorithms, pseudocode, ROS2 topic integration with PX4 `fmu` uORB ROS2 topics you provided).

Target audience: engineers with ROS2 + PX4 (uORB via ROS2 topics) experience, using Gazebo and RViz for simulation.

---

## Table of contents

1. Summary & recommended stack
2. Mapping: guidance outputs -> PX4 ROS2 topics (no MAVROS)
3. Prediction approaches (target motion forecasting)

   * Constant Velocity (CV) predictor
   * Constant Acceleration (CA) predictor
   * Interacting Multiple Model (IMM) predictor (CV+CA)
   * Practical notes about prediction uncertainty & time-to-go
   * Pseudocode for each
4. Planning / Guidance approaches

   * Pure Pursuit (PP)
   * Proportional Navigation (PN) — canonical
   * Linearized PN (LPN)
   * General PN (GPN) — brief notes
   * Fast Response PN (FRPN) — detailed (paper contribution)
   * Model Predictive Control (MPC) — detailed, ACADOS/solver guidance
   * Pseudocode for each
5. ROS2 node architecture & messages (no MAVROS)

   * Topics to subscribe/publish (using the provided `/fmu/*` list)
   * Offboard enabling & safety
   * Example flow: guidance -> actuator
6. Integration & testing checklist (Gazebo, SITL)
7. Tuning recipes & recommended parameter seeds (from paper)
8. Appendix: conversions, numerics, helper functions

---

# 1) Summary & recommended stack

* ROS2 (Foxy/Galactic/Rolling as you use) for nodes and pub/sub.
* PX4 (v1.12+) with ROS2 uORB bridge (topics listed in your message). **Do not use MAVROS.**
* Guidance nodes run on companion computer (ROS2). Use `rclcpp` (C++) for low-latency guidance; Python `rclpy` OK for prototyping.
* For MPC: use **ACADOS** or **qpOASES / OSQP** solver. ACADOS is recommended for fast embedded MPC; implement solver as a compiled library called from C++.
* Simulation: PX4 SITL + Gazebo, visualize in RViz.

**Recommendation:** Use a lightweight reactive controller (FRPN) as the primary guidance layer; add MPC only where constraints, obstacle avoidance, or smoother behavior are required.

---

# 2) Mapping guidance outputs -> PX4 ROS2 topics (no MAVROS)

Below are practical mappings to the `/fmu/in/*` and `/fmu/out/*` topics you provided. Choose the interface level appropriate for your vehicle and safety requirements.

**Common choices (ordered by abstraction / safety):**

* High level trajectory setpoint (preferred when PX4 position/trajectory controllers are used):

  * **Publish**: `/fmu/in/trajectory_setpoint` (use to provide full position/velocity/accel trajectory setpoints).
  * **Also useful**: `/fmu/in/goto_setpoint` for single-goal commands.

* Mid level velocity / attitude references (more direct control):

  * **Publish**: `/fmu/in/vehicle_rates_setpoint` or `/fmu/in/vehicle_attitude_setpoint_v1` + `/fmu/in/vehicle_thrust_setpoint` (for thrust control). Use these when you want to command body rates and thrust directly.

* Low-level actuator commands (least safe; bypasses many PX4 safety checks):

  * **Publish**: `/fmu/in/actuator_motors` or `/fmu/in/actuator_servos` if you have validated low-level control; **exercise extreme caution**.

* Mode and offboard control activation:

  * **Publish**: `/fmu/in/offboard_control_mode` — tell PX4 you will provide offboard setpoints.
  * **Publish**: `/fmu/in/vehicle_command` — for arming / mode switching if needed.

* Telemetry / introspection from PX4:

  * **Subscribe**: `/fmu/out/vehicle_odometry` or `/fmu/out/vehicle_local_position_v1` for interceptor state (position, velocity) — these are your closed-loop feedback.
  * **Subscribe**: `/fmu/out/vehicle_attitude` for body orientation.

**Mapping examples:**

* If your guidance law outputs a desired acceleration `a_cmd` in body or inertial frame:

  * Option A (safe): convert `a_cmd` -> short-horizon position/velocity trajectory and publish to `/fmu/in/trajectory_setpoint`.
  * Option B (direct): convert `a_cmd` -> `vehicle_rates_setpoint` + `vehicle_thrust_setpoint` (requires attitude control mapping).

**Which to pick?** For most multicopter setups, use `/fmu/in/trajectory_setpoint` or `/fmu/in/vehicle_rates_setpoint` combined with PX4's built-in controllers. Use `/fmu/in/actuator_motors` only for hardware-in-the-loop advanced setups.

---

# 3) Prediction approaches (target motion forecasting)

We assume your perception stack publishes a `TargetState` message (position `p_t`, velocity `v_t`, optional covariance `P_t`). Prediction nodes subscribe to that and produce predicted target future states `p_t(k+τ), v_t(k+τ)` used by guidance.

**Design principle:** predictors are lightweight. Predictors output both point forecasts and a covariance (uncertainty growth) so guidance can scale aggressiveness by confidence.

## 3.1 Constant Velocity (CV) predictor

**Model**: assumes target maintains current velocity `v_t`.

* Prediction: `p(t+τ) = p_t + v_t * τ`
* Optional uncertainty growth: `P(τ) = P_t + Q_cv * τ` (Q_cv is process noise intensity)

**Pseudocode**:

```python
# Inputs: p_t, v_t, P_t (covariance), Q_cv (process noise intensity)
def cv_predict(p_t, v_t, P_t, Q_cv, tau_list):
    predictions = []
    for tau in tau_list:
        p_pred = p_t + v_t * tau
        P_pred = P_t + Q_cv * tau
        predictions.append((tau, p_pred, v_pred=v_t, P_pred))
    return predictions
```

**When suitable**: target flying steady or short prediction horizon.

## 3.2 Constant Acceleration (CA) predictor

**Model**: assumes current acceleration `a_t` (if available) persists; otherwise estimate a_t via finite difference `(v_t - v_prev)/dt`.

* Prediction: `p(t+τ) = p_t + v_t * τ + 0.5 * a_t * τ^2`
* Velocity: `v(t+τ) = v_t + a_t * τ`
* Uncertainty: `P(τ) = P_t + Q_ca * τ` (or growing quadratically)

**Pseudocode**:

```python
# Inputs: p_t, v_t, a_t (or estimate), P_t, Q_ca
def ca_predict(p_t, v_t, a_t, P_t, Q_ca, tau_list):
    preds = []
    for tau in tau_list:
        p_pred = p_t + v_t * tau + 0.5 * a_t * (tau**2)
        v_pred = v_t + a_t * tau
        P_pred = P_t + Q_ca * tau
        preds.append((tau, p_pred, v_pred, P_pred))
    return preds
```

**When suitable**: if target shows smooth acceleration patterns or you can estimate short-term acceleration.

## 3.3 Interacting Multiple Model (IMM) predictor (CV + CA)

**Purpose**: adaptively mix CV and CA predictions to handle switching target maneuvers (e.g., periods of near-constant velocity and bursts of acceleration). IMM produces a mode probability vector and a fused predicted state.

**Key elements**:

* Two models (CV & CA) with their own state estimates and covariances.
* Model transition probability matrix `T` (e.g., p(stay)=0.95, p(switch)=0.05).
* At each time-step do: mixing → predict each model → compute likelihoods with the new measurement → update mode probabilities → combine estimates.

**IMM Prediction-only variant**: since you said you don't care about estimation, we use IMM as a prediction fusion mechanism: mix model forecasts according to likelihoods derived from recent innovations or fixed heuristics (e.g., accelerate when measured |Δv| large).

**Pseudocode (prediction-oriented IMM)**:

```python
# Inputs: models = [cv_model, ca_model] each with predict(state, P, tau_list)
# mu = [mu_cv, mu_ca] initial model probs
# T = 2x2 transition matrix
# meas_history: recent observed velocity changes to compute heuristics or likelihoods

def imm_predict(models, mu, T, meas_history, tau_list):
    # Mixing probabilities
    c_j = [sum(T[i][j] * mu[i] for i in range(2)) for j in range(2)]
    # For prediction-oriented use, mix using c_j as weights (no state mixing if not maintaining full KFs)
    model_preds = [models[i].predict(tau_list) for i in range(2)]
    # Optionally compute likelihoods L_i based on recent innovation magnitude
    L = compute_likelihoods_from_meas(meas_history, model_preds)
    # Update mu
    mu_new = normalize([c_j[i] * L[i] for i in range(2)])
    # Fuse predictions
    fused_preds = []
    for k, tau in enumerate(tau_list):
        p_fused = sum(mu_new[i] * model_preds[i][k].p_pred for i in range(2))
        v_fused = sum(mu_new[i] * model_preds[i][k].v_pred for i in range(2))
        P_fused = sum(mu_new[i] * model_preds[i][k].P_pred for i in range(2))
        fused_preds.append((tau, p_fused, v_fused, P_fused))
    return fused_preds, mu_new
```

**Notes**:

* For full IMM you would implement parallel KFs and compute exact likelihoods from innovation covariances. Here we use a pragmatic prediction-oriented approach; it's simpler and adequate if you already have target states.
* When `mu` favors CA, the fused prediction will predict curvature/accelerations; when it favors CV, predictions stay linear.

## 3.4 Practical notes about prediction & time-to-go (tgo)

* Many guidance laws use `t_go = ||Δp|| / ||Δv||` (time-to-go). Handle edge cases when `||Δv||` is small: clip denominator, or use a heuristic like `tgo = max(min_tgo, ||Δp|| / max(||Δv||, v_eps))`.
* Output predicted trajectory array for horizon `tau_list = [0, dt, 2dt, ..., N*dt]` to feed MPC or to compute expected interception point for PN variants.

---

# 4) Planning / Guidance approaches (the meat)

For each guidance law we provide: short description, when to use, required inputs, and detailed pseudocode. Guidance nodes assume subscription to:

* interceptor pose & velocity: `/fmu/out/vehicle_odometry` or `/fmu/out/vehicle_local_position_v1`
* *predicted* target states: from predictor node (see section 3) — can be either current `p_t, v_t` or predicted `p_t(t+τ)` depending on method.

## Shared helper functions (used in pseudocode)

```python
def norm(v):
    return sqrt(v.dot(v))

def clamp_vec(v, max_vec):
    return np.clip(v, -max_vec, max_vec)

# convert accel->velocity setpoint using small dt (for trajectory_setpoint or velocity setpoint publishing)
def accel_to_velocity_setpoint(v_current, a_cmd, dt_cmd):
    return v_current + a_cmd * dt_cmd
```

## 4.1 Pure Pursuit (PP)

**Inputs**: interceptor p_i, v_i; target p_t (and optionally v_t). Output: acceleration command `a_cmd` proportional to position error.

**Pseudocode**:

```python
# Params: G_pp (gain), amax
def pure_pursuit(p_i, v_i, p_t, G_pp, amax):
    dp = p_t - p_i
    a_cmd = G_pp * dp
    a_cmd = clamp_vec(a_cmd, amax)
    return a_cmd
```

**Publish**: convert `a_cmd` to `/fmu/in/trajectory_setpoint` (short-horizon) or velocity setpoint.

**Notes**: very simple but can overshoot and is not optimal for moving targets.

## 4.2 Proportional Navigation (PN) — canonical

**Idea**: command acceleration perpendicular to line-of-sight (LOS) proportional to LOS rate times closing speed.

**Key steps**:

* LOS vector `λ = p_t - p_i`.
* LOS angle rate `λ_dot` computed from relative motion: `λ_dot = (λ × (v_t - v_i)) / ||λ||^2` (vector form in 3D).
* Closing speed `Vc = - (λ · (v_t - v_i)) / ||λ||` (positive if closing).
* Command lateral acceleration: `a_cmd = N * Vc * λ_dot` where `N` is navigation constant (gain).

**Pseudocode**:

```python
def canonical_pn(p_i, v_i, p_t, v_t, N, amax):
    lam = p_t - p_i
    rel_v = v_t - v_i
    lam_norm = norm(lam)
    if lam_norm < eps: return np.zeros(3)
    lam_dot = np.cross(lam, rel_v) / (lam_norm**2)   # vector form
    Vc = -np.dot(lam, rel_v) / lam_norm
    a_cmd = N * Vc * lam_dot
    a_cmd = clamp_vec(a_cmd, amax)
    return a_cmd
```

**Notes**: PN is classic for guided missiles. It can fail when closing speed is nearly zero or geometry degenerates.

## 4.3 Linearized Proportional Navigation (LPN)

**Idea**: a Cartesian/linearized formulation that yields an acceleration command using position & velocity errors and `t_go` estimate.

**Formula used** (paper / prior literature):

```
a_cmd = G * ( (Δp + Δv * tgo) / tgo^2 )
```

where `Δp = p_t - p_i`, `Δv = v_t - v_i`, `tgo = ||Δp|| / ||Δv||` (safeguarded).

**Pseudocode**:

```python
def lpn(p_i, v_i, p_t, v_t, G_lpn, amax, min_tgo):
    dp = p_t - p_i
    dv = v_t - v_i
    vrel = norm(dv)
    if vrel < 1e-3:
        tgo = max(min_tgo, norm(dp) / (norm(v_i) + 1e-3))
    else:
        tgo = max(min_tgo, norm(dp)/vrel)
    lpn_term = (dp + dv * tgo) / (tgo * tgo)
    a_cmd = G_lpn * lpn_term
    return clamp_vec(a_cmd, amax)
```

**Notes**: more robust than canonical PN in many geometries.

## 4.4 General PN (GPN)

GPN variants introduce modifications to PN to handle special cases (e.g., when closure is negative) and may include additional gain scheduling. Implementation depends on the variant; the general approach is to compute LOS geometry and apply tuned nonlinear gains. Use the LPN or FRPN unless you have a specific GPN law and parameters.

## 4.5 Fast Response Proportional Navigation (FRPN) — detailed (paper's main contribution)

FRPN blends an LPN-like term with a small Pure Pursuit (PP) term to avoid pausing when `||Δv||` is small and to remain aggressive when geometry allows. The paper reports FRPN provides the best trade-off for fast interceptions.

**Formula (paper Eq. 19, adapted)**:

```
a_cmd = G * ( (1 - W) * (Δp + Δv * tgo) / tgo^2  +  W * Δp )
```

where:

* `G` is the global gain (large, e.g., ~20)
* `W` is a small blending weight (e.g., 0.05)
* `tgo` is time-to-go estimate: `||Δp|| / ||Δv||`, safeguarded with `min_tgo`

**Key design details (practical):**

* **Safeguard for small relative velocity**: if `||Δv||` < `v_eps`, use `tgo = max(min_tgo, ||Δp|| / max(||v_i||, v_eps))` to avoid infinite tgo.
* **Scaling by uncertainty**: if your predictor provides covariance `P_pred` with large positional variance, you may scale down `G` proportionally (e.g., `G_eff = G / (1 + alpha * trace(P_pos))`) to be conservative when target uncertainty is high.
* **Axis-wise acceleration limits**: clamp per-axis to `amax` vector.

**Pseudocode (detailed)**:

```python
# Params: G (float), W (float small), min_tgo, v_eps, amax (vector), alpha_uncertainty

def frpn(p_i, v_i, p_t, v_t, P_t=None, G
```


# Continuation of Prediction & Planning Guide (from Section 4.5 onward)

## 4.5 Fast Response PN (FRPN) — detailed (paper's main contribution)

FRPN blends an LPN-like term with a small Pure Pursuit (PP) term to avoid pausing when relative velocity is small and to remain aggressive when geometry allows. This is the recommended guidance method due to its fast response and high interception success.

### Formula (paper Eq. 19, adapted)

```
a_cmd = G * ( (1 - W) * (Δp + Δv * tgo) / tgo^2  +  W * Δp )
```

Where:
- **Δp = p_target – p_interceptor**
- **Δv = v_target – v_interceptor**
- **tgo = ||Δp|| / ||Δv||** (safeguarded)
- **G** is global gain (≈ 20)
- **W** is a small blending weight (≈ 0.05)

### Practical design details
- Use **min_tgo** safeguard to avoid infinite time-to-go.
- If **Δv ≈ 0**, rely more on PP component.
- Clamp output acceleration per-axis using platform limits.
- (Optional) scale G down when prediction covariance is large.

### Pseudocode (detailed)

```python
def frpn(p_i, v_i, p_t, v_t, P_t=None,
         G=19.7, W=5.1e-2, min_tgo=0.05, v_eps=0.1,
         amax=np.array([4,4,2]), alpha_uncert=0.0):

    dp = p_t - p_i
    dv = v_t - v_i
    dp_norm = norm(dp)
    dv_norm = norm(dv)

    # Compute time-to-go robustly:
    if dv_norm < v_eps:
        denom = max(norm(v_i), v_eps)
        tgo = max(min_tgo, dp_norm / denom)
    else:
        tgo = max(min_tgo, dp_norm / dv_norm)

    lpn_term = (dp + dv * tgo) / (tgo * tgo)
    pp_term  = dp

    combined = (1 - W) * lpn_term + W * pp_term

    # Optional uncertainty scaling:
    G_eff = G
    if P_t is not None and alpha_uncert > 0:
        pos_var = np.trace(P_t[0:3, 0:3])
        G_eff = G / (1.0 + alpha_uncert * pos_var)

    a_cmd = G_eff * combined
    return clamp_vec(a_cmd, amax)
```

### Publishing to PX4 (ROS2, no MAVROS)

Convert the acceleration into a short-horizon **velocity** + **position** setpoint and publish it to:

```
/fmu/in/trajectory_setpoint
```

---

## 4.6 Model Predictive Control (MPC) — detailed

MPC solves an optimal control problem over a prediction horizon using the predicted target motion. It is powerful but more computationally expensive.

### System model (discrete)

```
p_{k+1} = p_k + v_k * dt + 0.5 * u_k * dt^2
v_{k+1} = v_k + u_k * dt
```

Where:
- **u_k** = commanded acceleration (control input)
- Use axis-wise bounds: |u_x| ≤ a_max_x, etc.

### Cost function

```
J = Σ_k || p_i(k) - p_t_pred(k) ||_Q^2 + || u(k) ||_R^2
```

### Constraints
- Velocity limits
- Acceleration limits
- Optional collision/safety bounds

### Pseudocode (high-level)

```python
def mpc_solve(x0, p_t_pred, N, dt, vmax, amax, Q, R):
    # Build linearized discrete dynamics A, B
    # Formulate QP/NLP
    # Warm-start solver
    # Solve using ACADOS/OSQP
    return u[0]
```

### MPC Node Loop

```python
while True:
    x0 = read_interceptor_state()
    p_t_pred = predictor.get_predicted_trajectory()

    u0 = mpc_solve(x0, p_t_pred, ...)
    if solver_failed:
        u0 = frpn(...)  # fallback

    publish_to_px4(u0)  # -> /fmu/in/trajectory_setpoint
```

---

# 5) ROS2 Node Architecture & PX4 Topics (no MAVROS)

## Required Nodes
- **predictor_node**
- **guidance_node** (FRPN / PN / LPN / PP)
- **offboard_manager** (publishes `/fmu/in/offboard_control_mode`)
- **mpc_node** (optional)

## Important PX4 ROS2 Topics

### Subscribe
- `/fmu/out/vehicle_odometry`
- `/fmu/out/vehicle_local_position_v1`

### Publish
- High-level: `/fmu/in/trajectory_setpoint`
- Mid-level: `/fmu/in/vehicle_rates_setpoint`, `/fmu/in/vehicle_thrust_setpoint`
- Mode/arming: `/fmu/in/offboard_control_mode`, `/fmu/in/vehicle_command`

---

# 6) Integration & Testing (Simulation)

1. Launch PX4 SITL + ROS2 uORB bridge.
2. Start predictor node → verify predicted target trajectory.
3. Start FRPN guidance node (low gain first).
4. Enable offboard control.
5. Visualize in RViz.
6. Test static → moving target.
7. Tune G, W, amax.
8. Optional: integrate MPC and compare performance.

---

# 7) Tuning Recipes (recommended seeds)

### FRPN
- G ≈ 19.7  
- W ≈ 0.051  
- min_tgo = 0.05  
- amax = [4, 4, 2]

### LPN
- G ≈ 20  
- min_tgo = 0.05–0.1  

### MPC
- horizon: N = 10–20  
- dt = 0.1–0.2  
- Q = diag([10,10,5])  
- R = 0.1 * I  

---

# 8) Appendix: Helper Conversions

### Acceleration → Trajectory Setpoint

```
v_target = v_i + a_cmd * dt_cmd
p_target = p_i + v_i * dt_cmd + 0.5 * a_cmd * dt_cmd^2
```

Publish fields:
- position  
- velocity  
- acceleration  
- yaw / yaw_rate  
- timestamp  
