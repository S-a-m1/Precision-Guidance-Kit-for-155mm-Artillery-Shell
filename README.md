# Precision-Guidance-Kit-for-155mm-Artillery-Shell

# 155mm Guidance/EKF Sim — MATLAB Port

Native MATLAB port of the Python GNC pipeline (EKF nav + proportional-navigation
guidance + PID canard autopilot + a simplified ground-truth physics engine).
No MATLAB toolboxes required — pure `.m` classdef/function files.

## Files
- `RocketEKF.m` — 9-state EKF (position, velocity, roll/pitch/yaw). Ported from `ekf.py`.
- `LaunchFrame.m` — lat/lon/alt → local ENU meters. Ported from `geo.py`.
- `GuidanceLaw.m` — True Proportional Navigation. Ported from `guidance_law.py`.
- `CanardAutopilotPID.m` — PID → canard deflections + simple aero model. Ported from `pid_canard.py`.
- `run_hackathon_simulation.m` — the full closed-loop sim + plots. Ported from `Integrated_GNC.py`.
- `pre_fire_cli.m` — interactive lat/lon/alt prompt, then runs the sim. Ported from `pre_fire_cli.py`.

## Quick start
```matlab
run_hackathon_simulation()   % runs with built-in default pad/target coords
```
or, for your own coordinates:
```matlab
run_hackathon_simulation(34.0, -117.0, 300, 34.018, -116.982, 300)
```
or interactively:
```matlab
pre_fire_cli()
```

## Bugs fixed vs. the original Python
1. **Shell mass mismatch.** `pid_canard.py` used `mass=48.0` while
   `Integrated_GNC.py`'s ground-truth physics used `mass=43.2` — two different
   masses for the same shell. `CanardAutopilotPID.m` now owns `mass`,
   `surface_area`, and `Cl_delta` as single properties, and
   `run_hackathon_simulation.m` reuses them for drag too, so there's one
   source of truth. Defaulted to 43.2 kg — **replace with your actual
   shell/projectile mass**.
2. **Crash on direct run.** `Integrated_GNC.py`'s `if __name__` block called
   `run_hackathon_simulation()` with zero arguments even though the function
   requires 6. `run_hackathon_simulation.m` now has real default coordinates
   so `run_hackathon_simulation()` just works.
3. Left out `main_matlab.py` / `main_matlab_2.py` from this port — those are a
   different subsystem (a UDP bridge for hardware-in-the-loop style testing).
   Say the word if you need that bridge built out in MATLAB.

## Changes in this revision (EKF completion pass)

### RocketEKF.m
- **Q process noise replaced with ADIS16488-derived values** (was round
  placeholder numbers). Position: 1e-4 m²/s; Velocity: 3e-6 (m/s²)²/Hz
  (ADIS16488 baseline × 50 for boost-phase vibration); Orientation: 4.3e-8
  rad²/s (ADIS16488 baseline × 10 for rapid heading changes). The
  ADIS16488 continuous-time PSD derivations are documented inline.
- **Sensor fusion policy documented** in the class header: IMU (predict every
  step), GPS (10 Hz), Baro (20 Hz), Mag (50 Hz).
- **Radar decision noted**: radar/proximity sensor is deliberately excluded
  from EKF fusion — it is fuze/payload logic only (standoff detonation trigger),
  not a navigation sensor.

### run_hackathon_simulation.m
- **Barometer correction added** (~20 Hz, every 5 steps). Simulated noise:
  σ = 0.5 m (MS5611 in-flight estimate). R = 0.25 m² passed explicitly.
- **Magnetometer yaw correction added** (~50 Hz, every 2 steps). Simulated
  noise: σ = 0.035 rad ≈ 2° (ADIS16488 integrated mag, benign environment).
  R = 0.0012 rad² passed explicitly.
- **EKF position error history added** (`pos_error_hist`): records
  `norm(ekf_pos - true_pos)` at every step.
- **Third subplot added** — "EKF Position Error (Estimated vs. True)" —
  shows the error magnitude over time. This is the primary EKF validation
  artifact (convergence rate, steady-state noise floor). Figure widened from
  1400 → 1800 px.

## Known simplifications / still-open issues
These won't stop it from running, but are worth noting for future improvement:

- **Q is not phase-scheduled.** The new ADIS16488-derived Q values are set
  at the boost-phase scale for the whole flight. Scheduling Q_vel back to
  6.0e-8 and Q_ori back to 4.3e-9 after motor burnout (coast/descent) would
  let the EKF trust the IMU more during the quieter phases and reduce noise
  on the error plot.
- **EKF covariance Jacobian is identity-ish** (a reasonable approximation at
  100 Hz; the full linearized F is the next accuracy upgrade).
- **Mag updates during motor firing.** High-EMI near motor exhaust can
  inflate magnetometer noise by 5-10×. Consider gating `update_mag_yaw`
  during boost (e.g. if `t < t_burnout`) and raising R accordingly.
- **Constant drag coefficient** (`cd = 0.25`), no Mach-number dependence.
  A Cd(Mach) lookup table is the single biggest realism gap for supersonic
  flight.
- **No spin/Magnus effect.** Relevant for a spin-stabilised body with
  course-correcting canards.
- **Flat-earth ENU approximation** in `LaunchFrame` — fine at these ranges.

## Requirements
Base MATLAB only (tested logic against R2021a+ syntax). No Simulink, no
toolboxes required — Gaussian noise uses plain `randn`, not `normrnd`.
