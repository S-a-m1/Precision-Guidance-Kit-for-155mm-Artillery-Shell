# Precision-Guidance-Kit-for-155mm-Artillery-Shell

# 155mm Guidance/EKF Simulation — MATLAB Port

Native MATLAB port of the Python GNC pipeline for a 155mm precision-guided shell[cite: 6]. This simulation includes an Extended Kalman Filter (EKF) for navigation, True Proportional Navigation (TPN) for guidance, a PID canard autopilot, and a simplified ground-truth physics engine[cite: 6]. No MATLAB toolboxes are required—it runs on pure `.m` classdef/function files using base MATLAB[cite: 6].

## File Structure
*   `RocketEKF.m`: 9-state EKF (position, velocity, roll/pitch/yaw) that fuses IMU, GPS, Barometer, and Magnetometer data[cite: 6, 7].
*   `LaunchFrame.m`: Coordinate converter from GPS lat/lon/alt to local East-North-Up (ENU) meters[cite: 4, 6].
*   `GuidanceLaw.m`: True Proportional Navigation (TPN) steering logic[cite: 3, 6].
*   `CanardAutopilotPID.m`: PID controller mapping commanded acceleration into 4 individual canard deflections (limited to a max 15-degree deflection)[cite: 2, 6].
*   `run_hackathon_simulation.m`: The main physics loop tying the environment, sensors, EKF, and flight computer together. Generates the 3D trajectory and performance plots[cite: 6, 8].
*   `pre_fire_cli.m`: An interactive command-line interface to enter launch and target coordinates before running the sim[cite: 5, 6].

## How to Run
1. Place all six `.m` files into a single folder on your computer.
2. Open MATLAB and set that folder as your **Current Folder**.
3. In the Command Window, run the simulation in one of three ways[cite: 6]:
    *   **Default Run:** `run_hackathon_simulation()` (uses built-in test coordinates).
    *   **Custom Run:** `run_hackathon_simulation(34.0, -117.0, 300, 34.018, -116.982, 300)`
    *   **Interactive Mode:** `pre_fire_cli()`

## Features & Sensor Integrations
*   **Multi-Sensor EKF Fusion:** The EKF correctly predicts via 100 Hz IMU data and corrects using 10 Hz GPS, 20 Hz Barometer (altitude), and 50 Hz Magnetometer (yaw) readings[cite: 7, 8].
*   **Realistic Process Noise:** The EKF covariance matrices (`Q`) use values explicitly derived from the ADIS16488 IMU datasheet, correctly scaled for the high-vibration boost phase[cite: 6, 7].
*   **Performance Tracking:** The simulation plots a dedicated "EKF Position Error" graph to validate filter convergence and accuracy[cite: 6, 8].
*   **Unified Shell Properties:** Mass (43.2 kg), surface area, and aerodynamic coefficients are centralized in the Autopilot class to prevent mismatch between the flight computer and reality[cite: 2, 6].

## Important Simulation Adjustments (For a Successful Strike)
To ensure the projectile actually hits the target rather than missing due to kinematic limits or flying underground, ensure the following updates are applied to `run_hackathon_simulation.m`:
1.  **Ground Collision Detection:** Add a check `if true_pos(3) < 0.0` to break the simulation loop and log a miss when the shell hits the ground.
2.  **Trajectory Alignment:** Update the initial launch velocity (`true_vel`) to point roughly in the direction of the target coordinates so the canards do not saturate trying to make a physically impossible cross-range turn.

## Known Limitations (Future Work)
*   **Constant Drag Coefficient:** Drag is currently locked at `cd = 0.25`[cite: 8]. For supersonic 155mm flight, implementing a Mach-dependent drag lookup table is the highest-priority realism upgrade[cite: 6].
*   **No Spin/Magnus Effect:** The physics engine does not yet simulate the spin of a rifled shell[cite: 6].
*   **Static EKF Process Noise:** IMU noise scales are locked at high-vibration "boost phase" levels[cite: 7]. Phase-scheduling `Q` to drop back to baseline levels during coast/descent would improve steady-state tracking[cite: 6].
