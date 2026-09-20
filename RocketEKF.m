classdef RocketEKF < handle
    % RocketEKF  Extended Kalman Filter for a boosted, tumbling shell/rocket body.
    %
    % State vector (9 states), stored as a 9x1 column:
    %   x = [px; py; pz;          % position, meters, local ENU frame
    %        vx; vy; vz;          % velocity, m/s
    %        roll; pitch; yaw]    % orientation, radians
    %
    % Angular rates are NOT tracked as separate states -- raw gyro
    % readings are integrated straight into orientation each tick.
    %
    % Prediction runs every IMU tick (fast, ~50-100 Hz).
    % Correction runs whenever a sensor produces new data (GPS / baro / mag).
    %
    % Ported from ekf.py (Python). Same equations, same structure.
    %
    % Sensor fusion policy:
    %   IMU   -- every step   (predict)
    %   GPS   -- ~10 Hz       (update_gps)
    %   Baro  -- ~20 Hz       (update_baro)    <-- called in run_hackathon_simulation
    %   Mag   -- ~50 Hz       (update_mag_yaw) <-- called in run_hackathon_simulation
    %
    % Note: A radar/proximity sensor is NOT fused in this EKF.
    % Per design decision, radar is used only for fuze/payload logic
    % (standoff detonation trigger), not for navigation state estimation.

    properties
        dt
        n = 9
        x            % 9x1 state
        P            % 9x9 covariance
        Q            % 9x9 process noise
        g = [0; 0; -9.80665];   % gravity, world frame (Z-up ENU)
    end

    methods
        function obj = RocketEKF(dt)
            if nargin < 1
                dt = 0.02;
            end
            obj.dt = dt;
            obj.x = zeros(obj.n, 1);
            obj.P = eye(obj.n) * 10.0;

            % Process noise (continuous-time spectral density Q_c).
            % Covariance update: P <- F*P*F' + Q_c*dt  (see predict()).
            %
            % Values derived from the ADIS16488 IMU datasheet:
            %
            %   Accel velocity random walk : 0.025 mg/sqrt(Hz)
            %                             = 2.45e-4 m/s^2/sqrt(Hz)
            %       -> Q_c_vel (quiet-flight baseline) = sigma_a^2 = 6.0e-8 (m/s^2)^2/Hz
            %   A factor of 50 is applied for the high-vibration boost phase
            %   (structural vibration + unmodelled aerodynamic forces):
            %       -> Q_c_vel_boost = 3.0e-6 (m/s^2)^2/Hz
            %   FUTURE: schedule Q_vel back to 6.0e-8 after motor burnout.
            %
            %   Gyro angular random walk   : 0.226 deg/sqrt(hr)
            %                             = 6.58e-5 rad/sqrt(s)
            %       -> Q_c_ori (ADIS16488 baseline) = sigma_g^2 = 4.3e-9 rad^2/s
            %   A factor of 10 applied for high-rate heading changes during boost:
            %       -> Q_c_ori_boost = 4.3e-8 rad^2/s
            %   FUTURE: schedule back to 4.3e-9 on coast/descent.
            %
            %   Position: no independent position noise source -- errors
            %   accumulate from velocity integration only. A small conservative
            %   value (1e-4 m^2/s) is retained for numerical health; 10 Hz GPS
            %   corrections bound any drift regardless.
            obj.Q = diag([ ...
                1.0e-4, 1.0e-4, 1.0e-4, ...    % position  (m^2/s)         -- conservative baseline
                3.0e-6, 3.0e-6, 3.0e-6, ...    % velocity  ((m/s^2)^2/Hz)  -- ADIS16488 x50 boost phase
                4.3e-8, 4.3e-8, 4.3e-8]);      % orientation (rad^2/s)     -- ADIS16488 x10 boost phase
        end

        function predict(obj, accel_body, gyro_body, dt)
            % accel_body, gyro_body: 1x3 or 3x1, body frame
            if nargin < 4 || isempty(dt)
                dt = obj.dt;
            end
            accel_body = accel_body(:);
            gyro_body = gyro_body(:);

            roll = obj.x(7); pitch = obj.x(8); yaw = obj.x(9);

            % --- integrate orientation from gyro ---
            obj.x(7) = roll + gyro_body(1) * dt;
            obj.x(8) = pitch + gyro_body(2) * dt;
            obj.x(9) = yaw + gyro_body(3) * dt;

            % --- rotate body-frame accel into world frame, remove gravity ---
            R = RocketEKF.rotation_matrix(roll, pitch, yaw);
            a_world = R * accel_body + obj.g;

            % --- integrate velocity & position ---
            obj.x(4:6) = obj.x(4:6) + a_world * dt;
            obj.x(1:3) = obj.x(1:3) + obj.x(4:6) * dt;

            % --- covariance propagation (linearized; identity-ish Jacobian,
            %     a reasonable approximation at high update rates) ---
            F = eye(obj.n);
            F(1:3, 4:6) = eye(3) * dt;
            obj.P = F * obj.P * F' + obj.Q * dt;
        end

        function update_gps(obj, pos_xyz, vel_xyz, R_diag)
            % pos_xyz already converted from lat/lon/alt to local meters
            % (see LaunchFrame.to_enu).
            if nargin < 4 || isempty(R_diag)
                R_diag = [4.0, 4.0, 8.0, 1.0, 1.0, 1.0];
            end
            z = [pos_xyz(:); vel_xyz(:)];
            H = zeros(6, obj.n);
            H(1:3, 1:3) = eye(3);
            H(4:6, 4:6) = eye(3);
            R = diag(R_diag);
            obj.correct(z, H, R);
        end

        function update_baro(obj, altitude, R_val)
            if nargin < 3 || isempty(R_val)
                R_val = 1.0;
            end
            z = altitude;
            H = zeros(1, obj.n);
            H(1, 3) = 1.0;  % pz
            R = R_val;
            obj.correct(z, H, R);
        end

        function update_mag_yaw(obj, yaw_measured, R_val)
            if nargin < 3 || isempty(R_val)
                R_val = 0.1;
            end
            z = yaw_measured;
            H = zeros(1, obj.n);
            H(1, 9) = 1.0;  % yaw
            R = R_val;
            obj.correct(z, H, R);
        end

        function correct(obj, z, H, R)
            hx = H * obj.x;
            y = z(:) - hx;
            S = H * obj.P * H' + R;
            K = obj.P * H' / S;
            obj.x = obj.x + K * y;
            obj.P = (eye(obj.n) - K * H) * obj.P;
        end

        function pos = get_position(obj)
            pos = obj.x(1:3)';
        end

        function vel = get_velocity(obj)
            vel = obj.x(4:6)';
        end

        function ori = get_orientation(obj)
            ori = obj.x(7:9)';
        end
    end

    methods (Static)
        function R = rotation_matrix(roll, pitch, yaw)
            % Body -> world (ZYX Euler convention).
            cr = cos(roll); sr = sin(roll);
            cp = cos(pitch); sp = sin(pitch);
            cy = cos(yaw); sy = sin(yaw);
            Rz = [cy, -sy, 0; sy, cy, 0; 0, 0, 1];
            Ry = [cp, 0, sp; 0, 1, 0; -sp, 0, cp];
            Rx = [1, 0, 0; 0, cr, -sr; 0, sr, cr];
            R = Rz * Ry * Rx;
        end
    end
end
