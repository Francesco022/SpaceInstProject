% Attitude determination via sensor fusion (Kalman-ready setup)
% External sensors  : Sun + Horizon (Euler angles ZYX)
% Internal sensors  : Gyros (angular velocity body frame)
% Computations done in quaternions.

% ========================================================================
% Multi-rate MEKF Attitude Determination with Gyro Bias
% ========================================================================

close all; clear; clc;
format long;

rng(1);   % reproducibility

%% ========================================================================
% Sampling parameters
% ========================================================================

f_gyro = 100;      % [Hz]
f_ext  = 1;        % [Hz]

dt = 1/f_gyro;
T  = 6000;          % [s]
N  = T*f_gyro;

t = (0:N-1)'*dt;
ext_step = f_gyro / f_ext;

%% ========================================================================
% TRUE dynamics
% ========================================================================

w_true = [1e-6 0 0];           % true angular rate [rad/s]
b_true = [5e-7 -3e-7 2e-7];    % true gyro bias [rad/s]

%% ========================================================================
% Sensor noise
% ========================================================================

sigma_g    = 2.62e-4;     % gyro noise [rad/s]
sigma_b    = 2e-6;     % bias RW (filter)
sigma_meas = 1e-3;     % external attitude noise [rad]

%% ========================================================================
% Simulated measurements
% ========================================================================

% Gyro measurements
intMeas_w = repmat(w_true, N, 1) ...
            + repmat(b_true, N, 1) ...
            + sigma_g*randn(N,3);

% External attitude (Euler ZYX)
extMeas_e = zeros(N,3);
for k = 2:N
    extMeas_e(k,:) = extMeas_e(k-1,:) + w_true*dt;
end
extMeas_e = extMeas_e + sigma_meas*randn(N,3);

%% ========================================================================
% Initial attitude
% ========================================================================

q = eul2quat([0 0 0],'ZYX');

%% ========================================================================
% MEKF initialization
% ========================================================================

x = zeros(6,1);    % [delta_theta; bias]
P = blkdiag(1e-6*eye(3), 1e-8*eye(3));

Q = diag([sigma_g^2*ones(1,3), sigma_b^2*ones(1,3)]);
R = sigma_meas^2 * eye(3);

%% ========================================================================
% Storage
% ========================================================================

attitude = zeros(N,4);
bias_est = zeros(N,3);

attitude(1,:) = q;
bias_est(1,:) = x(4:6)';

%% ========================================================================
% Main loop
% ========================================================================

for k = 2:N

    %% Bias-corrected gyro
    w_meas = intMeas_w(k,:)';
    b_hat  = x(4:6);
    w = w_meas - b_hat;

    %% Quaternion propagation
    Omega = [  0    -w(1) -w(2) -w(3);
              w(1)   0     w(3) -w(2);
              w(2) -w(3)   0     w(1);
              w(3)  w(2) -w(1)   0  ];

    qdot = 0.5 * q * Omega';
    q = q + qdot * dt;
    q = quatnormalize(q);

    %% MEKF propagation
    F = [ -skew(w)   -eye(3);
           zeros(3)   zeros(3) ];

    Phi = eye(6) + F*dt;

    x = Phi*x;
    P = Phi*P*Phi' + Q*dt;

    %% Measurement update (low-rate)
    if mod(k, ext_step) == 0

        q_meas = eul2quat(extMeas_e(k,:),'ZYX');
        q_err  = quatmultiply(q_meas, quatinv(q));
        z = 2*q_err(2:4)';

        H = [eye(3) zeros(3)];

        K = P*H'/(H*P*H' + R);
        x = x + K*(z - H*x);
        P = (eye(6) - K*H)*P;

        %% Quaternion correction
        delta_q = [1;
                   0.5*x(1);
                   0.5*x(2);
                   0.5*x(3)]';

        q = quatmultiply(q, delta_q);
        q = quatnormalize(q);

        x(1:3) = 0;
    end

    attitude(k,:) = q;
    bias_est(k,:) = x(4:6)';
end

%% ========================================================================
% ============================= PLOTS =====================================
% ========================================================================

% Quaternion
figure;
plot(t, attitude, 'LineWidth',1.1)
grid on
title('Estimated Quaternion')
legend('q_0','q_1','q_2','q_3')

% Euler angles
eul = quat2eul(attitude,'ZYX');
figure;
plot(t, rad2deg(eul),'LineWidth',1.2)
grid on
title('Estimated Euler Angles [deg]')
legend('\psi','\theta','\phi')

% Bias estimation
figure;
plot(t, bias_est,'LineWidth',1.2)
hold on
yline(b_true(1),'--'), yline(b_true(2),'--'), yline(b_true(3),'--')
grid on
title('Gyro Bias Estimation')
legend('b_x','b_y','b_z','true b_x','true b_y','true b_z')

%% ========================================================================
% Utility
% ========================================================================

function S = skew(w)
S = [  0   -w(3)  w(2);
      w(3)   0   -w((1));
     -w(2)  w(1)   0  ];
end


%% REFERENCES TRACK:
%   - Quaternion kinematics for the error-state KF
%   - Kalman Filtering for Attitude Estimation with Quaternions and Concepts from Manifold Theory
