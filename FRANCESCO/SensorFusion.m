% Attitude determination via sensor fusion (Kalman-ready setup)
% External sensors  : Sun + Horizon (Euler angles ZYX)
% Internal sensors  : Gyros (angular velocity body frame)
% Computations done in quaternions

close all; clear; clc;
format long;

%% ========================================================================
%  Sampling and time definition
% =========================================================================

N = 1000;          % Number of samples
T = 5400;          % Total time [s]
t = linspace(0,T,N)';   % Time vector [Nx1]


%% ========================================================================
%  External measurements (Euler angles ZYX)
% =========================================================================
% extMeas_e : [psi theta phi] [rad]  (Nx3)
% extMeas   : {time, Euler}          (Nx2 cell)

extMeas_e = zeros(N,3);   % Placeholder (external attitude measurements)

extMeas = cell(N,2);
extMeas(:,1) = num2cell(t);                                   % Time
extMeas(:,2) = mat2cell(extMeas_e, ones(N,1), 3);             % Euler angles


%% ========================================================================
%  Internal measurements (Gyro angular velocity)
% =========================================================================
% intMeas_w : [wz wy wx] [rad/s] (Nx3)
% intMeas   : {time, omega}     (Nx2 cell)

intMeas_w = zeros(N,3);   % Placeholder (gyro measurements)

intMeas = cell(N,2);
intMeas(:,1) = num2cell(t);                                   % Time
intMeas(:,2) = mat2cell(intMeas_w, ones(N,1), 3);             % Angular velocity


%% ========================================================================
%  Initial satellite attitude
% =========================================================================
% Euler angles [Z Y X] (yaw, pitch, roll)

eul_0 = [0 0 0];               % Initial attitude [rad]
q0 = eul2quat(eul_0, 'ZYX');   % Initial quaternion

%% ========================================================================
%  Attitude integration (gyro propagation)
% =========================================================================

% Preallocate output attitude (quaternions)
attitude = cell(N,2);
attitude{1,1} = t(1);
attitude{1,2} = q0;

q = q0;

for k = 2:N

    % Time step
    dt = intMeas{k,1} -intMeas{k-1,1};
    
    % Angular velocities
    w1 = intMeas{k-1,2};   % omega at k-1
    w2 = intMeas{k,2};     % omega at k
    
    % Omega matrices
    Omega1 = [  0    -w1(1) -w1(2) -w1(3);
               w1(1)   0     w1(3) -w1(2);
               w1(2) -w1(3)   0     w1(1);
               w1(3)  w1(2) -w1(1)   0  ];
           
    Omega2 = [  0    -w2(1) -w2(2) -w2(3);
               w2(1)   0     w2(3) -w2(2);
               w2(2) -w2(3)   0     w2(1);
               w2(3)  w2(2) -w2(1)   0  ];
    
    % Quaternion derivatives
    qdot1 = 0.5 * q * Omega1';
    
    % Predictor (Euler step)
    q_pred = q + qdot1 * dt;
    
    % Derivative at predicted step
    qdot2 = 0.5 * q_pred * Omega2';
    
    % Trapezoidal integration
    q = q + 0.5 * (qdot1 + qdot2) * dt;
    
    % Normalize quaternion
    q = q / norm(q);
    
    % Debug
    % disp(norm(q));
    
    % Store
    attitude{k,1} = intMeas{k,1};
    attitude{k,2} = q;
end


%% ========================================================================
%  Plot attitude
% =========================================================================

% Extract time and quaternion history
t_att = cell2mat(attitude(:,1));      % Nx1
q_att = cell2mat(attitude(:,2));      % Nx4

figure;

subplot(4,1,1)
plot(t_att, q_att(:,1), 'LineWidth', 1.2)
grid on
ylabel('q_0')
title('Quaternion propagation (gyro only)')

subplot(4,1,2)
plot(t_att, q_att(:,2), 'LineWidth', 1.2)
grid on
ylabel('q_1')

subplot(4,1,3)
plot(t_att, q_att(:,3), 'LineWidth', 1.2)
grid on
ylabel('q_2')

subplot(4,1,4)
plot(t_att, q_att(:,4), 'LineWidth', 1.2)
grid on
ylabel('q_3')
xlabel('Time [s]')
