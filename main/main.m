close; clear; clc;

addpath('Functions');
addpath('Utils');


%% Initialize all objects

% Orbit
tle = tleread('transat.tle');
orbit = Orbit(tle);

% Satellite
pos0 = [0;0;0];       % Initial position [x;y;z]
eul0 = [0 0 0];       % Initial Euler angles ZYX [deg]
satellite = Satellite(pos0,eul0);

% Sensor 1 (Horizon Sensor)
pos = [0;0;1];    % Sensor position wrt sat body frame [x;y;z]
eul = [0 -90 0];      % Sensor attitude wrt sat body frame ZYX [deg]
fov = [30, 30];          % Field of view [degxdeg]
noise = [0.042, 0.06]./3;      % Sensor noise characteristics [WN, bias] [deg]
sensor1 = Sensor(satellite, pos, eul, fov, noise);
% Sensor 2 (Sun Sensor 1)
pos = [1;0;0];    % Sensor position wrt sat body frame [x;y;z]
eul = [0 0 0];    % Sensor attitude wrt sat body frame ZYX [deg]
fov = [100, 100];          % Field of view [degxdeg]
noise = [0.05, 0]./3;      % Sensor noise characteristics [WN, bias] [deg]
sensor2 = Sensor(satellite, pos, eul, fov, noise);
% Sensor 3 (Sun Sensor 2)
pos = [0;1;0];    % Sensor position wrt sat body frame [x;y;z]
eul = [90 0 0];    % Sensor attitude wrt sat body frame ZYX [deg]
fov = [100, 100];          % Field of view [degxdeg]
noise = [0.05, 0]./3;      % Sensor noise characteristics [WN, bias] [deg]
sensor3 = Sensor(satellite, pos, eul, fov, noise);
% Sensor 4 (Sun Sensor 3)
pos = [-1;0;0];    % Sensor position wrt sat body frame [x;y;z]
eul = [180 0 0];    % Sensor attitude wrt sat body frame ZYX [deg]
fov = [100, 100];          % Field of view [degxdeg]
noise = [0.05, 0]./3;      % Sensor noise characteristics [WN, bias] [deg]
sensor4 = Sensor(satellite, pos, eul, fov, noise);
% Sensor 5 (Sun Sensor 4)
pos = [0;-1;0];    % Sensor position wrt sat body frame [x;y;z]
eul = [-90 0 0];    % Sensor attitude wrt sat body frame ZYX [deg]
fov = [100, 100];          % Field of view [degxdeg]
noise = [0.05, 0]./3;      % Sensor noise characteristics [WN, bias] [deg]
sensor5 = Sensor(satellite, pos, eul, fov, noise);


%% Simulation loop

% Allocate results table for attitude determination Euler angles 'ZYX' [def]
results = table('Size', [0 2], ...
    'VariableTypes',{'double','double'}, ...
    'VariableNames',{'True_Attitude_deg','Measured_Attitude_deg'});

startTime = datetime(2026,3,20,0,0,0);
endTime   = datetime(2026,3,20,1,0,0);
numSteps  = 1000;

timeVector = linspace(startTime, endTime, numSteps)';


for step = 1:numSteps
    currentTime = timeVector(step);
    
    % Update orbit position and attitude
    orbit.update_position_and_attitude(currentTime);
    
    % Get current satellite position and attitude from orbit
    satPos = orbit.get_position();
    satAtt = orbit.get_attitude();

    % Retrive earth and sun vectors (sat inertial pov)
    earthVec = orbit.get_earth_vector();
    sunVec   = orbit.get_sun_vector();
    
    % Update satellite state
    satellite.setPosition(satPos);
    satellite.setAttitude(satAtt);

    satellite.rotateByEuler([0.1*step, 1, 1]);   % For testing purposes only
    
    % Get sensor az el readings in satellite body frame [az, el, flag] [deg]
    sensor1_azel = sensor1.get_measured_azel(earthVec);
    sensor2_azel = sensor2.get_measured_azel(sunVec);
    sensor3_azel = sensor3.get_measured_azel(sunVec);
    sensor4_azel = sensor4.get_measured_azel(sunVec);
    sensor5_azel = sensor5.get_measured_azel(sunVec);

    % Compile all valid measurements
    % NOTE: if more than one sensor sees the same vector (e.g., sun)
    %       QUEST algorithm can not handle it properly
    %       solution is considering them as independent with seme standard deviation take the average
    measure = [NaN, NaN;   % Horizon
               NaN, NaN];  % Sun (mean of all sun sensors)

    true_vector = [earthVec';
                   sunVec' ];

    nsun_measurements = 0;
    sun_sum = [0, 0];

    % Horizon sensor
    if sensor1_azel(3) == 1
        measure(1,:) = sensor1_azel(1:2);
    end

    % Sun sensors
    if sensor2_azel(3) == 1
        sun_sum = sun_sum + sensor2_azel(1:2);
        nsun_measurements = nsun_measurements + 1;
    end
    if sensor3_azel(3) == 1
        sun_sum = sun_sum + sensor3_azel(1:2);
        nsun_measurements = nsun_measurements + 1;
    end
    if sensor4_azel(3) == 1
        sun_sum = sun_sum + sensor4_azel(1:2);
        nsun_measurements = nsun_measurements + 1;
    end
    if sensor5_azel(3) == 1
        sun_sum = sun_sum + sensor5_azel(1:2);
        nsun_measurements = nsun_measurements + 1;
    end

    % Final sun measurement
    if nsun_measurements > 0
        measure(2,:) = sun_sum / nsun_measurements;
    end

    % Attitude determination from sensor measurements
    estimate_attitude = measure2attitude(measure, true_vector);

    % Satellite true attitude in Euler angles ZYX [deg]
    satAtt = satellite.getAttitude();
    satAtt_eul = eulerd(satAtt, 'ZYX', 'frame');

    % Store results in Euler angles 'ZYX' [deg]
    results = [results;
               {satAtt_eul, estimate_attitude}];

end

%% COVARIANCE ANALYSIS OF THE RESULTS

% Calculate attitude errors and wrap them to [-180, 180] [-90, 90] [-180, 180] deg
attitude_errors = results.True_Attitude_deg - results.Measured_Attitude_deg;
attitude_errors(:,1) = wrapTo180(attitude_errors(:,1));
attitude_errors(:,2) = wrapTo180(attitude_errors(:,2));
attitude_errors(:,3) = wrapTo180(attitude_errors(:,3));

% Bias
bias = mean(attitude_errors);

% Root mean square
rms_q = rms(attitude_errors);

% Covariance matrix, eigenvalues and eigenvector
data = attitude_errors;
cov_q = cov(data);
[eigenvec_q, eigenval_q] = eig(cov_q);

max_dev = sqrt(max(diag(eigenval_q)));
fprintf('Maximum standard deviation of attitude errors: %.4f deg\n', max_dev);

% Coverage factor and confidence level for three dimensional measures
mmf3 = [2.1544, 2.3059, 2.5003, 2.7955, 3.3682, 4.0331];
ccl3 = [0.8000, 0.8500, 0.9000, 0.9500, 0.9900, 0.9990];

% Compute the coverage factor for a confidence level of 0.997
conf_val = ppval(spline(ccl3, mmf3), 0.997);
fprintf('Expanded uncertainty (99.7%% confidence level): %.4f\n', conf_val*max_dev);

elli_center = bias;
elli_semiaxes = sqrt(diag(eigenval_q))';
elli_eulang_deg = rad2deg(rotm2eul(eigenvec_q,'ZYX'));
elli = [elli_center elli_semiaxes elli_eulang_deg];


%% DISPLAY RESULTS

% Sactter plot of error distributions
figure(1);
t = 1:height(results);

subplot(3,1,1);
plot(t, attitude_errors(:,1), 'rx');
title('Attitude Determination Errors - Roll (deg)');
grid on;

subplot(3,1,2);
plot(t, attitude_errors(:,2), 'rx');
title('Attitude Determination Errors - Pitch (deg)');
grid on;

subplot(3,1,3);
plot(t, attitude_errors(:,3), 'rx');
title('Attitude Determination Errors - Yaw (deg)');
grid on;

% 3D Error Ellipsoid Plot
figure(2);
plot_err_ellipsoid;
title('3D Attitude Error Ellipsoid');

