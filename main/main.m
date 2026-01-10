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
numSteps = 1000;

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
    
    % Get sensor az el readings in satellite body frame [az, el, flag] [deg]
    sensor1_azel = sensor1.get_measured_azel(earthVec);
    sensor2_azel = sensor2.get_measured_azel(sunVec);
    sensor3_azel = sensor3.get_measured_azel(sunVec);
    sensor4_azel = sensor4.get_measured_azel(sunVec);
    sensor5_azel = sensor5.get_measured_azel(sunVec);

    % Compile all valid measurements
    measure = [];
    true_vector = [];

    if sensor1_azel(3) == 1  % Horizon sensor valid
        measure = [measure; sensor1_azel(1:2)];
        true_vector = [true_vector; earthVec'];
    end
    if sensor2_azel(3) == 1  % Sun sensor 1 valid
        measure = [measure; sensor2_azel(1:2)];
        true_vector = [true_vector; sunVec'];
    end
    if sensor3_azel(3) == 1  % Sun sensor 2 valid
        measure = [measure; sensor3_azel(1:2)];
        true_vector = [true_vector; sunVec'];
    end
    if sensor4_azel(3) == 1  % Sun sensor 3 valid
        measure = [measure; sensor4_azel(1:2)];
        true_vector = [true_vector; sunVec'];
    end
    if sensor5_azel(3) == 1  % Sun sensor 4 valid
        measure = [measure; sensor5_azel(1:2)];
        true_vector = [true_vector; sunVec'];
    end

    % Attitude determination from sensor measurements
    estimate_attitude = measure2attitude(measure, true_vector);

    % Store results in Euler angles 'ZYX' [deg]
    satAtt_eul = eulerd(satAtt, 'ZYX', 'frame');
    results = [results;
               {satAtt_eul, estimate_attitude}];

end

%% COVARIANCE ANALYSIS OF THE RESULTS

% Calculate attitude errors
attitude_errors = results.True_Attitude_deg - results.Measured_Attitude_deg;

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


%{
% (Optional) Display or log the sensor readings
fprintf('Time: %s\n', datestr(currentTime));
fprintf('  Sensor 1 (Horizon) Az/El: [%.2f, %.2f] deg\n', sensor1_azel(1), sensor1_azel(2));
fprintf('  Sensor 2 (Sun 1) Az/El: [%.2f, %.2f] deg\n', sensor2_azel(1), sensor2_azel(2));
fprintf('  Sensor 3 (Sun 2) Az/El: [%.2f, %.2f] deg\n', sensor3_azel(1), sensor3_azel(2));
fprintf('  Sensor 4 (Sun 3) Az/El: [%.2f, %.2f] deg\n', sensor4_azel(1), sensor4_azel(2));
fprintf('  Sensor 5 (Sun 4) Az/El: [%.2f, %.2f] deg\n', sensor5_azel(1), sensor5_azel(2));
fprintf('\n');
fprintf('\n');


%% Create Figure and Axes
figure(); 
ax = axes; 
axis(ax,'equal'); grid(ax,'on'); view(ax,3);
xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');


%% Initialize Satellite and Sensors Visualization             
satellite.initVisual(ax, 1.5);

sensor1.initVisual(ax, 0.5);
sensor2.initVisual(ax, 0.5);
sensor3.initVisual(ax, 0.5);
sensor4.initVisual(ax, 0.5);
sensor5.initVisual(ax, 0.5);

% Debug simulation
simTime = datetime(2026,3,20,0,0,0);
orbit.update_position_and_attitude(simTime);

sat_pos = orbit.get_position();
sat_att = orbit.get_attitude();

sat_att  = compact(sat_att);

fprintf('Satellite Position (km): [%.2f, %.2f, %.2f]\n', sat_pos(1), sat_pos(2), sat_pos(3));
fprintf('Satellite Attitude (quaternion): [%.4f, %.4f, %.4f, %.4f]\n', sat_att(1), sat_att(2), sat_att(3), sat_att(4));
%}