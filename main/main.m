close; clear; clc;

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
fov = [200, 200];          % Field of view [degxdeg]
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
eul = [0 0 0];    % Sensor attitude wrt sat body frame ZYX [deg]
fov = [100, 100];          % Field of view [degxdeg]
noise = [0.05, 0]./3;      % Sensor noise characteristics [WN, bias] [deg]
sensor3 = Sensor(satellite, pos, eul, fov, noise);
% Sensor 4 (Sun Sensor 3)
pos = [-1;0;0];    % Sensor position wrt sat body frame [x;y;z]
eul = [0 0 0];    % Sensor attitude wrt sat body frame ZYX [deg]
fov = [100, 100];          % Field of view [degxdeg]
noise = [0.05, 0]./3;      % Sensor noise characteristics [WN, bias] [deg]
sensor4 = Sensor(satellite, pos, eul, fov, noise);
% Sensor 5 (Sun Sensor 4)
pos = [0;-1;0];    % Sensor position wrt sat body frame [x;y;z]
eul = [0 0 0];    % Sensor attitude wrt sat body frame ZYX [deg]
fov = [100, 100];          % Field of view [degxdeg]
noise = [0.05, 0]./3;      % Sensor noise characteristics [WN, bias] [deg]
sensor5 = Sensor(satellite, pos, eul, fov, noise);


%% Simulation loop
startTime = datetime(2026,3,20,0,0,0);
endTime   = datetime(2026,3,20,1,0,0);
timeStep  = seconds(1);

timeVector = linspace(startTime, endTime, 100);
numSteps   = length(timeVector);

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
    
    % Get sensor az el readings in satellite body frame
    sensor1_azel = sensor1.get_measured_azel(earthVec);
    sensor2_azel = sensor2.get_measured_azel(sunVec);
    sensor3_azel = sensor3.get_measured_azel(sunVec);
    sensor4_azel = sensor4.get_measured_azel(sunVec);
    sensor5_azel = sensor5.get_measured_azel(sunVec);
    
    % (Optional) Display or log the sensor readings
    fprintf('Time: %s | Sensor1 AzEl: [%.2f, %.2f] | Sensor2 AzEl: [%.2f, %.2f]\n', ...
        datestr(currentTime), sensor1_azel(1), sensor1_azel(2), ...
        sensor2_azel(1), sensor2_azel(2));
end


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


%% Animation Loop
nSteps = 360;              
for k = 1:nSteps 

    % Update satellite visualization
    satellite.updateVisual();

    % Update sensors visualization
    sensor1.updateVisual();
    sensor2.updateVisual();
    sensor3.updateVisual();
    sensor4.updateVisual();
    sensor5.updateVisual();

    drawnow;
    pause(0.01);
end



%{
% Debug simulation
simTime = datetime(2026,3,20,0,0,0);
orbit.update_position_and_attitude(simTime);

sat_pos = orbit.get_position();
sat_att = orbit.get_attitude();

sat_att  = compact(sat_att);

fprintf('Satellite Position (km): [%.2f, %.2f, %.2f]\n', sat_pos(1), sat_pos(2), sat_pos(3));
fprintf('Satellite Attitude (quaternion): [%.4f, %.4f, %.4f, %.4f]\n', sat_att(1), sat_att(2), sat_att(3), sat_att(4));
%}