close; clear; clc;

%% Initialize Satellite and Sensors
pos0 = [0;0;0];       % Initial position [x;y;z]
eul0 = [0 0 0];       % Initial Euler angles ZYX [deg]
sat = Satellite(pos0,eul0);

% Sensor 1: pointing forward
s1_pos = [0;0;1];        % Position relative to satellite body
s1_eul = [0 0 0];        % Euler angles relative to satellite
sensor1 = Sensor(sat, s1_pos, s1_eul);

% Sensor 2: pointing sideways
s2_pos = [1;0;0];      
s2_eul = [0 90 0];       
sensor2 = Sensor(sat, s2_pos, s2_eul);

% Sensor 3: pointing up
s3_pos = [0;1;0];      
s3_eul = [90 90 0];       
sensor3 = Sensor(sat, s3_pos, s3_eul);


%% Create Figure and Axes
figure(); 
ax = axes; 
axis(ax,'equal'); grid(ax,'on'); view(ax,3);
xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');


%% Initialize Satellite and Sensors Visualization             
sat.initVisual(ax, 1.5);

sensor1.initVisual(ax, 0.5);
sensor2.initVisual(ax, 0.5);
sensor3.initVisual(ax, 0.5);


%% Animation Loop
nSteps = 360;              
for k = 1:nSteps
    % Update satellite
    sat.translate([0;0;0]);  
    sat.rotateByEuler([2 3 1]);    

    % Update satellite visualization
    sat.updateVisual();

    % Update sensors visualization
    sensor1.updateVisual();
    sensor2.updateVisual();
    sensor3.updateVisual();

    drawnow;
    pause(0.01);
end
