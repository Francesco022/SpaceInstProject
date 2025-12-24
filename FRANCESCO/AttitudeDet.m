% Define sensor orientation wrt satellite orientation
% 
% Sensor orientation can be defined as:
%  - Euler angles with a 'ZYX' rotation
%  - rotation matrix
%  - quaternions
%
% To define the angles the Euler angles are used while to perform all other
% types of calcualtions like rotations ecc quaternion are used.

close; clear; clc;
format long;

%% Define Satellite Body Referece Frame

% Euler angles defiend as ZYX rotation
Sat_eul = [0 30 40];        % [deg]
Sat_q = quaternion(deg2rad(Sat_eul), 'euler', 'ZYX', 'frame');

%% Define Sensors Frame wrt Body Frame

% Euler angles defiend as ZYX rotation
S1_body_eul = [0 0 0];    % [deg] horizon
S2_body_eul = [0 90 0];   % [deg] sun 1
S3_body_eul = [90 90 0];   % [deg] sun 2

S1_body_q = quaternion(deg2rad(S1_body_eul), 'euler', 'ZYX', 'frame');
S2_body_q = quaternion(deg2rad(S2_body_eul), 'euler', 'ZYX', 'frame');
S3_body_q = quaternion(deg2rad(S3_body_eul), 'euler', 'ZYX', 'frame');

S_body_q = [S1_body_q, ...
            S2_body_q, ...
            S3_body_q];

% NOTE
% To calculate the actual quaternion in the required reference frame only
% need to mutiply Sat_q*Si_body_q



%% PLOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Define satellite body

% Reference frame
RF = eye(3);

% Satellite body frame
sat_BF = rotmat(Sat_q, 'point');

% Create satellite cylinder
sat_r = 1;              % radius
sat_h = 2;              % height
sat_color = [0 0 1];    % color (rgb)
sat_nSides = 100;       % number of sides

[X,Y,Z] = cylinder(sat_r, sat_nSides);
Z = sat_h*(Z-0.5);

% Rotate satellite body
R = rotmat(Sat_q, 'point');

sat_points1 = [X(1,:); Y(1,:); Z(1,:)];  % 3xN
sat_points2 = [X(2,:); Y(2,:); Z(2,:)];  % 3xN

sat_points1_rot = R * sat_points1;
sat_points2_rot = R * sat_points2;

X(1,:) = sat_points1_rot(1,:);
Y(1,:) = sat_points1_rot(2,:);
Z(1,:) = sat_points1_rot(3,:);

X(2,:) = sat_points2_rot(1,:);
Y(2,:) = sat_points2_rot(2,:);
Z(2,:) = sat_points2_rot(3,:);

% Calculate the actual quaternions for each sensor in the satellite reference frame
sens_sat_q = Sat_q * S_body_q;

%% Create sensor cubes
l = 0.2;  % cube edge
sens_color = [1 0 0];    % color (rgb)
sens_nSides = 4;       % number of sides

[Xc,Yc,Zc] = cylinder(l/sqrt(2), sens_nSides);
Zc = l*(Zc-0.5) + 1;

% Rotate of 45° to alignate with axes
sens_points1 = [Xc(1,:); Yc(1,:); Zc(1,:)];  % 3xN (bottom)
sens_points2 = [Xc(2,:); Yc(2,:); Zc(2,:)];  % 3xN (top)

theta = deg2rad(45);
Rz = [cos(theta) -sin(theta) 0;
      sin(theta)  cos(theta) 0;
      0           0          1];

sens_points1 = Rz * sens_points1;
sens_points2 = Rz * sens_points2;

% Apply Rotation to sensor and their frame
sens_X = cell(1, size(S_body_q, 2));
sens_Y = cell(1, size(S_body_q, 2));
sens_Z = cell(1, size(S_body_q, 2));

sens_frame = cell(1, size(S_body_q, 2));
sens_pos = cell(1, size(S_body_q, 2));

for i = 1:size(S_body_q, 2)
    R = rotmat(sens_sat_q(i), 'point');

    sens_points1_rot = R * sens_points1; 
    sens_points2_rot = R * sens_points2; 

    % Reassign
    Xc(1,:) = sens_points1_rot(1,:);
    Yc(1,:) = sens_points1_rot(2,:);
    Zc(1,:) = sens_points1_rot(3,:);

    Xc(2,:) = sens_points2_rot(1,:);
    Yc(2,:) = sens_points2_rot(2,:);
    Zc(2,:) = sens_points2_rot(3,:);

    % Frame and position
    s_frame = R * eye(3);
    s_pos = R * [0;0;1];

    % Compile cells
    sens_X{i} = Xc;
    sens_Y{i} = Yc;
    sens_Z{i} = Zc;

    sens_frame{i} = s_frame;
    sens_pos{i} = s_pos;
end


%% Plot figure
figure()
hold on
alphaValue = 0.2;

% Plot satellite body
surf(X,Y,Z,'FaceColor',sat_color,'LineStyle','none','FaceAlpha',alphaValue,'HandleVisibility','off');
fill3(X(1,:),Y(1,:),Z(1,:),sat_color,'FaceAlpha',alphaValue,'HandleVisibility','off');
fill3(X(2,:),Y(2,:),Z(2,:),sat_color,'FaceAlpha',alphaValue,'HandleVisibility','off');

% Plot sensors
for i = 1:numel(sens_X)
    Xc = sens_X{i}; Yc = sens_Y{i}; Zc = sens_Z{i};
    surf(Xc,Yc,Zc,'FaceColor',sens_color,'LineStyle','none','FaceAlpha',alphaValue,'HandleVisibility','off');
    fill3(Xc(1,:),Yc(1,:),Zc(1,:),sens_color,'FaceAlpha',alphaValue,'HandleVisibility','off');
    fill3(Xc(2,:),Yc(2,:),Zc(2,:),sens_color,'FaceAlpha',alphaValue,'HandleVisibility','off');
end

% Plot canonical frame (RF)
scale = 2; % length of arrows
qRFx = quiver3(0,0,0,RF(1,1)*scale,RF(2,1)*scale,RF(3,1)*scale,'r','LineWidth',2,'MaxHeadSize',0.5);
qRFy = quiver3(0,0,0,RF(1,2)*scale,RF(2,2)*scale,RF(3,2)*scale,'g','LineWidth',2,'MaxHeadSize',0.5);
qRFz = quiver3(0,0,0,RF(1,3)*scale,RF(2,3)*scale,RF(3,3)*scale,'b','LineWidth',2,'MaxHeadSize',0.5);

% Plot satellite body frame (sat_BF)
qBFx = quiver3(0,0,0,sat_BF(1,1)*scale,sat_BF(2,1)*scale,sat_BF(3,1)*scale,'r--','LineWidth',2,'MaxHeadSize',0.5);
qBFy = quiver3(0,0,0,sat_BF(1,2)*scale,sat_BF(2,2)*scale,sat_BF(3,2)*scale,'g--','LineWidth',2,'MaxHeadSize',0.5);
qBFz = quiver3(0,0,0,sat_BF(1,3)*scale,sat_BF(2,3)*scale,sat_BF(3,3)*scale,'b--','LineWidth',2,'MaxHeadSize',0.5);

% Plot sensors frame frame (sens_F)
scale = 0.5;
firstSens = true;
for i = 1:numel(sens_X)
    frame = scale * sens_frame{i};
    p = frame(:,1); q = frame(:,2); r = frame(:,3);
    pos = sens_pos{i};
    x = pos(1); y = pos(2); z = pos(3);

    hSx = quiver3(x,y,z,p(1),p(2),p(3),'y','LineWidth',2,'MaxHeadSize',0.5);
    hSy = quiver3(x,y,z,q(1),q(2),q(3),'m','LineWidth',2,'MaxHeadSize',0.5);
    hSz = quiver3(x,y,z,r(1),r(2),r(3),'c','LineWidth',2,'MaxHeadSize',0.5);

    if firstSens
        qSx = hSx;
        qSy = hSy;
        qSz = hSz;
        firstSens = false;
    end
end


%% Formatting
axis equal
grid on
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3)
title('Satellite Body and Sensor Cubes with Frames')

% Legend only for the frames
legend([ ...
    qRFx qRFy qRFz ...
    qBFx qBFy qBFz ...
    qSx  qSy  qSz ], ...
    { ...
    'RF X','RF Y','RF Z', ...
    'BF X','BF Y','BF Z', ...
    'Sensor X','Sensor Y','Sensor Z' }, ...
    'Location','northeastoutside');
