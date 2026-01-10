%**************************************************************************
%
% Plot the error ellipsoid about the Euler angles estimation
% Andrea Valmorbida and Anese Giovanni- 22/11/2023
%
% This script plots the error ellipsoid about the estimation of the Euler
% angles.
%
%**************************************************************************

% draw sampling points
drawPoint3d(attitude_errors,'.r');

hold on;
axis equal;

% draw error ellipsoid
elli_3de = [elli_center conf_val*elli_semiaxes elli_eulang_deg];
drawEllipsoidV2(elli_3de,'rotm',eigenvec_q,'FaceAlpha',0.3,'drawEllipses', true);

% draw reference vectors
for i=1:3
    drawVector3d(bias,conf_val*elli_semiaxes(i)*eigenvec_q(:,i)','b','LineWidth',2);
end

xlabel('x [deg]');ylabel('y [deg]');zlabel('z [deg]');
view(30,30);
