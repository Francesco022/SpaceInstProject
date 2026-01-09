clc; close all; clear
epsilon = deg2rad(23.4393);
R_ecl2eci = [1 0 0;
             0 cos(epsilon) sin(epsilon);
             0 -sin(epsilon) cos(epsilon)];
mu = 3.986004418e14;
JD0 = juliandate(datetime(2025,12,24,13,36,0));

r_sun = planetEphemeris(JD0,'Earth','Sun','432t','km')';
r_sun = R_ecl2eci*r_sun;
r_sun = r_sun / norm(r_sun);

alpha_sun = atan2(r_sun(2), r_sun(1));
if alpha_sun < 0
    alpha_sun = alpha_sun + 2*pi;
end

LSTAN = 13.5;  % 10:30 local solar time

Omega = alpha_sun + pi - 2*pi*LSTAN/24;
Omega = mod(Omega,2*pi);

h = 800e3;
a = 6371e3 + h;
e = 0;
i = deg2rad(98.6);     % Sun-synchronous a 800 km       

omega = 0;
theta0 = 0;

elements = struct('a',a,'e',e,'i',i,'Omega',Omega,'omega',omega,'theta',theta0);

T = 2*pi*sqrt(a^3/mu);

tspan = 0:50:10000*T;

figure('Color','k')
axis equal
grid on
hold on
xlabel('X'); ylabel('Y'); zlabel('Z');
plot3(0,0,0,'yo','MarkerSize',10,'MarkerFaceColor','y') % Sole

hEarth = plot3(0,0,0,'bo','MarkerSize',10,'MarkerFaceColor','b');
hSat   = plot3(0,0,0,'ro','MarkerSize',3,'MarkerFaceColor','r');
vecScale = 0.35;  % scala dei vettori rispetto alla lunghezza reale

% vettori dal satellite
hSunV   = quiver3(0,0,0,0,0,0,'g','LineWidth',1.5);
hEarthV = quiver3(0,0,0,0,0,0,'c','LineWidth',1.5);

satTrail = nan(3,length(tspan));
hTrail = plot3(nan,nan,nan,'r');




legend([hEarth hSat hSunV hEarthV], {'Earth','Satellite','Sun vector','Earth vector'}, ...
       'TextColor','w','Location','northeast')

% % view lungo X per mostrare piano Y-Z
% view([1 0 0])

for k=1:length(tspan)

    JD = juliandate(datetime(2025,12,24,13,36,0) + seconds(tspan(k)));

    % Terra nel frame eclittico (polo nord eclittico lungo Z)
    r_earth_eci = planetEphemeris(JD,'Sun','Earth','432t','km')'*1e3;
    r_earth_j2000 = R_ecl2eci  * r_earth_eci;

    % Satellite attorno alla Terra (frame geocentrico locale)
    [r_sat_eci,~] = propagate_keplerian(tspan(k),elements,mu);
    r_sat_ecl = R_ecl2eci * r_sat_eci;

    % Trasformazione in frame eliocentrico
    r_sat = r_earth_j2000 + r_sat_ecl;

    % Vettore dal satellite verso il Sole
    r_sat2sun = -r_sat;
    sun_unit = r_sat2sun / norm(r_sat2sun);

    % Vettore dal satellite verso la Terra
    r_sat2earth = r_earth_j2000 - r_sat;
    earth_unit = r_sat2earth / norm(r_sat2earth);

    % Aggiorna posizione plot
    set(hEarth,'XData',r_earth_j2000(1),'YData',r_earth_j2000(2),'ZData',r_earth_j2000(3))
    set(hSat,  'XData',r_sat(1),'YData',r_sat(2),'ZData',r_sat(3))
        L = 1e7;
set(hSunV, 'XData', r_sat(1), 'YData', r_sat(2), 'ZData', r_sat(3), ...
            'UData', sun_unit(1)*vecScale*L, ...
            'VData', sun_unit(2)*vecScale*L, ...
            'WData', sun_unit(3)*vecScale*L);

set(hEarthV, 'XData', r_sat(1), 'YData', r_sat(2), 'ZData', r_sat(3), ...
              'UData', earth_unit(1)*vecScale*L, ...
              'VData', earth_unit(2)*vecScale*L, ...
              'WData', earth_unit(3)*vecScale*L);

    % Centra visuale sulla Terra

    xlim([r_earth_j2000(1)-L , r_earth_j2000(1)+L])
    ylim([r_earth_j2000(2)-L , r_earth_j2000(2)+L])
    zlim([r_earth_j2000(3)-L , r_earth_j2000(3)+L])

    % Aggiorna scia del satellite
    satTrail(:,k) = r_sat;
    set(hTrail,'XData',satTrail(1,1:k), ...
               'YData',satTrail(2,1:k), ...
               'ZData',satTrail(3,1:k))

    drawnow
    pause(1e-15)
    
end

function [r_actual, v_actual] = propagate_keplerian(dt, elements, mu)
    a = elements.a;
    e = elements.e;
    i = elements.i;
    Omega = elements.Omega;
    omega = elements.omega;
    theta0 = elements.theta;
    

    % ORBITA ELLITTICA
    E0 = 2 * atan2(sqrt(1-e)*sin(theta0/2), sqrt(1+e)*cos(theta0/2));
    M0 = E0 - e*sin(E0);
    n = sqrt(mu/abs(a)^3);
    M_new = M0 + n*dt;
    E_new = solve_kepler_equation(M_new, e);
    theta_new = 2 * atan2(sqrt(1+e)*sin(E_new/2), sqrt(1-e)*cos(E_new/2));
    r_c = a*(1 - e^2)/(1 + e*cos(theta_new));
    p = a*(1 - e^2);
    o_dot_x = -sqrt(mu/p) * sin(theta_new);
    o_dot_y =  sqrt(mu/p) * (e + cos(theta_new));

    % Posizione nel frame orbitale
    o_x = r_c * cos(theta_new);
    o_y = r_c * sin(theta_new);
    o_z = 0;
    o_dot_z = 0;
    
    % Trasforma al frame inerziale
    cos_Om = cos(Omega); sin_Om = sin(Omega);
    cos_om = cos(omega); sin_om = sin(omega);
    cos_i = cos(i);
    
    R11 = cos_om*cos_Om - sin_om*cos_i*sin_Om;
    R12 = -sin_om*cos_Om - cos_om*cos_i*sin_Om;
    R21 = cos_om*sin_Om + sin_om*cos_i*cos_Om;
    R22 = -sin_om*sin_Om + cos_om*cos_i*cos_Om;
    R31 = sin_om*sin(i);
    R32 = cos_om*sin(i);
    
    r_actual = [R11*o_x + R12*o_y;
             R21*o_x + R22*o_y;
             R31*o_x + R32*o_y];
    
    v_actual = [R11*o_dot_x + R12*o_dot_y;
             R21*o_dot_x + R22*o_dot_y;
             R31*o_dot_x + R32*o_dot_y];
end

function E = solve_kepler_equation(M, e)
    M = mod(M, 2*pi);
    E = M;
    
    for iter = 1:50
        f = E - e*sin(E) - M;
        fp = 1 - e*cos(E);
        E_new = E - f/fp;
        
        if abs(E_new - E) < 1e-12
            E = E_new;
            return;
        end
        E = E_new;
    end
end 