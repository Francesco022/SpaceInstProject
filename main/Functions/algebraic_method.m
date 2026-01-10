%**************************************************************************
%
% Algebraic method for the three axis determination
% Andrea Valmorbida and Anese Giovanni- 22/11/2023
%
% This function computes the rotation matrix that determines the attitude
% of the satellite w.r.t. a reference frame by using two measures.
% 
% A = algebraic_method(target_b, target_r)
% INPUT
% target_b - 3X2 matrix with the target points coordinates in body frame
% target_r - 3X2 matrix with the target points coordinates in inertial frame
%
% OUTPUT 
% A - attitude matrix (rotation from body to inertial)
% 
%**************************************************************************
function A = algebraic_method(target_b, target_r)
    % control
    [r1,c1] = size(target_b);
    [r2,c2] = size(target_b);
    if(c1~=c2)
      error('Point sets must be of the same size');
    end
    
    if(r1~=3 || r2~=3)
      error('Supplied points should be of dimension three');
    end
    
    if(c1~=2)
      error('Two point pairs needed');
    end

    % define two vectors in body frame
    u_b = target_b(:,1);
    v_b = target_b(:,2);
    % define two vectors in inertial frame
    u_r = target_r(:,1);
    v_r = target_r(:,2);

    % Body matrix
    q_b = u_b;
    r_b = cross(u_b,v_b) ./ norm( cross(u_b,v_b) );
    s_b = cross(q_b,r_b);
    MB = [q_b, r_b, s_b];

    % Inertial matrix
    q_r = u_r;
    r_r = cross(u_r,v_r) ./ norm( cross(u_r,v_r) );
    s_r = cross(q_r,r_r);
    MR = [q_r, r_r, s_r];

    % Attitude Matrix 
    A = MB * MR'; % from inertial to body
    A = A';       % from body to inertial

end