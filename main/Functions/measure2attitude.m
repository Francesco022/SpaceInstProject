function attitude = measure2attitude(measurements, true_vectors)
%MEASURE2ATTITUDE Compute attitude from sensor measurements and known target vectors
%   attitude is given in euler angles (ZYX sequence, degrees)

    % Define which algorithm to use based on number of measurements
    nMeasures = height(measurements);

    % Convert to points_b and points_r
     % points_b: measured directions in body frame
     % points_r: known target directions in inertial frame

    points_b = [];
    points_r = [];

    for i = 1:nMeasures
        az = deg2rad(measurements(i,1));
        el = deg2rad(measurements(i,2));
        [x_b, y_b, z_b] = sph2cart(az, el, 1);
        points_b = [points_b; [x_b, y_b, z_b]];

        target_vec = true_vectors(i, :);
        target_vec = target_vec / norm(target_vec); % Normalize
        points_r = [points_r; target_vec];
    end

    if nMeasures < 2
        error('At least two measurements are required for attitude determination.');
    elseif nMeasures == 2
        % Use algebraic method for 2 measurements
        A = algebraic_method(points_b', points_r');
        attitude = rotm2eul(A, 'ZYX'); % Extract Euler angles from the rotation matrix
        attitude = rad2deg(attitude);
    else
        % Use QUEST algorithm for more than 2 measurements
        [R, ~, ~] = quest(points_b', points_r');
        attitude = rotm2eul(R, 'ZYX'); % Extract Euler angles from the rotation matrix
        attitude = rad2deg(attitude);
    end

end