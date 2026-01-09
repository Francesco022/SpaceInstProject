classdef Orbit < handle
    % Orbit class to handle orbital mechanics calculations
    % Input the tle of the satellite to initialize the object
    % OTPUTS:
    %   given utc time return position and attitude of satellite (nadir pointing)

    properties
        tle % Two-line element set for the satellite

        pos % Position of the satellite
        att % Attitude of the satellite
        time % Current time

    end
    
    methods
        % CREATION
        function obj = Orbit(tle)
            % Constructor for Orbit class
            obj.tle = tle;
        end
        
        % UPDATE METHODS
        function update_position_and_attitude(obj, utc_time)
            % Calculate the satellite position and attitude at a given time
            [pos,vel] = propagateOrbit(utc_time,obj.tle);

            % Nadir pointing attitude calculation
            r = pos;
            v = vel;
            z_body = -r / norm(r); % Nadir direction
            y_body = -cross(r, v);
            y_body = y_body / norm(y_body);
            x_body = cross(y_body, z_body);

            R = [x_body, y_body, z_body]'; % Rotation matrix from body to ECI
            att = quaternion(R, 'rotmat', 'frame'); %#ok<*PROPLC> % Convert to quaternion

            % Update properties
            obj.pos = pos;
            obj.att = att;
            obj.time = utc_time;
        end

        % RETURN METHODS
        function pos = get_position(obj)
            % Return the current position of the satellite
            pos = obj.pos;
        end
        function att = get_attitude(obj)
            % Return the current attitude of the satellite
            att = obj.att;
        end

        function earth_vector = get_earth_vector(obj)
            % Calculate the earth vector in ECI frame with respect to the satellite
            r_sat = obj.pos;
            earth_vector = - r_sat / norm(r_sat);
        end
        function sun_vector = get_sun_vector(obj)
            % Calculate the sun vector in ECI frame with respect to the satellite at a given time
            JD = juliandate(obj.time);
            r_sun_eci = planetEphemeris(JD,'Sun','Earth','432t','km')'*1e3; % Sun position in ECI frame [m]
            r_sun_eci = r_sun_eci - obj.pos; % Vector from satellite to sun
            sun_vector = r_sun_eci / norm(r_sun_eci);
        end
    end
end