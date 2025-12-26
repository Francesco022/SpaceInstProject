classdef Sensor < handle
    %SENSOR Represent a sensor attached to a satellite
    %   Defines orientation and position relative to the parent satellite
    %   and updates visualization automatically when the satellite moves or rotates.

    properties
        parent          % Handle to the parent satellite
        position        % Position relative to parent [3x1]
        attitude        % Orientation relative to parent (quaternion)
        body            % Body structure (X,Y,Z,color)

        % Visualization handles
        hSurf
        hTop
        hBottom
        hQuiverX
        hQuiverY
        hQuiverZ
        ax
        scale
    end

    methods
        %% CONSTRUCTOR
        function obj = Sensor(parentSat, pos_0, eul_0, body)
            % Initialize sensor attached to a satellite
            obj.parent = parentSat;
            obj.position = pos_0;
            obj.attitude = quaternion(deg2rad(eul_0), 'euler', 'ZYX', 'frame');

            % Default cube body
            if nargin < 4 || isempty(body)
                l = 0.2;  % cube edge
                nSides = 4;
                [X,Y,Z] = cylinder(l/sqrt(2), nSides);
                Z = l*(Z-0.5);

                % Rotate 45° around Z to align with axes
                pts1 = [X(1,:); Y(1,:); Z(1,:)];
                pts2 = [X(2,:); Y(2,:); Z(2,:)];
                theta = deg2rad(45);
                Rz = [cos(theta) -sin(theta) 0;
                      sin(theta)  cos(theta) 0;
                      0           0          1];
                pts1 = Rz * pts1;
                pts2 = Rz * pts2;

                X(1,:) = pts1(1,:); Y(1,:) = pts1(2,:); Z(1,:) = pts1(3,:);
                X(2,:) = pts2(1,:); Y(2,:) = pts2(2,:); Z(2,:) = pts2(3,:);

                obj.body.type = 'cube';
                obj.body.X = X;
                obj.body.Y = Y;
                obj.body.Z = Z;
                obj.body.color = [1 0 0];  % default red
            else
                obj.body = body;
            end
        end

        %% POSITION AND ATTITUDE
        function pos = getPosition(obj)
            % Get sensor position in inertial frame
            parentPos = obj.parent.getPosition();
            parentAtt = rotmat(obj.parent.getAttitude(), 'point');
            pos = parentPos + parentAtt * obj.position;
        end

        function q = getAttitude(obj)
            % Get sensor orientation in inertial frame
            parentAtt = obj.parent.getAttitude();
            q = parentAtt * obj.attitude;
        end

        function bodyOut = getBody(obj)
            % Get sensor body points transformed in inertial frame
            pos = obj.getPosition();
            q = obj.getAttitude();
            R = rotmat(q, 'point');

            X = obj.body.X; Y = obj.body.Y; Z = obj.body.Z;
            Xr = X; Yr = Y; Zr = Z;
            for i = 1:size(X,1)
                pts = [X(i,:); Y(i,:); Z(i,:)];
                pts_rot = R * pts;
                Xr(i,:) = pts_rot(1,:) + pos(1);
                Yr(i,:) = pts_rot(2,:) + pos(2);
                Zr(i,:) = pts_rot(3,:) + pos(3);
            end

            bodyOut.X = Xr;
            bodyOut.Y = Yr;
            bodyOut.Z = Zr;
            bodyOut.color = obj.body.color;
        end

        %% VISUALIZATION
        function initVisual(obj, ax, scale)
            % Initialize visualization on axes ax
            if nargin < 3, scale = 1; end
            obj.ax = ax;
            obj.scale = scale;
            hold(ax,'on');

            body_f = obj.getBody();
            alphaValue = 0.2;

            obj.hSurf   = surf(ax, body_f.X, body_f.Y, body_f.Z, ...
                               'FaceColor', body_f.color, 'LineStyle','none', 'FaceAlpha', alphaValue);
            obj.hTop    = fill3(ax, body_f.X(1,:), body_f.Y(1,:), body_f.Z(1,:), body_f.color, 'FaceAlpha', alphaValue);
            obj.hBottom = fill3(ax, body_f.X(2,:), body_f.Y(2,:), body_f.Z(2,:), body_f.color, 'FaceAlpha', alphaValue);

            % Draw sensor axes
            R = rotmat(obj.getAttitude(), 'point');
            p = obj.getPosition();
            obj.hQuiverX = quiver3(ax, p(1),p(2),p(3), R(1,1)*scale, R(2,1)*scale, R(3,1)*scale, 'y','LineWidth',2);
            obj.hQuiverY = quiver3(ax, p(1),p(2),p(3), R(1,2)*scale, R(2,2)*scale, R(3,2)*scale, 'm','LineWidth',2);
            obj.hQuiverZ = quiver3(ax, p(1),p(2),p(3), R(1,3)*scale, R(2,3)*scale, R(3,3)*scale, 'c','LineWidth',2);
        end

        function updateVisual(obj)
            % Update visualization to match current satellite state
            body_f = obj.getBody();

            set(obj.hSurf,   'XData', body_f.X, 'YData', body_f.Y, 'ZData', body_f.Z);
            set(obj.hTop,    'XData', body_f.X(1,:), 'YData', body_f.Y(1,:), 'ZData', body_f.Z(1,:));
            set(obj.hBottom, 'XData', body_f.X(2,:), 'YData', body_f.Y(2,:), 'ZData', body_f.Z(2,:));

            R = rotmat(obj.getAttitude(), 'point');
            p = obj.getPosition();
            set(obj.hQuiverX, 'XData', p(1), 'YData', p(2), 'ZData', p(3), ...
                'UData', R(1,1)*obj.scale, 'VData', R(2,1)*obj.scale, 'WData', R(3,1)*obj.scale);
            set(obj.hQuiverY, 'XData', p(1), 'YData', p(2), 'ZData', p(3), ...
                'UData', R(1,2)*obj.scale, 'VData', R(2,2)*obj.scale, 'WData', R(3,2)*obj.scale);
            set(obj.hQuiverZ, 'XData', p(1), 'YData', p(2), 'ZData', p(3), ...
                'UData', R(1,3)*obj.scale, 'VData', R(2,3)*obj.scale, 'WData', R(3,3)*obj.scale);
        end
    end
end
