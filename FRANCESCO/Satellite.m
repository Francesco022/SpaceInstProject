classdef Satellite  < handle
    %SATELLITE Create and manage satellite object
    %   Create a satellite object with:
    %   - position
    %   - attitude
    %   - body
    %
    %   For easyer interpretation the initial attitude is given in Euler
    %   angles ZYX rotation [deg]

    properties
        position
        attitude
        body
        % --- Handles for visualization
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
        %% CREATION
        function obj = Satellite(pos_0, eul_0, body)
            %SATELLITE Construct an instance of this class
        
            % Position
            obj.position = pos_0;
        
            % Attitude (Euler ZYX -> quaternion)
            obj.attitude = quaternion(deg2rad(eul_0), 'euler', 'ZYX', 'frame');
        
            % Body definition
            if nargin < 3 || isempty(body)
                % Default body: cylinder
        
                r = 1;        % radius
                h = 2;        % height
                nSides = 100; % resolution
        
                [X,Y,Z] = cylinder(r, nSides);
                Z = h * (Z - 0.5);   % center the cylinder in body frame
        
                obj.body.type = 'cylinder';
                obj.body.X = X;
                obj.body.Y = Y;
                obj.body.Z = Z;
        
                obj.body.color = [0 0 1];   % color (rgb)
            else
                % User-defined body
                obj.body = body;
            end
        end

        %% UPDATE METHODS
        function setPosition(obj, newPos)
            %SETPOSITION Set satellite position [3x1]
            validateattributes(newPos, {'numeric'}, {'vector','numel',3});
            obj.position = newPos(:);
        end
        function translate(obj, deltaPos)
            %TRANSLATE Translate satellite by deltaPos [3x1]
            validateattributes(deltaPos, {'numeric'}, {'vector','numel',3});
            obj.position = obj.position + deltaPos(:);
        end

        function setAttitude(obj, q)
            %SETATTITUDE Set satellite attitude quaternion
            validateattributes(q, {'quaternion'}, {});
            obj.attitude = normalize(q);
        end
        function rotateByQuaternion(obj, dq)
            %ROTATEBYQUATERNION Incremental rotation (inertial frame)
            validateattributes(dq, {'quaternion'}, {});
            obj.attitude = normalize(obj.attitude * dq);
        end
        function rotateByEuler(obj, eul_deg)
            %ROTATEBYEULER Incremental ZYX rotation [deg]
            validateattributes(eul_deg, {'numeric'}, {'vector','numel',3});
            dq = quaternion(deg2rad(eul_deg), 'euler', 'ZYX', 'frame');
            obj.attitude = normalize(obj.attitude * dq);
        end

        %% RETURN METHODS
        function pos = getPosition(obj)
            %GETPOSITION Returns satellite position in inertial frame
            pos = obj.position;
        end

        function q = getAttitude(obj)
            %GETATTITUDE Returns satellite attitude quaternion
            q = obj.attitude;
        end

        function bodyOut = getBody(obj)
            %GETBODY Returns satellite body transformed in inertial frame
        
            % Original body points (body frame)
            X = obj.body.X;
            Y = obj.body.Y;
            Z = obj.body.Z;
        
            % Rotation matrix from quaternion
            R = rotmat(obj.attitude, 'point');
        
            % Preallocate
            Xr = X;
            Yr = Y;
            Zr = Z;
        
            % Rotate top and bottom circles (same logic as reference code)
            for i = 1:size(X,1)
                points = [X(i,:); Y(i,:); Z(i,:)];   % 3xN
                points_rot = R * points;
        
                Xr(i,:) = points_rot(1,:) + obj.position(1);
                Yr(i,:) = points_rot(2,:) + obj.position(2);
                Zr(i,:) = points_rot(3,:) + obj.position(3);
            end
        
            % Output structure
            bodyOut.X = Xr;
            bodyOut.Y = Yr;
            bodyOut.Z = Zr;
            bodyOut.color = obj.body.color;
        end

        %% VISUALIZATION
        function initVisual(obj, ax, scale)
            %INITVISUAL Initialize satellite visualization
            %   ax    = axes handle where to plot
            %   scale = scaling factor for body-frame axes (optional, default=1)
        
            if nargin < 3, scale = 1; end
            obj.ax = ax;
            obj.scale = scale;
            hold(ax,'on');  % Keep axes for animation
        
            % Get satellite body in inertial frame
            body_f = obj.getBody();
            alphaValue = 0.2;  % Transparency
        
            % Create surface and top/bottom faces
            obj.hSurf   = surf(ax, body_f.X, body_f.Y, body_f.Z, ...
                               'FaceColor', body_f.color, 'LineStyle','none', 'FaceAlpha', alphaValue);
            obj.hTop    = fill3(ax, body_f.X(1,:), body_f.Y(1,:), body_f.Z(1,:), body_f.color, ...
                                'FaceAlpha', alphaValue);
            obj.hBottom = fill3(ax, body_f.X(2,:), body_f.Y(2,:), body_f.Z(2,:), body_f.color, ...
                                'FaceAlpha', alphaValue);
        
            % Create body-frame axes (quivers)
            R = rotmat(obj.attitude,'point'); % Rotation matrix
            p = obj.position;
            obj.hQuiverX = quiver3(ax, p(1),p(2),p(3), R(1,1)*scale, R(2,1)*scale, R(3,1)*scale, ...
                                   'r','LineWidth',2);
            obj.hQuiverY = quiver3(ax, p(1),p(2),p(3), R(1,2)*scale, R(2,2)*scale, R(3,2)*scale, ...
                                   'g','LineWidth',2);
            obj.hQuiverZ = quiver3(ax, p(1),p(2),p(3), R(1,3)*scale, R(2,3)*scale, R(3,3)*scale, ...
                                   'b','LineWidth',2);
        end
        
        function updateVisual(obj)
            %UPDATEVISUAL Update satellite visualization in the current axes
            %   Only updates the coordinates of surfaces and quivers for smooth animation
        
            % Get updated body in inertial frame
            body_f = obj.getBody();
        
            % Update surface and top/bottom faces
            set(obj.hSurf,   'XData', body_f.X, 'YData', body_f.Y, 'ZData', body_f.Z);
            set(obj.hTop,    'XData', body_f.X(1,:), 'YData', body_f.Y(1,:), 'ZData', body_f.Z(1,:));
            set(obj.hBottom, 'XData', body_f.X(2,:), 'YData', body_f.Y(2,:), 'ZData', body_f.Z(2,:));
        
            % Update body-frame axes
            R = rotmat(obj.attitude,'point'); % Rotation matrix
            p = obj.position;
            set(obj.hQuiverX, 'XData', p(1), 'YData', p(2), 'ZData', p(3), ...
                'UData', R(1,1)*obj.scale, 'VData', R(2,1)*obj.scale, 'WData', R(3,1)*obj.scale);
            set(obj.hQuiverY, 'XData', p(1), 'YData', p(2), 'ZData', p(3), ...
                'UData', R(1,2)*obj.scale, 'VData', R(2,2)*obj.scale, 'WData', R(3,2)*obj.scale);
            set(obj.hQuiverZ, 'XData', p(1), 'YData', p(2), 'ZData', p(3), ...
                'UData', R(1,3)*obj.scale, 'VData', R(2,3)*obj.scale, 'WData', R(3,3)*obj.scale);
        end

    end
end