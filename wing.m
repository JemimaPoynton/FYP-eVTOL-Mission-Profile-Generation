classdef wing < liftSurf
    % 
    % 
    %
    % Jemima Poynton 06/02/24
    properties
        %% GEOMETRY
        sweep(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        CSs = []
        %% MOUNTING AND POSITION
        % All distances are taken from the centre of gravity, and angles
        % are taken from the wing root
        %
        % pos: position (inboard root point trailing edge point) in the form [x, y, z]
        % ang: mounting in the form [theta_x, theta_y, theta_z]
        %
        % x: defined as forward flight direction (cruise) in the body frame
        % y: defined as perpendicular to forward flight, on the horizontal 
        %    plane, in the body frame
        % z: z position, defined vertically down in the body frame
        % theta_x: mounting angle CW about X
        % theta_y: mounting angle CW about y
        % theta_z: mounting angle CW about z

        pos(1,3) {mustBeReal, mustBeFinite} = [0 0 0];
        ang(1,3) {mustBeReal, mustBeFinite} = [0 0 0];
    end
    
    methods
        function Cl = getLiftCoeff(obj, alpha, Re)
        % getLiftCoeff gets the lift coefficient under the current
        % conditions
        %
        % Cl: lift coefficient provided by the surface in [x, y, z]
        % alpha: angle of attack
        % Re: Reynold's number

        % Calculate lift coefficient using Prandtl, accounting for any
        % deflection in control surfaces - input these as states
        % Get for x,y,z taking into consideration the mounting angles etc.
        end

        function obj = addCS(obj, CS)
        % function addCS adds the control surface CS to the wing
        obj.CSs = [obj.CSs CS];
        end
    end
end