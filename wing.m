classdef wing < liftSurf
    % 
    % 
    %
    % Jemima Poynton 06/02/24
    properties
        %% GEOMETRY
        sweep(1,1) {mustBeReal, mustBeFinite} = 0
        CSs(1,:) = []
        sideY = 0; % side (left sideY = -1, right sideY = 1)
        mass(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        CG(1,1) {mustBeReal, mustBeFinite} = 0
        taper(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0.6;

        %% MOUNTING AND POSITION
        % pos: position (inboard root point trailing edge point) in the form [x, y, z]
        % ang: mounting in the form [theta_x, theta_y, theta_z]
        %
        % x: defined as forward flight direction (cruise) in the body frame
        %    +ve aft of the datum (typically the nose).
        % y: defined as perpendicular to forward flight, on the horizontal 
        %    plane, in the body frame +ve to the right of the datum.
        % z: z position, defined vertically up in the body frame
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

        function obj = addCS(obj, CS, distInb)
        % function addCS adds the control surface CS to the wing
        CS = setPosition(CS, distInb, obj);
        [~, CS] = getChord(CS, 0, obj);
        obj.CSs = [obj.CSs CS];
        end
    end
end