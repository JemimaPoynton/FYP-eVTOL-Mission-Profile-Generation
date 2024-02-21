classdef thrust
    % 
    % 
    %
    % Jemima Poynton 06/02/24 

     properties
        mass(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        CG(1,1) {mustBeReal, mustBeFinite} = 0
        %% MOUNTING AND POSITION
        % All distances are taken from the centre of gravity, and angles
        % are taken from the centre of thrust 
        %
        % pos: position in the form [x, y, z]
        % ang: mounting in the form [theta_x, theta_y, theta_z]
        %
        % x: defined as forward flight direction (cruise) in the body frame
        % y: defined as perpendicular to forward flight, on the horizontal 
        %    plane, in the body frame
        % z: z position, defined vertically in the body frame
        % theta_x: mounting angle CW about X
        % theta_y: mounting angle CW about y
        % theta_z: mounting angle CW about z

        pos(1,3) {mustBeReal, mustBeFinite} = [0 0 0];
        ang(1,3) {mustBeReal, mustBeFinite} = [0 0 0];
     end

     methods
         function obj = setAngle(obj, angles)
         % function setAngle sets the orientation of the rotor, i.e. thrust
         % vectoring
         obj.ang = angles;
         end
     end
end