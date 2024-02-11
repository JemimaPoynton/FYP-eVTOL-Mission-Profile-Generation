classdef fuselage
    % Class definition liftSurf represents the properties of a fuselage/body.
    %
    % Jemima Poynton 06/02/24

    properties
    %% Geometry
    % weight: overall weight
    % CG: Centre of gravity position, aft from nose
    % kf: factor representing contribution to !WHAT?!
    % length: length aft to fwd
    % diameter: mean fuselage diameter
    % Swet: 'wetted' area
    % kc: factor 1 - 1.4 representing how well the aft fuselage is estimated by a conical shape
    % laft: length of conical aft fuselage i.e. tail bearing section

        weight(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        CG(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        kf(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        leng(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        diameter(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        Swet(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        kc(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 1
        laft(1,1) = 0;

    %% MOUNTING AND POSITION
    % pos: position (inboard root point trailing edge point) in the form [x, y, z]
    % ang: mounting in the form [theta_x, theta_y, theta_z]
    %
    % x: defined as forward flight direction (cruise) in the body frame
    %    +ve aft of the datum.
    % y: defined as perpendicular to forward flight, on the horizontal 
    %    plane, in the body frame +ve to the right of the CG
    % z: z position, defined vertically down in the body frame

        pos(1,3) {mustBeReal, mustBeFinite} = [0 0 0];
    end
    
    methods
        function obj = setGeometry(obj, weight, CG, kf, leng, diameter)
            obj.weight = weight;
            obj.CG = CG;
            obj.kf = kf;
            obj.len = leng;
            obj.diameter = diameter;
        end

        function drag = estimateDrag(obj)
        end
    end
end