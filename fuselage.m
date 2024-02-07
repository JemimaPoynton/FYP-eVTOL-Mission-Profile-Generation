classdef fuselage
    % Class definition liftSurf represents the properties of a fuselage/body.
    %
    % Jemima Poynton 06/02/24

    properties
    % weight: overall weight
    % CG: Centre of gravity position, aft from nose
    % kf: factor representing drag contribution
    % length: length aft to fwd
    % diameter: mean fuselage diameter

        weight(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        CG(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        kf(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        length(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        diameter(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
    end
    
    methods
        function obj = setGeometry(obj, weight, CG, kf, length, diameter)
            obj.weight = weight;
            obj.CG = CG;
            obj.kf = kf;
            obj.length = length;
            obj.diameter = diameter;
        end

        function drag = estimateDrag(obj)
        end
    end
end