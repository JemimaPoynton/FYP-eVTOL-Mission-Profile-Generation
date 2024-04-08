classdef rotor < thrust
    properties
        radius(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0;
        rpm(1,1) {mustBeReal, mustBeFinite} = 0;
        solidity(1,1) {mustBeReal, mustBeFinite} = 0;
        maxdef = [1 -1]
        kt = 0.00613; % default
        kb = 0.0001; 
    end

    methods
        function obj = rotor(radius, rpm, solidity, mass, CG, pos, ang, maxThrust)
            obj.radius = radius;
            obj.rpm = rpm;
            obj.solidity = solidity;
            obj.mass = mass;
            obj.CG = CG;
            obj.pos = pos;
            obj.ang = ang;
            obj.maxThrust = maxThrust;
        end
    end
end