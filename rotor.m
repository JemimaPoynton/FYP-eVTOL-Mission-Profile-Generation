classdef rotor < thrust
    properties
        radius(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0;
        rpm(1,1) {mustBeReal, mustBeFinite} = 0;
        solidity(1,1) {mustBeReal, mustBeFinite} = 0;
        maxdef = [1 -1]
        kt = 3.13e-5; % default
        kb = 7.5e-7;
        dir(1,1){mustBeMember(dir,[-1 1])} = 1;
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