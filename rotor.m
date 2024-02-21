classdef rotor < thrust
    properties
        radius(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0;
        rpm(1,1) {mustBeReal, mustBeFinite} = 0;
        solidity(1,1) {mustBeReal, mustBeFinite} = 0;
    end

    methods
        function obj = rotor(radius, rpm, solidity, mass, CG, pos, ang)
            obj.radius = radius;
            obj.rpm = rpm;
            obj.solidity = solidity;
            obj.mass = mass;
            obj.CG = CG;
            obj.pos = pos;
            obj.ang = ang;
        end

        function thrust = calculateThrust(obj)
        end
        
        function obj = setrpm(obj, rpm, demandThrust)
            obj.rpm = rpm;
        end
    end
end