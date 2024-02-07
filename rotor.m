classdef rotor < thrust
    properties
        radius(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0;
        rpm(1,1) {mustBeReal, mustBeFinite} = 0;
    end

    methods
        function thrust = calculateThrust(obj)
        end

        function obj = setrpm(obj, rpm)
            obj.rpm = rpm;
        end
    end
end