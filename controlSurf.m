classdef controlSurf < liftSurf
    % Subclass representing control surfaces
    %
    % Jemima Poynton 06/02/24
    properties
        maxDeflection(1,2) {mustBeReal, mustBeFinite, mustBeInRange(maxDeflection,-3.142,3.142)} = [0 0]
        deflection(1,1) {mustBeReal, mustBeFinite} = 0
        pos(1,3) {mustBeReal, mustBeFinite} = [0 0 0];
    end

    methods
        function obj = setPosition(distInb, liftSurf)
        % function setPosition converts dimensions in the plane of the wing
        % into x,y,z coordinates.
        %
        % liftSurf: Associated lifting surface on which it is mounted (e.g. wing)
        % distInb: Inboard distance along lifting surface plane from root
        x = []; % !Determine this after testing in plotConfig!
        y = [];
        z = [];
        obj.pos = [x,y,z];

        end

        function obj = setMaxDeflection(ub, lb)
        % function setMaxDeflection restricts the deflection of the
        % control surface between ub, lb.
            obj.maxDeflection = [ub lb];
        end

        function obj = deflect(obj, def)
        % function deflect deflects the control surface by deflection
        % deflection: deflection from mounting i.e. inital state
    
            if def <= obj.maxDeflection(1)
                obj.deflection = obj.maxDeflection(1);
            elseif def >= obj.maxDeflection(2)
                obj.deflection = obj.maxDeflection(2);
            else
                obj.deflection = def;
            end
        end
    end
end