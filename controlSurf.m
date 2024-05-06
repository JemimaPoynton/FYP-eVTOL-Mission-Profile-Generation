classdef controlSurf < liftSurf
    % Subclass representing control surfaces
    %
    % Jemima Poynton 06/02/24
    properties
        maxDeflection(1,2) {mustBeReal, mustBeFinite, mustBeInRange(maxDeflection,-3.142,3.142)} = [-3.142 3.142]
        deflection(1,1) {mustBeReal, mustBeFinite} = 0
        pos(1,3) {mustBeReal, mustBeFinite} = [0 0 0];
        chordRatio(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative}
        edge(1,1) {mustBeMember(edge, ["LE" "TE"])} = "TE"
        roll(1,1) {mustBeMember(roll, [1 0])} = 0;
    end

    methods
        function obj = controlSurf(chordRatio, edge, Cla, span, tc)
            obj.chordRatio = chordRatio;
            obj.edge = edge;
            obj.Cla = Cla;
            obj.span = span;
            obj.tc = tc;
        end

        function obj = setPosition(obj, distInb, liftSurf)
        % function setPosition converts dimensions in the plane of the wing
        % into x,y,z coordinates.
        %
        % liftSurf: Associated lifting surface on which it is mounted (e.g. wing)
        % distInb: Inboard distance along lifting surface plane from root
            if obj.edge == "LE"
                Lt = transMatrix(liftSurf.ang);
                obj.pos = liftSurf.pos + [distInb*tan(liftSurf.sweep)- getChord(liftSurf, distInb) + getChord(obj, 0, liftSurf) distInb*liftSurf.sideY 0]*Lt;
            else
                Lt = transMatrix(liftSurf.ang);
                obj.pos = liftSurf.pos + [distInb*tan(liftSurf.sweep) distInb*liftSurf.sideY 0]*Lt;
            end          

            obj.c = obj.chordRatio*liftSurf.c;

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

        function [chord, obj] = getChord(obj, dist, liftSurf)
        % function getChord gets the chord at a given dist along the
        % span
            obj.c = obj.chordRatio*liftSurf.getChord(abs(obj.pos(2)) + obj.span/2);
            chord =obj.chordRatio*liftSurf.getChord(abs(obj.pos(2)) + dist);      
        end
    end
end