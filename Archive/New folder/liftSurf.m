classdef liftSurf
    % Class definition liftSurf represents the properties and mounting
    % position of a single lifting surface, e.g. a wing.
    %
    % Jemima Poynton 06/02/24

    properties
    %% AERODYNAMIC
       % Cla: mean lift curve slope (linearised aerodynamics) [rad]
       % span: Span of lift surface [m]. Note that wings are individually
       %       defined, so for a symmetric aircraft the wingspan is 2*span.
       % tc: chord thickness ratio
       % c: mean chord [m]
       % taper: taper ratio
       % NACA_Data: object containing empirical lift-curve slope data. 

       Cla(1,1) {mustBeReal, mustBeFinite} = 2*pi;
       span(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0;
       tc(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0;
       c(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0;
       aerofoil = "0012";

    end

    methods
        function obj = setGeometry(obj, Cla, span, tc, c)
            obj.Cla = Cla;
            obj.span = span;
            obj.tc = tc;
            obj.c = c;
        end

        function chord = getChord(obj, dist)
        % function getChord gets the chord at a given dist along the
        % span
            CR = ((2*obj.c)/(1 + obj.taper));
            CT = obj.taper*CR;
            chord = CR + (CT - CR)*(abs(dist)/obj.span);      
        end

        function obj = getNACA_Data(obj, name)
        % function getNACA_Data gets the data for a given aerofoil, defined
        % by name, if available
        %
        % name: NACA aerofoil standard name

        [obj.NACA_Data, obj.Cla, obj.tc, ~] = getNACA(name);
        obj.aerofoil = name;
        obj.tc = char2double(char(name(end-1:end)));
        end
    end
end