classdef liftSurf
    % Class definition liftSurf represents the properties and mounting
    % position of a single lifting surface, e.g. a wing.
    %
    % Jemima Poynton 06/02/24

    properties
    %% AERODYNAMIC
       % Cla: mean lift curve slope (linearised aerodynamics) [rad]
       % span: Span of wing [m]
       % tc: chord thickness ratio
       % c: mean chord [m]
       % taper: taper ratio
       % NACA_Data: object containing empirical lift-curve slope data. 

       Cla(1,1) {mustBeReal, mustBeFinite} = 2*pi;
       span(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0;
       tc(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0;
       c(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0;
       taper(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 1;

    end

    methods
        function obj = setGeometry(obj, Cla, span, tc, c, taper)
            obj.Cla = Cla;
            obj.span = span;
            obj.tc = tc;
            obj.c = c;
            obj.c = taper;
        end

        function chord = getChord(obj, dist)
        % function getChord gets the chord at a given dist along the
        % span
        
        end

        function obj = getNACA_Data(obj, name)
        % function getNACA_Data gets the data for a given aerofoil, defined
        % by name, if available
        %
        % name: NACA aerofoil standard name

        [obj.NACA_Data, obj.Cla, obj.tc] = getNACA(name); % need to define getNACA function - is there already a NACA database addon?
        end
    end
end