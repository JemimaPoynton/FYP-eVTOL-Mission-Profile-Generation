classdef configDef
    % Class definition configDef represents a configuration including basic
    % components and geometry.
    %
    % thrust: vector containing all 'thrust' objects present in the configuration.
    %         configuration. These may be any thrust vectoring component or
    %         system e.g. rotors or ducts, defined as subclasses.
    %
    % lift: vector containing all liftSurf objects in the configuration. Includes
    %        only primary lifting surfaces, with integrated control surfaces.
    %
    % fuselage: vector containing all fuselage objects in the configuration.
    %
    % I: Moments of inertia in the form [Ixx, Iyy, Izz]
    % Jemima Poynton 06/02/24

    properties
        description % free variable for labelling
        I(1,3)
        thrust = struct('rotors',[],'ducts',[])
        lift
        fuselage
        CoL(1,3) = [0 0 0]
        CG(1,3) = [0 0 0]
    end

    methods
        function obj = addThrust(obj, newThrust)
        % function addThrust adds an additional thruster to the
        % configuration. This may be either the rotor or duct subclass.
        if class(newThrust) == "rotor"
            obj.thrust.rotors = [obj.thrust.rotors newThrust];
        elseif class(newThrust) == "duct"
            obj.thrust.ducts = [obj.thrust.ducts newThrust];
        end

        end

        function obj = addLift(obj, newLift)
        % function addThrust adds an additional lift surface to the
        % configuration. This may be a wing or tail, and will contain 
        % control surfaces.

        obj.lift = [obj.lift newLift];

        end

        function obj = addFuselage(obj, newFuselage)
        % function addThrust adds an additional lift surface to the
        % configuration. This may be a wing or tail, and will contain 
        % control surfaces.
        obj.fuselage = [obj.fuselage newFuselage];
        end

        function obj = calculateCoL(obj)
        % function calculateCoL calculates and sets the centre of lift of
        % the configuration.
        end

        function obj = calculateCG(obj)
        end

        function obj = estimateCD(obj)
        % function calculateCoL calculates and sets the centre of lift of
        % the configuration.
        end
    end
end