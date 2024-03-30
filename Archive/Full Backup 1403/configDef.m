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
        I(3,3) = [0 0 0; 0 0 0; 0 0 0]
        thrust = struct('rotors',[],'ducts',[])
        lift
        fuselage
        CoL(1,3) = [0 0 0]
        CG(1,3) = [0 0 0]
        Cd0(1,1)
        m(1,1) = 0
        refGeo = struct('Sref', 0, 'cref', 0, 'bref', 0)
    end

    properties (Access = private)
        Cfe(1,1) = 0.004; % default value from empirical reference: https://www.fzt.haw-hamburg.de/pers/Scholz/HOOU/AircraftDesign_13_Drag.pdf
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

        function obj = setReferenceGeometry(obj, S, b, c)
            obj.refGeo.Sref = S;
            obj.refGeo.bref = b;
            obj.refGeo.cref = c;
        end

        function Cd0 = estimateFormDrag(obj, Sref, Cfe)
        % function estimateDrag estimates the drag introduced by the wetted
        % area of the fuselage and lifting surfaces
        %
        % Cfe: equivalent friction coefficient
        % Sref: reference area for non-dimensionalisation

            if ~exist('Cfe','var') % Allow for use of default value or input value
                Cfe = obj.Cfe;
            end
            
            obj.Cd0 = 0;

            for i = 1:length(obj.fuselage)
                obj.fuselage(i) = estimateWettedArea(obj.fuselage(i), obj.fuselage(i).type, obj.fuselage(i).ln);       
                obj.Cd0 = obj.Cd0 + Cfe*(obj.fuselage(i).Swet/Sref);
            end

            for i = 1:length(obj.lift)
                obj.lift(i) = estimateWettedArea(obj.lift(i));
                obj.Cd0 = obj.Cd0 + Cfe*(obj.lift(i).Swet/Sref);
            end
        end
    end
end