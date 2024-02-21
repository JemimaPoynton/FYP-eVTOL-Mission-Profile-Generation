classdef fuselage
    % Class definition liftSurf represents the properties of a fuselage/body.
    %
    % Jemima Poynton 06/02/24

    properties
    %% Geometry
    % weight: overall weight
    % CG: Centre of gravity position, aft from nose
    % kf: factor representing contribution to !what?!
    % length: length aft to fwd
    % diameter: mean fuselage diameter
    % Swet: 'wetted' area
    % kc: factor 1 - 1.4 representing how well the aft fuselage is estimated by a conical shape
    % laft: length of conical aft fuselage i.e. tail bearing section

        weight(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        CG(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        kf(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        leng(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        diameter(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        Swet(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        kc(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 1
        laft(1,1) = 0;
        type = "cylindrical"
        ln(1,1)

    %% MOUNTING AND POSITION
    % pos: position (inboard root point trailing edge point) in the form [x, y, z]
    % ang: mounting in the form [theta_x, theta_y, theta_z]
    %
    % x: defined as forward flight direction (cruise) in the body frame
    %    +ve aft of the datum.
    % y: defined as perpendicular to forward flight, on the horizontal 
    %    plane, in the body frame +ve to the right of the CG
    % z: z position, defined vertically down in the body frame

        pos(1,3) {mustBeReal, mustBeFinite} = [0 0 0];
    end
    
    methods
        function obj = fuselage(weight, CG, kf, leng, diameter, Swet, kc, laft)
            obj.Swet = Swet;
            obj.kc = kc;
            obj.laft = laft;

            obj = setGeometry(obj, weight, CG, kf, leng, diameter);
            obj = estimateWettedArea(obj, "cylindrical"); % get default Swet
        end

        function obj = setGeometry(obj, weight, CG, kf, leng, diameter)
            obj.weight = weight;
            obj.CG = CG;
            obj.kf = kf;
            obj.leng = leng;
            obj.diameter = diameter;
        end

        function obj = estimateWettedArea(obj, type, ln)
        % function setWettedArea applies an estimation of wetted area. 
        %
        % type: specifies whether to use estimations for a fuselage with a
        %       centre cylindrical section, or a streamlined fuselage.
    
            assert(ismember(lower(string(type)), ["cylindrical", "streamlined"]), "'type' must be either ""cylindrical"" or ""streamlined""")
            type = lower(string(type)); % case and type flexibility
    
            fineness = obj.leng/obj.diameter;
    
            if type == "cylindrical"
                obj.Swet = pi*obj.diameter*obj.leng*(1 + (1/fineness^2))*(1 - (2/fineness))^(2/3);

            elseif type == "streamlined"
                obj.Swet = pi*obj.diameter*obj.leng*(1.015 + (0.3/fineness^1.5))*(0.5 + 0.135*(ln/obj.leng))^(2/3);
            end
        
        end
    end
end