classdef wing < liftSurf
    % 
    % 
    %
    % Jemima Poynton 06/02/24
    properties
        %% GEOMETRY
        % sweep: TE sweep 

        sweep(1,1) {mustBeReal, mustBeFinite} = 0
        CSs(1,:) = []
        sideY = 0; % side (left sideY = -1, right sideY = 1)
        mass(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        CG(1,1) {mustBeReal, mustBeFinite} = 0
        taper(1,1) {mustBeReal, mustBeFinite, mustBeNonnegative} = 0.6
        sections(1,1) = 1
        twist(1,:) = [0]
        S(1,1) = 0;
        mesh = [];
        ac(1,3)
        Swet(1,1)

        %% MOUNTING AND POSITION
        % pos: position (inboard root point trailing edge point) in the form [x, y, z]
        % ang: mounting in the form [theta_x, theta_y, theta_z]
        %
        % x: defined as forward flight direction (cruise) in the body frame
        %    +ve aft of the datum (typically the nose).
        % y: defined as perpendicular to forward flight, on the horizontal 
        %    plane, in the body frame +ve to the right of the datum.
        % z: z position, defined vertically up in the body frame
        % theta_x: mounting angle CW about X
        % theta_y: mounting angle CW about y
        % theta_z: mounting angle CW about z

        pos(1,3) {mustBeReal, mustBeFinite} = [0 0 0];
        ang(1,3) {mustBeReal, mustBeFinite} = [0 0 0];
    end
    
    methods
        function obj = wing(sweep, sideY, mass, CG, taper, pos, ang, Cla, span, tc, c, aerofoil)
            obj.sweep = sweep;
            obj.sideY = sideY;
            obj.mass = mass;
            obj.CG = CG;
            obj.taper = taper;
            obj.pos = pos;
            obj.ang = ang;
            obj.aerofoil = aerofoil;
            
            obj = setGeometry(obj, Cla, span, tc, c);
            obj.S = obj.c*obj.span;
        end

        function [CL, obj] = getLift(obj, alpha, VLM, n)
        % getLiftCoeff gets the lift coefficient under the current
        % conditions
        %
        % Cl: lift coefficient provided by the surface in [x, y, z]
        % alpha: angle of attack
        % VLM: boolean determining whether vortex lattice method sould be
        %      applied, or mean aerofoil Cla used for estimation
        % n: number of panels in [x y] for VLM
        
        if meshBool == 1 % create mesh if wing needs to be re-meshed et
            obj = createMesh3D(obj, N, 1, 1);
        end

        [obj, CL, CD, distribution] = VLM(lift, alpha, n);

        end

        function obj = estimateWettedArea(obj)
            if obj.tc < 0.05
                obj.Swet = 2.003*obj.S;
            else
                obj.Swet = (1.977 + 0.52*obj.tc)*obj.S;
            end

        end

        function obj = getAC(obj)
        % Get aerodynamic centre from datum (usually nose)
            t = obj.taper;
            y_ = (obj.span/6)*((1 + 2*t)/(1+t));
            c_ = (2/3)*getChord(obj,0)*(1 + t + t^2)/(1 + t);

            x_ = obj.ac(2)*tan(obj.sweep) - 0.75*c_;
            
            Lt = transMatrix(obj.ang.*[1 -1 1]);
            obj.ac = obj.pos + [x_ y_ 0]*Lt;
        end

        function obj = addCS(obj, CS, distInb)
        % function addCS adds the control surface CS to the wing
        CS = setPosition(CS, distInb, obj);
        [~,CS] = getChord(CS, distInb, obj);
        obj.CSs = [obj.CSs CS];
        end
    end
end