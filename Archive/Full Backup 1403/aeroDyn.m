function [Fb,Mcg, forces, aero] = aeroDyn(coeff, u, alpha, beta, V, rho, X, refGeo, m, thrust, cg)
% Function aeroDyn simulates ....

%% Setting up intermediate variables
b = refGeo.bref;
S = refGeo.Sref;
c = refGeo.cref;

Q = 0.5*rho*V^2; % Dynamic pressure Q

ndc = c/(2*V);
ndb = b/(2*V);

%% Seperating vectors (BFR)
uvw = X(1:3); % Translational velocity vector
pqr = X(4:6); % Angular velocity vector
EA = X(7:end);

%% Non-dimensional Aerodynamic Force Coefficients
% In wind axis
% Lift
CL = applyDeriv(coeff.CL0, coeff.CLa, coeff.CLb, coeff.CLn, 0, coeff.CLq, 0, u, alpha, beta, pqr, ndc);

% Drag 
CD = applyDeriv(coeff.CD0, coeff.CDa, coeff.CDb, coeff.CDn, 0, 0, 0, u, alpha, beta, pqr, ndc);

% Sideforce
CY = applyDeriv(0, coeff.CYa, coeff.CYb, coeff.CYn, coeff.CYp, 0, coeff.CYr, u, alpha, beta, pqr, ndb);

%% Rotate wind to body
Faw = [-max(CD,0); CY; -CL].*Q*S; % Force vector in the stability frame

Rwb = [cos(alpha)*cos(beta), -cos(alpha)*sin(beta), -sin(alpha);  
       sin(beta)           ,  cos(beta)           ,  0         ; 
       cos(beta)*sin(alpha), -sin(alpha)*sin(beta),  cos(alpha)];

Fab = Rwb*Faw; % aerodynamic force on body

%% Aerodynamic Moment about CG
% In Fb, with moments taken about the centre of gravity

% Roll
Cl = applyDeriv(0, coeff.Cla, coeff.Clb, coeff.Cln, coeff.Clp, 0, coeff.Clr, u, alpha, beta, pqr, ndb);

% Pitch
Cm = applyDeriv(coeff.Cm0, coeff.Cma, coeff.Cmb, coeff.Cmn, 0, coeff.Cmq, 0, u, alpha, beta, pqr, ndc);

% Yaw
Cn = applyDeriv(0, coeff.Cna, coeff.Cnb, coeff.Cnn, coeff.Cnp, 0, coeff.Cnr, u, alpha, beta, pqr, ndb);

Ma = [b*Cl; c*Cm; b*Cn].*Q*S;

%% Propulsion Effects
% Moment and forces induced by throttle
FTb = zeros(3, length(thrust.Tinit));
MTcg = zeros(3, length(thrust.Tinit));

for i = 1:length(thrust.Tinit)
    idx = length(coeff.CLn) + 1 + (i-1)*4; % index of thrust-deflection pairs for thrust objects
    FTt = [0      ; % Thrust in rotor plane
           0      ;
           -u(idx)];
    
    Rrb = transMatrix([u(idx+1) u(idx+2) u(idx+3)].*[1 1 1]); % rotation matrix from rotor plane to body, first converting to geometric axis

    FTb(:,i) = Rrb*FTt; % Rotate to body frame, switching from geometric to dynamic coordinates
    MTcg(:,i) = cross(thrust.xyz_tr(i,:) - cg, FTb(:,i));
end

FTb = sum(FTb,2);
MTcg = sum(MTcg,2);

% Gravity Effects
Fgb = [-9.81*sin(X(8))          ; % Body frame
        9.81*cos(X(8))*sin(X(7));
        9.81*cos(X(8))*cos(X(7))]*m; % ASSUMPTION: Constant Mass

%% Explicit First Order Form
%Inertia matrix

if abs(alpha) > 14*(pi/180)
    Fab = [0 0 0]';
    Ma = [0 0 0]';
    disp("caught")
end


Fb = FTb + Fgb + Fab;
Mcg = MTcg + Ma;

forces = [FTb; Fgb; Fab];
aero = [CL; CD; Cm];

end

%% Extra functions

function C = applyDeriv(C0, Ca, Cb, Cn, Cp, Cq, Cr, u, alpha, beta, pqr, nd)
% function applyDeriv calculates the coefficient C from the derivatives C0,
% Ca, Cb, Cn wrt 0, alpha, beta and control (n), respectively.

    C_nc = C0 + Ca*alpha + Cb*beta + Cp*pqr(1)*nd + Cq*pqr(2)*nd + Cr*pqr(3)*nd; % lift coefficient no control
    
    C_c = (Cn.*u(1:length(Cn)))';

    C = C_nc + sum(C_c);

end
