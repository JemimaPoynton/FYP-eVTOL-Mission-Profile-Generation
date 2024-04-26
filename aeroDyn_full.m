function xd = aeroDyn_full(coeff, u, rho, X, refGeo, m, thrust, cg, I)
% Function aeroDyn simulates .... Independent version (i.e. doesn't need alpha/beta/V inputs)

%% Calculate Airflow
V = sqrt(X(1)^2 + X(2)^2 + X(3)^2);
alpha = atan2(X(3), X(1));
beta = atan2(X(2), X(1));

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

%% Handle Stall

%% Non-dimensional Aerodynamic Force Coefficients
% In wind axis
% Lift
CL = applyDeriv(coeff.CL0, coeff.CLa, coeff.CLb, coeff.CLn, 0, coeff.CLq, 0, u, alpha, beta, pqr, ndc);

% Drag 
CD = applyDeriv(coeff.CD0, coeff.CDa, coeff.CDb, coeff.CDn, 0, 0, 0, u, abs(alpha), beta, pqr, ndc);

% Sideforce
CY = applyDeriv(0, coeff.CYa, coeff.CYb, coeff.CYn, coeff.CYp, 0, coeff.CYr, u, alpha, beta, pqr, ndb);

if abs(alpha) > 14.5*(pi/180)
    CL = 0;
    CD = coeff.CD0;
    CY = 0;
end

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

if abs(alpha) > 20*(pi/180)
    Cm = 0;
    Cn = 0;
    Cl = 0;
end

Ma = [b*Cl; c*Cm; b*Cn].*Q*S;

%% Propulsion Effects
% Moment and forces induced by throttle
FTb = zeros(3, length(thrust.Tinit));
MTcg = zeros(3, length(thrust.Tinit));

for i = 1:length(thrust.Tinit)
    kt = thrust.kt(i,:);
    kb = thrust.kb(i,:);
    dir = thrust.dir(i,:);

    idx = length(coeff.CLn) + 1 + (i-1)*4; % index of thrust-deflection pairs for thrust objects
    FTt = [0      ; % Thrust in rotor plane
           0      ;
           -kt*u(idx)^2];

    MTy = [0      ;
           0      ;
           dir*kb*(u(idx)^2)];
    
    Rrb = transMatrix([u(idx+1) u(idx+2) u(idx+3)].*[1 1 1]); % rotation matrix from rotor plane to body, first converting to geometric axis

    % ADD INDUCED ROTOR MOMENT HERE

    FTb(:,i) = Rrb*FTt; % Rotate to body frame, switching from geometric to dynamic coordinates
    MTcg(:,i) = Rrb*MTy + cross(thrust.xyz_tr(i,:) - cg, FTb(:,i))';
end

FTb = sum(FTb,2);
MTcg = sum(MTcg,2);

% Gravity Effects
Fgb = [-9.81*sin(X(8))          ; % Body frame
        9.81*cos(X(8))*sin(X(7));
        9.81*cos(X(8))*cos(X(7))]*m; % ASSUMPTION: Constant Mass

%% Out
Rbe = body2earth(X(7), X(8), X(9));

Fb = FTb + Fgb + Fab;
Mcg = MTcg + Ma;

forces = [FTb; Fgb; Fab; Rbe*Fb];
aero = [CL; CD; Cm];
uvw_e = Rbe*X(1:3);

xd = explicitFO(Fb, Mcg, I, X, m)';

end

%% Extra functions

function C = applyDeriv(C0, Ca, Cb, Cn, Cp, Cq, Cr, u, alpha, beta, pqr, nd)
% function applyDeriv calculates the coefficient C from the derivatives C0,
% Ca, Cb, Cn wrt 0, alpha, beta and control (n), respectively.

    C_nc = C0 + Ca*alpha + Cb*beta + Cp*pqr(1)*nd + Cq*pqr(2)*nd + Cr*pqr(3)*nd; % lift coefficient no control
    
    C_c = (Cn.*u(1:length(Cn)))';

    C = C_nc + sum(C_c);

end


