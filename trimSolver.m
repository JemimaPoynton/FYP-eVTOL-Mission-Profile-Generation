function [U, X, forces, aero, uvw_e, alpha, MTcg, failed] = trimSolver(aircraft, coeff, rho, Va, traj, rpitch, alphaLim)
% function trimSolver solves the overactuated trim optimisation problem for
% aircraft with state vector 'X' in the form [u v w p q r psi theta phi]'

% !Extract control surface max deflection, and add limits from config, extract coefficients!
failed = 0;

%% Extract Aircraft Data
nT = length(aircraft.thrust.rotors) + length(aircraft.thrust.ducts);
nC = length(coeff.CLn);

I = aircraft.I;
m = aircraft.m;
thrustIn = thrustobj2struct(aircraft, zeros(1,nT));

cg = aircraft.CG;
refGeo = aircraft.refGeo;

%% Define Anonymous Function
func = @(Z)trimCost(Z, Va, coeff, rho, refGeo, m, thrustIn, cg, I, traj, rpitch);
% trCon = @(U)

%% Define Saturation
uwRatio = atan(alphaLim);

lb = [0 -Va*uwRatio -pi/4 -pi/4 -0.52 -0.52 -0.52 0 0 0 0]';
ub = [+Va +Va*uwRatio +pi/4 +pi/4 0.52 0.52 0.52 7000 7000 7000 7000]';

lb_ga = [0 -Va*uwRatio -pi/4 -pi/4 -0.52 -0.52 -0.52 0]';
ub_ga = [+Va +Va*uwRatio +pi/4 +pi/4 0.52 0.52 0.52 7000]';

%% Solve
opts = optimset('PlotFcns','optimplotfval', 'Algorithm', 'sqp');

z0p_func = @(Z)trimCost_ga(Z, Va, coeff, rho, refGeo, m, thrustIn, cg, I, traj, rpitch);
z0p = guess(z0p_func, lb_ga, ub_ga, 8, Va)';

z0 = zeros(1,11)';
z0(1:7) = z0p(1:7);
z0(end-3:end) = z0p(end);

[Z, fval] = fmincon(func, z0, [], [], [], [], lb, ub, [], opts);

% if fval > 0.01
%     opts = optimoptions('patternsearch','PlotFcn', @psplotbestf, 'FunctionTolerance', 1e-62 ...
%     , 'MaxIterations', 50000, 'MaxFunctionEvaluations', 50000);
% 
%     [Z, fval] = patternsearch(func, Z, [], [], [], [], lb, ub, [], opts);
% end

%% Check Convergence
U = [Z(5:7)' Z(8) 0 rpitch 0 ...
            Z(9) 0 rpitch 0 ...
            Z(10) 0 rpitch 0 ...
            Z(11) 0 rpitch 0]';

X = [Z(1) 0 Z(2)' 0 0 0 Z(3) 0 Z(4)]';

if abs(fval) < 1e-6
    disp(['Successfully trimmed at ' mat2str(X)])
else
    disp(['Unable to trim at ' mat2str(X)])
    disp(['for trajectory ' mat2str(traj)])
    failed = 1;
end

if abs(fval) > 4
    disp("catch")
end

%% Get Forces for Optimum Control
V = sqrt(X(1)^2 + X(2)^2 + X(3)^2);
alpha = atan2(X(3), X(1));
beta = atan2(X(2), X(1));

X(8) = alpha + traj(1);

[Fb, Mcg, forces, aero, uvw_e, MTcg] = aeroDyn(coeff, U', alpha, beta, V, rho, X, refGeo, m, thrustIn, cg);
end