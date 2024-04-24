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

lb = [0 -0.25*Va -Va*uwRatio -pi/4 -pi/4 -pi/4 -0.52 -0.52 -0.52 0 0 0 0]';
ub = [+Va +0.25*Va +Va*uwRatio +pi/4 +pi/4 +pi/4 0.52 0.52 0.52 7000 7000 7000 7000]';

%% Solve
opts = optimset('PlotFcns','optimplotfval','TolX',1e-30, 'MaxIter', 100000, 'MaxFunEvals', 100000);

z0p_func = @(Z)trimCost_ga(Z, Va, coeff, rho, refGeo, m, thrustIn, cg, I, traj, rpitch);
z0p = guess(z0p_func, lb, ub, 11, Va)';

z0 = zeros(1,13)';
z0(1:9) = z0p(1:9);
z0(end-3:end) = z0p(10);

[Z, fval] = fmincon(func, z0, [], [], [], [], lb, ub, [], opts);

% if fval > 0.01
%     opts = optimoptions('patternsearch','PlotFcn', @psplotbestf, 'FunctionTolerance', 1e-62 ...
%     , 'MaxIterations', 50000, 'MaxFunctionEvaluations', 50000);
% 
%     [Z, fval] = patternsearch(func, Z, [], [], [], [], lb, ub, [], opts);
% end

%% Check Convergence
X = [Z(1) 0 Z(3)' 0 0 0 Z(4:6)']';
U = [Z(7:9)' Z(10) 0 rpitch 0 ...
            Z(11) 0 rpitch 0 ...
            Z(12) 0 rpitch 0 ...
            Z(13) 0 rpitch 0]';

if abs(fval) < 1e-3
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

[Fb, ~, forces, aero, uvw_e, MTcg] = aeroDyn(coeff, U', alpha, beta, V, rho, X, refGeo, m, thrustIn, cg);
end