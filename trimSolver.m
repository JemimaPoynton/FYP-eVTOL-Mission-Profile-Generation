function [U, X, forces, aero, uvw_e, alpha, MTcg, failed] = trimSolver(aircraft, coeff, rho, Va, traj, rpitch, alphaLim, prev, ubR)
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

%% Define Saturation
uwRatio = atan(alphaLim);

if rpitch > pi/2 - 0.05
    diff = interp1([pi/2 - 0.05, pi/2], [0.01 0], rpitch);
    pitchlim = [1-diff 1+diff];
else
    pitchlim = [0.99 1.01];
end
lb = [0 -0.25*Va -Va*uwRatio -pi/4 -pi/4 -pi/4 -0.52 -0.52 0 0 0 0 0 0 0 pitchlim(1)*rpitch]';
ub = [+Va +0.25*Va +Va*uwRatio +pi/4 +pi/4 +pi/4 0.52 0.52 ubR ubR ubR ubR ubR 17000 17000 pitchlim(2)*rpitch]';

%% Define Anonymous Function
func = @(Z)trimCost(aircraft, Z, Va, coeff, rho, refGeo, m, thrustIn, cg, I, traj, rpitch, ub, prev);
% trCon = @(U)

%% Solve
opts = optimset('PlotFcns','optimplotfval','TolX',1e-9, 'MaxIter', 100000, 'MaxFunEvals', 1e4, 'ObjectiveLimit', 1e-9);

z0p_func = @(Z)trimCost_ga(aircraft, Z, Va, coeff, rho, refGeo, m, thrustIn, cg, I, traj, rpitch, ub, prev);
z0p = guess(z0p_func, lb, ub, 12, Va)';

Rrb = transMatrix([0 z0p(12) 0]);
dist = (thrustIn.xyz_tr - cg)*Rrb;
br = sqrt(abs(dist(3,1))/abs(dist(1,1)));

z0 = zeros(1,15)';
% z0(1) = Va;
z0(1:9) = z0p(1:9);
z0([10 11]) = z0p(10);
z0([12 13]) = z0(10)*br;
z0([14 15]) = z0p(11);
% z0(end) = z0p(11);
z0(16) = z0p(12);

[Z, fval] = fmincon(func, z0, [], [], [], [], lb, ub, [], opts);
% if fval > 0.01
%     opts = optimoptions('patternsearch','PlotFcn', @psplotbestf, 'FunctionTolerance', 1e-62 ...
%     , 'MaxIterations', 50000, 'MaxFunctionEvaluations', 50000);
% 
%     [Z, fval] = patternsearch(func, Z, [], [], [], [], lb, ub, [], opts);
% end

%% Check Influence of u
% check if control surfaces are influencing trim, if not, set to zero.
% Handles low speed cases where deflection is arbritrary.
J = func(Z);

for i = 1:nC
    Zu = Z;

    Zu(i + 6) = 1.9.*Zu(i + 6);
    Ju = func(Zu);

    if abs(Ju-J) < 1e-12
        Z(i + 6) = 0;
    end
end

%% Check Convergence
U = [Z(7:8)' Z(10) 0 0 0 ...
             Z(11) 0 0 0 ...
             Z(12) 0 0 0 ...
             Z(13) 0 0 0 ...
             Z(14) 0 Z(16) 0 ...
             0 0 Z(16) 0]';

X = [Z(1) 0 Z(3)' 0 0 0 0 Z(5)' Z(6)]';

[Jtrim, Jraj] = trimCostOutputs(aircraft, Z, Va, coeff, rho, refGeo, m, thrustIn, cg, I, traj, rpitch, ub, prev);

if abs(Jtrim) < 1e-3
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
alpha = atan2(X(3), X(1));

[Fb, Mcg, forces, aero, uvw_e, MTcg] = aeroDyn_ind(aircraft, coeff, U', rho, X, refGeo, m, thrustIn, cg);
end