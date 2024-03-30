function [U, forces, aero] = trimSolver(aircraft, coeff, rho, X)
% function trimSolver solves the overactuated trim optimisation problem for
% aircraft with state vector 'X' in the form [u v w p q r psi theta phi]'

% !Extract control surface max deflection, and add limits from config, extract coefficients!

%% Extract Aircraft Data
nT = length(aircraft.thrust.rotors) + length(aircraft.thrust.ducts);
nC = length(coeff.CLn);

I = aircraft.I;
m = aircraft.m;
thrustIn = thrustobj2struct(aircraft, zeros(1,nT));

cg = aircraft.CG;
refGeo = aircraft.refGeo;

%% Define Anonymous Function
func = @(U)trimCost(U, X, coeff, rho, refGeo, m, thrustIn, cg, I);

%% Define Saturation
% !THIS NEEDS TO BE FORMED FOR A GENERIC NUMBER OF ROTORS!
lb = [-0.52 -0.52 -0.52 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';
ub = [0.52 0.52 0.52 3000 0 pi/2 0 3000 0 pi/2 0 3000 0 pi/2 0 3000 0 pi/2 0]';

%% Solve
opts = optimset('PlotFcns','optimplotfval','TolX',1e-12, 'MaxIter', 10000, 'MaxFunEvals', 10000);

z0 = zeros(1, nC + 4*nT)';
z0([4 8 12 16]) = m*9.81/nT; % starting point of balanced weight to help solver converge quicker

[U, fval] = fmincon(func, z0, [], [], [], [], lb, ub, [], opts);

%% Check Convergence
if fval < 1e-3
    disp(['Successfully trimmed at ' mat2str(X)])
else
    disp(['Unable to trim at ' mat2str(X)])
end

%% Get Forces for Optimum Control
V = sqrt(X(1)^2 + X(2)^2 + X(3)^2);
alpha = atan2(X(3), X(1));
beta = asin(X(2)/V);

[~, ~, forces, aero] = aeroDyn(coeff, U', alpha, beta, V, rho, X, refGeo, m, thrustIn, cg);