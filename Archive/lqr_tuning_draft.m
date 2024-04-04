%% Linearise System
xref = ones(9,1);
uref = ones(19,1);
dx = 0.1;

func = @(X,u)aeroDyn_ind(coeff, u, 1.225, X, referenceGeo, VX4.m, thrustobj2struct(VX4, zeros(1,4)), VX4.CG);

[A, B] = lineariseTrim(func, xref, uref, dx);

%% Solve for Q, R

%% Create Gain Matrix