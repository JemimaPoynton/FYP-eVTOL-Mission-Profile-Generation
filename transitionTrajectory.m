
%% Define Anonymous Function
rho = 1.225;
m = VX4.m;
thrustin.Tinit = [0 0 0 0];
thrustin.xyz_tr = [0 3 0.3;0 -3 0.3;2.2 3 -0.3;2.2 -3 -0.3];
I = VX4.I;

refGeo = referenceGeo;
cg = VX4.CG;
coeff = coeff; % placeholder
X = [20 0 0 0 0 0 0 0 0]';

func = @(U)trimCost(U, X, coeff, rho, refGeo, m, thrustin, cg, I);

%% Define Saturation
lb = [-0.5 -0.5 -0.5 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';
ub = [0.5 0.5 0.5 3000 pi/2 pi/2 pi/2 3000 pi/2 pi/2 pi/2 3000 pi/2 pi/2 pi/2 3000 pi/2 pi/2 pi/2]';

%% Solve 
nT = length(VX4.thrust.rotors) + length(VX4.thrust.ducts);
nC = length(coeff.CLn);

opts = optimset('PlotFcns','optimplotfval','TolX',1e-6, 'MaxIter', 5000, 'MaxFunEvals', 5000);

z0 = zeros(1, nC + 4*nT)';
z0([4 8 12 16]) = 150*9.81/4;

[u, fval] = fmincon(func, z0, [], [], [], [], lb, ub, [], opts);
