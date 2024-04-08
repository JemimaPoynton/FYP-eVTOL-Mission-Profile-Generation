%% Get Points for Reference
load('trimUAM1', 'X', 'U')

X = reshape(X,size(X,1),[]); % reshape to linear indices (no stages)
U = reshape(U,size(U,1),[]);

Xe = X(:,5);
Xes = X(:,4);
Ue = U(:,5);

%% Linearise Trim Points
sysMat = lineariseTrimFull(VX4, referenceGeo, coeff, 'trimUAM1', 1e-3);
sys = sysMat{5};

%% 
opts = optimoptions('particleswarm', 'Display','iter','PlotFcn','pswplotbestf', ...
    'SwarmSize', 100, 'MaxIterations', 400);
lb = zeros(size(sys.A,1),1);
ub = ones(size(sys.A,1),1)*30;

func = @(QR)QRerr(QR, sys);
QR = particleswarm(func, (size(sys.A,1)), lb, ub, opts);

% [sol, fval] = fmincon(func, QR, [], [], [], [], lb, ub);