%% Load Trim Points
% get the trim data xref and uref
load('trimUAM1', 'X', 'U')

%% Set Variables
X = reshape(X,size(X,1),[]); % reshape to linear indices (no stages)
U = reshape(U,size(U,1),[]);

xref = X(:,1);
uref = U(:,1);
dx = 1e-3;

% define dynamics function wrt control input u and state x
func = @(X,u)aeroDyn_full(coeff, u', 1.225, X, referenceGeo, VX4.m, thrustobj2struct(VX4, zeros(1,4)), VX4.CG, VX4.I);

%% Run Lineariser
[A, B] = lineariseTrim(func, xref, uref, dx, VX4, 3);

C = eye(9,9);
D = []; % no controls included in output

%% Create State-Space
Bplt = B(:,1); % plot only response to first input
Cplt = C(1,:); % select ouput states for plot

sys = ss(A,B,C,D);
config = RespConfig('Amplitude', 0.05,'Delay',10);
% step(sys, [1,120], config)
grid on

%% Solve LQR for Trim Point