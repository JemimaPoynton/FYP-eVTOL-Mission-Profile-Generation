%% Load Trim Points
% get the trim data xref and uref
load('testData].mat', 'X', 'U')

%% Set Variables
xref = X(:,32);
uref = U(:,32);
dx = 1e-6;

% define dynamics function wrt control input u and state x
func = @(X,u)aeroDyn_ind(coeff, u', 1.225, X, referenceGeo, VX4.m, thrustobj2struct(VX4, zeros(1,4)), VX4.CG);

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