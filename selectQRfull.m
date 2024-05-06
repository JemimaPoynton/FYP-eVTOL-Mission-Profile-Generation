%% Get Points for Reference
load('trimUAM1', 'trim')

X = reshape(trim.X,size(trim.X,1),[]); % reshape to linear indices (no stages)
U = reshape(trim.U,size(trim.U,1),[]);
Ut = [U(1:3,:); reshape(trim.Ut, size(trim.Ut,1),[])];

idx = [10, 36, 45, 61; 
       20, 40, 55, 86]; % indices of analysis points (1 sample from each mode)

sysMat = lineariseTrimFull(VX4, VX4.refGeo, coefficients, 'trimUAM1', 1e-5);

% Xes = X(:,1);

%% Iterate for LQI
sysMat_LQI = augmentSys(sysMat);
[Q, R] = solveQR_LQI(sysMat_LQI, idx, X, Ut);

%% Save Results in File
save('QRvalsLQI.mat', 'Q', 'R', 'idx')

%% Iterate for LQR
[Q, R] = solveQR(sysMat, idx, X, Ut);

%% Save Results in File
save('QRvalsLQR.mat', 'Q', 'R', 'idx')

%% Create Gain Matrix
Kpi = createGainMat(sysMat_LQI, 'QRvalsLQI.mat', 'trimUAM1');
K = createGainMat(sysMat, 'QRValsLQR', 'trimUAM1');

%% Mission Runtime
missionruntime = mission;
missionruntime = rmfield(missionruntime, 'modes');
missionruntime = rmfield(missionruntime, 'rotorTilt');

%% Setup time in each stage
t = linspace(0, mission.secTime(end), size(X,2));
tl = linspace(0, mission.secTime(end), 10000);

xvec = zeros(9,10000);
uvec = zeros(9,10000);

for i = 1:9
    xvec(i,:) = interp1(t, X(i,:), tl);
end

for i = 1:9
    uvec(i,:) = interp1(t, Ut(i,:), tl);
end

Xes = X(:,1);