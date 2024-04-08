%% Get Points for Reference
load('trimUAM1', 'trim')

X = reshape(trim.X,size(trim.X,1),[]); % reshape to linear indices (no stages)
U = reshape(trim.U,size(trim.U,1),[]);

idx = [7 21 36]; % indices of analysis points (1 sample from each mode)

sysMat = lineariseTrimFull(VX4, referenceGeo, coeff, 'trimUAM1', 1e-3);

% interpolate between trim points
for i = 1:size(X,1)
    Xint(i,:) = interp1(1:1:size(X,2), X(i,:), linspace(1,size(X,2), 900));
end

Xes = X(:,1);

%% Iterate
[Q, R] = solveQR(sysMat, idx, X, U);

%% Save Results in File
save('QRvals.mat', 'Q', 'R', 'idx')

%% Create Gain Matrix
K = createGainMat(sysMat, 'QRvals.mat', 'trimUAM1');

% X = Xint;

%% Mission Runtime
missionruntime = mission;
missionruntime = rmfield(missionruntime, 'modes');
missionruntime = rmfield(missionruntime, 'rotorTilt');