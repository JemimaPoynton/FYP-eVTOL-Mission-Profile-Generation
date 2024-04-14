%% Get Points for Reference
load('trimUAM1', 'trim')

X = reshape(trim.X,size(trim.X,1),[]); % reshape to linear indices (no stages)
U = reshape(trim.U,size(trim.U,1),[]);
Ut = [U(1:3,:); reshape(trim.Ut, size(trim.Ut,1),[])];

idx = [8, 20, 35, 43; 
       15, 21, 38, 49]; % indices of analysis points (1 sample from each mode)

sysMat = lineariseTrimFull(VX4, referenceGeo, coefficients, 'trimUAM1', 1e-4);

% interpolate between trim points
for i = 1:size(X,1)
    Xint(i,:) = interp1(1:1:size(X,2), X(i,:), linspace(1,size(X,2), 900));
end

Xes = X(:,1);

%% Iterate
[Q, R] = solveQR(sysMat, idx, X, Ut);

%% Save Results in File
save('QRvalsEdgeCases.mat', 'Q', 'R', 'idx')

%% Create Gain Matrix
K = createGainMat(sysMat, 'QRvalsEdgeCases.mat', 'trimUAM1');

% X = Xint;

%% Mission Runtime
missionruntime = mission;
missionruntime = rmfield(missionruntime, 'modes');
missionruntime = rmfield(missionruntime, 'rotorTilt');

%% 
t = linspace(0, mission.secTime(end), size(X,2));
tl = linspace(0, mission.secTime(end), 3000);

xvec = zeros(9,3000);
uvec = zeros(9,3000);

for i = 1:9
    xvec(i,:) = interp1(t, X(i,:), tl);
end

for i = 1:9
    uvec(i,:) = interp1(t, Ut(i,:), tl);
end
