%% Get Points for Reference
load('trimUAM1', 'trim')

X = reshape(trim.X,size(trim.X,1),[]); % reshape to linear indices (no stages)
U = reshape(trim.U,size(trim.U,1),[]);
Ut = [U(1:3,:); reshape(trim.Ut, size(trim.Ut,1),[])];

idx = [5, 119, 91, 68; 
       135, 31, 59, 73;
       148, 38, 98, 80;
       140, 110, 54, 86 ]; % indices of analysis points (1 sample from each mode)

sysMat = lineariseTrimFull(VX4, VX4.refGeo, coefficients, 'trimUAM1', 1e-5);

Xes = X(:,1);

%% Iterate
sysMat_LQI = augmentSys(sysMat);
[Q, R] = solveQR_LQI(sysMat_LQI, idx, X, Ut);

% Save Results in File
save('QRvalsLQI_new.mat', 'Q', 'R', 'idx')

%% Create Gain Matrix
Kpi = createGainMat(sysMat_LQI, 'QRvalsLQI_new.mat', 'trimUAM1');


%% Iterate
[Q, R] = solveQR(sysMat, idx, X, Ut);

% Save Results in File
save('QRvalsLQR_new.mat', 'Q', 'R', 'idx')

%% Create Gain Matrix
K = createGainMat(sysMat, 'QRvalsLQR_new.mat', 'trimUAM1');
% X = Xint;

%% Mission Runtime
missionruntime = mission;
missionruntime = rmfield(missionruntime, 'modes');
missionruntime = rmfield(missionruntime, 'rotorTilt');

%% 
t = linspace(0, 287.5, size(X,2));
tl = linspace(0, 287.5, 150);

t1 = linspace(0, 40, 31);
t2 = linspace(t1(end), t1(end) + 24.504, 31);
t3 = linspace(t2(end), t2(end) + 37.908, 31);
t4 = linspace(t3(end), t3(end) + 96.15, 31);
t5 = linspace(t4(end), t4(end) + 44.05, 31);

t = [t1(1:end-1) t2(1:end-1) t3(1:end-1) t4(1:end-1) t5(1:end-1)];

xvec = zeros(9,150);
uvec = zeros(9,150);

for i = 1:9
    xvec(i,:) = interp1(t, X(i,:), tl);
end

for i = 1:9
    uvec(i,:) = interp1(t, Ut(i,:), tl);
end

Xes = X(:,1);

%% Create Augmented Control With Integral
Kpi = createGainMatLQI(sysMat, 'QRvalsLQI.mat', 'trimUAM1');
% %K = Kpi(:,1:9,:);
% Ki = Kpi(:,10:12,:);
