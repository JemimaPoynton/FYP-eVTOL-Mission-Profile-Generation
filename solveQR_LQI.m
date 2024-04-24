function [Q, R] = solveQR_LQI(sysMat, idx, X, U, ub_)
% function solveQR determines the optimum Q and R matrices for LQR
% controller design for the points idx in the matrix of linearised systems
% in sysMat, where idx represents 1 point on each mode (VTOL/transition/cruise)

%% Setup
if ~exist('ub', 'var')
    ub_ = 4000; % default
end

for i = 1:length(idx(1,:))  
    opts = optimoptions('particleswarm', 'Display','iter','PlotFcn','pswplotbestf', ...
                    'FunctionTolerance', 1, 'MaxIterations',8);

    lb = (1e-6)*ones(size(sysMat{1,1}.A,1),1);
    ub = ones(size(sysMat{1,1}.A,1),1)*ub_;
    
    func = @(QR)QRerr_LQI(QR, sysMat, idx(:,i), X, U);
    QR(i,:) = particleswarm(func, size(sysMat{1,1}.A,1), lb, ub, opts);

    Q(:,:,i) = eye(size(sysMat{1,1}.A,1)).*QR(i,1:size(sysMat{1,1}.A,1))';
    R(:,:,i) = eye(size(sysMat{1,1}.B,2)).*1';  % extract from optimisation
end
