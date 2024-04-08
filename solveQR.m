function [Q, R] = solveQR(sysMat, idx, X, U, ub_)
% function solveQR determines the optimum Q and R matrices for LQR
% controller design for the points idx in the matrix of linearised systems
% in sysMat, where idx represents 1 point on each mode (VTOL/transition/cruise)

%% Setup
if ~exist('ub', 'var')
    ub_ = 30; % default
end

for i = 1:length(idx)
    Xe = X(:,idx(i)+1); % desired equilibrium state
    Xes = X(:,idx(i)); % current equilibrium state
    Ue = U(:,idx(i)+1); % baseline trim input 

    assignin('base','Xe',Xe)
    assignin('base', 'Xes', Xes)
    assignin('base', 'Ue', Ue)
    
    sys = sysMat{idx(i)+1}; % extract system
    
    opts = optimoptions('particleswarm', 'Display','iter','PlotFcn','pswplotbestf', ...
        'SwarmSize', 100, 'MaxIterations', 400);

    lb = zeros(size(sys.A,1),1);
    ub = ones(size(sys.A,1),1)*ub_;
    
    func = @(QR)QRerr(QR, sys);
    QR(i,:) = particleswarm(func, size(sys.A,1), lb, ub, opts);

    Q(:,:,i) = eye(size(sys.A,1)).*QR(i,1:size(sys.A,1))';
    R(:,:,i) = eye(size(sys.B,2)).*1';  % extract from optimisation
end
