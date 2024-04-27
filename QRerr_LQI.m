function J = QRerr_LQI(QR, sysMat, idx, X, U)
%% Setup Q and R
Q = eye(size(sysMat{1,1}.A,1)).*QR(1:size(sysMat{1,1}.A,1))';
R = eye(size(sysMat{1,1}.B,2)).*1';

W = [2 2 2   1 1 1   4 4 4]; % Error weighting matrix

%% Calculate Gain Matrix
try
    Xe = X(:,idx(1)+1); % desired equilibrium state
    Xes = X(:,idx(1)); % current equilibrium state
    Ue = U(:,idx(1)+1); % baseline trim input 

    assignin('base','Xe',Xe)
    assignin('base', 'Xes', Xes)
    assignin('base', 'Ue', Ue)
    
    sys = sysMat{idx(1)+1}; % extract system

    Kpi = lqr(sys, Q, R);

    assignin('base','Kpi',Kpi)
    %% Run model and Output Integrated Error
    warning('off', 'MATLAB:callback:error') % supress warnings due to model input callbacks
    warning('off', 'Stateflow:translate:SFcnBlkNotTunableParamChangeFastRestart')

    tic1 = tic;
    out1 = sim('Cruise_Model_V8_QRTuning_LQI.slx');
    toc1 = toc(tic1);

    Xe = X(:,idx(2)+1); % desired equilibrium state
    Xes = X(:,idx(2)); % current equilibrium state
    Ue = U(:,idx(2)+1); % baseline trim input 

    assignin('base','Xe',Xe)
    assignin('base', 'Xes', Xes)
    assignin('base', 'Ue', Ue)
    
    sys = sysMat{idx(2)+1}; % extract system

    Kpi = lqr(sys, Q, R);

    assignin('base','Kpi',Kpi)

    tic2 = tic;
    out2 = sim('Cruise_Model_V8_QRTuning_LQI.slx');
    toc2 = toc(tic2);

    warning('on', 'MATLAB:callback:error')
    warning('on', 'Stateflow:translate:SFcnBlkNotTunableParamChangeFastRestart')

    [~, idx1] = min(abs(out1.error.Time - 0.01));
    [~, idx2] = min(abs(out2.error.Time - 0.01));

    if toc1 >= 7 || toc2 >= 7
        J = 1e9;
    else
        J = abs(sum((out1.error.Data(end,:) - (out1.error.Data(idx1,:))).*W)) + ...
        abs(sum((out2.error.Data(end,:) - (out2.error.Data(idx2,:))).*W));
    end

    try % check for singularities in range
        for i = 1:length(sysMat)
            lqr(sysMat{i,1}, Q, R);
        end
    catch
        J = 1e9; % check solution is acceptable for lqr
    end
catch
    J = 1e9; % punish invalid solutions (not positive semi-definite) 
end


