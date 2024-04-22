function J = QRerr_p(QR, sysMat, idx, X, U)
%% Setup Q and R
Q = sysMat{1}.C'*sysMat{1}.C;
Q(7:12,7:12) = Q(7:12,7:12).*QR';
R = eye(size(sysMat{1,1}.B,2)).*1';

W = [2 2 2   4 4 4]; % Error weighting matrix

%% Calculate Gain Matrix
try
    Xe = X(:,idx(1)+1); % desired equilibrium state
    Xes = X(:,idx(1)); % current equilibrium state
    Ue = U(:,idx(1)+1); % baseline trim input 

    assignin('base','Xe',Xe)
    assignin('base', 'Xes', Xes)
    assignin('base', 'Ue', Ue)
    
    sys = sysMat{idx(1)+1}; % extract system

    K = lqr(sys, Q, R);

    assignin('base','K',K)
    %% Run model and Output Integrated Error
    warning('off', 'MATLAB:callback:error') % supress warnings due to model input callbacks
    warning('off', 'Stateflow:translate:SFcnBlkNotTunableParamChangeFastRestart')

    tic1 = tic;
    out1 = sim('Cruise_Model_V8_QRTuning_FM.slx');
    toc1 = toc(tic1);

    Xe = X(:,idx(2)+1); % desired equilibrium state
    Xes = X(:,idx(2)); % current equilibrium state
    Ue = U(:,idx(2)+1); % baseline trim input 

    assignin('base','Xe',Xe)
    assignin('base', 'Xes', Xes)
    assignin('base', 'Ue', Ue)
    
    sys = sysMat{idx(2)+1}; % extract system

    K = lqr(sys, Q, R);

    assignin('base','K',K)

    tic2 = tic;
    out2 = sim('Cruise_Model_V8_QRTuning_FM.slx');
    toc2 = toc(tic2);

    warning('on', 'MATLAB:callback:error')
    warning('on', 'Stateflow:translate:SFcnBlkNotTunableParamChangeFastRestart')

    [~, idx1] = min(abs(out1.error.Time - 5));
    [~, idx2] = min(abs(out2.error.Time - 5));

    if toc1 >= 10 || toc2 >= 10
        J = 1e9;
    else
        J = abs(sum((out1.error.Data(end,7:12) - (out1.error.Data(idx1,7:12))).*W)) + ...
        abs(sum((out2.error.Data(end,7:12) - (out2.error.Data(idx2,7:12))).*W));
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



