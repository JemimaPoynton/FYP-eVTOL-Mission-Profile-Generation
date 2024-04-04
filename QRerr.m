function J = QRerr(QR, sys)
%% Setup Q and R
Q = eye(size(sys.A,1)).*QR(1:size(sys.A,1))';
R = eye(size(sys.B,2)).*1';

W = [0.5 0.5 0.5   4 4 4   4 4 4]; % Error weighting matrix

%% Calculate Gain Matrix
try
    K = lqr(sys, Q, R);
    assignin('base','K',K)
    %% Run model and Output Integrated Error
    warning('off', 'MATLAB:callback:error') % supress warnings due to model input callbacks
    warning('off', 'Stateflow:translate:SFcnBlkNotTunableParamChangeFastRestart')

    out = sim('Cruise_Model_V8_QRTuning.slx');

    warning('on', 'MATLAB:callback:error')
    warning('on', 'Stateflow:translate:SFcnBlkNotTunableParamChangeFastRestart')

    [~, idx] = min(out.error.Time - 15); % allow 15s settling time
    J = sum((out.error.Data(end,:) - out.error.Data(idx,:)).*W);
catch
    J = 1e9; % punish invalid solutions (not positive semi-definite) 
end



