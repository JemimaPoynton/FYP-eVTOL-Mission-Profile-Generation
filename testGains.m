idx = 10;
stg = 1;

Xe = X(:,idx+1); % desired equilibrium state
Xes = X(:,idx); % current equilibrium state
Ue = Ut(:,idx+1); % baseline trim input 
% 
% sysMat_LQI = augmentSys(sysMat);
sys = sysMatstr_LQI{idx}; % extract system
Kpi = lqr(sys, eye(15), eye(9));

%% Plot output stuff
% !Probably worth adding some sample rate here!
clear Ur
for i = 1:length(out.simout.Data(:,1)) 
    Ur(:,i) = torqueForce2U(out.simout.Data(i,4:6)', out.simout.Data(i,7:9)', thrustobj2struct(aircraft, zeros(1,4)), U(:,2), 3, 4, 150, VX4.CG);
end

plot(out.simout.Time, Ur)