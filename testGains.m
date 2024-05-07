idx = 20;
stg = 1;

Xe = X(:,idx+1); % desired equilibrium state
Xes = X(:,idx); % current equilibrium state
Ue = Ut(:,idx+1); % baseline trim input 

sysMat = augmentSys(sysMat);
sys = sysMat{idx}; % extract system
Kpi = lqr(sys, Q(:,:,1), R(:,:,1));

%% Plot output stuff
% !Probably worth adding some sample rate here!
clear Ur
for i = 1:length(out.simout.Data(:,1)) 
    Ur(:,i) = torqueForce2U(out.simout.Data(i,4:6)', out.simout.Data(i,7:9)', thrustobj2struct(aircraft, zeros(1,4)), U(:,2), 3, 4, 150, VX4.CG);
end

plot(out.simout.Time, Ur)