idx = 1;
stg = 1;

Xe = X(:,idx+1); % desired equilibrium state
Xes = X(:,idx); % current equilibrium state
Ue = U(:,idx+1); % baseline trim input 

sys = sysMat{idx}; % extract system
K = lqr(sys, eye(9), eye(7));

%% Plot output stuff
% !Probably worth adding some sample rate here!
clear Ur
for i = 1:length(out.simout.Data(:,1)) 
    Ur(:,i) = torqueForce2U(out.simout.Data(i,4:6)', out.simout.Data(i,7:9)', thrustobj2struct(aircraft, zeros(1,4)), U(:,2), 3, 4, 150, VX4.CG);
end

plot(out.simout.Time, Ur)