idx = 15;
stg = 1;

Xe = X(:,idx+1); % desired equilibrium state
Xes = X(:,idx); % current equilibrium state
Ue = U(:,idx+1); % baseline trim input 

sys = sysMat{idx}; % extract system
K = lqr(sys, Q(:,:,stg), R(:,:,stg));