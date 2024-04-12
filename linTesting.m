%% Transition Point
sys = sysMat{15};

Xes = X(:,15);
Ue = U(:,16);
Xe = X(:,16);

sys = ss(sys.A, sys.B(:,[1:4 8 12 16]), sys.C, sys.D(:,[1:4 8 12 16]));

K = lqr(sys, 100.*eye(9), eye(7));

%% VTOL Mode
sys = sysMat{1};

Xes = X(:,1);
Ue = U(:,1);
Xe = X(:,1);

sys = ss(sys.A, sys.B(:,[4 8 12 16]), sys.C, sys.D(:,[4 8 12 16]));

Q = 1.*eye(9);
K = lqr(sys, Q, eye(4));
