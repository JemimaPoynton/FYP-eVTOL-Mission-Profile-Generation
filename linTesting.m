%% Transition Point
sys = sysMat{21};

Xes = X(:,20);
Ue = Ut(:,21);
Xe = X(:,21);

K = lqr(sys, 100.*eye(9), eye(9));

C = zeros(3,size(sys.C,2)); C(1:3,1:3) = eye(3); C(4:6,7:9) = eye(3);

Aa = [sys.A zeros(9,6);
      -C    zeros(6,6)];

Ba = [sys.B; zeros(6, size(sys.B,1))];

Ca = zeros(15,15); Ca(1:9,1:9) = eye(9);

Da = [];

Ra = R(:,:,1);
Qa = eye(15); Qa(1:9,1:9) = Q(:,:,1);

sys_a = ss(Aa, Ba, Ca, Da);
Kpi = lqr(sys_a, Qa, Ra,2*ones(15,9));

%% VTOL Mode
sys = sysMat{1};

Xes = X(:,1);
Ue = U(:,1);
Xe = X(:,1);

sys = ss(sys.A, sys.B(:,[4 8 12 16]), sys.C, sys.D(:,[4 8 12 16]));

Q = 1.*eye(9);
K = lqr(sys, Q, eye(4));
