function Ur = torqueForce2U(Fd, MTd, thrust, u, nc, nT, m, cg)
% function torqueForce2U determines the U demand in the form [w1 ... wm]',
% where wm is the rotational velocity of the mth rotor, to meet a demand
% force and torque Fd and MTd. 

A = zeros(6,length(nT));
FTb_w2 = zeros(3,length(nT));
MTcg_w2 = zeros(3,length(nT));


%% Form Simultaneous Equations
for i = 1:nT
    kt = thrust.kt(i,:);
    kb = thrust.kb(i,:);
    dir = thrust.dir(i,:);

    idx = nc + 1 + (i-1)*4; % index of thrust-deflection pairs for thrust objects
    FTt_w2 = [ 0  ; % Thrust in rotor plane divided by w^2
               0  ;
              -kt];

    MTy_w2 = [0      ;
              0      ;
              dir*kb];
    
    Rrb = transMatrix([u(idx+1) u(idx+2) u(idx+3)].*[1 1 1]); % rotation matrix from rotor plane to body, first converting to geometric axis

    FTb_w2(:,i) = Rrb*FTt_w2; % Rotate to body frame, switching from geometric to dynamic coordinates
    MTcg_w2(:,i) = Rrb*MTy_w2 + cross(thrust.xyz_tr(i,:) - cg, FTb_w2(:,i))';

    A(:,i) = [FTb_w2(:,i);
              MTcg_w2(:,i)];
end

B = [Fd ;
     MTd];

%% Solve Simultaneous Equations
B(find(abs(B) < 1e-6)) = 1e-6; % handle rank deficiency

Alim = zeros(12, nT);

pairs = [];
pattern = [1 -1; -1 1];
for i = 1:nT-1
    for j = i+1:nT
        pairs = [pairs; i, j];
    end
end

for i = 1:6
    idx = 1 + 2*(i-1);
    Alim(idx:idx+1, pairs(i,:)) = pattern;
end

Blim = 0.2*ones(size(Alim,1),1);
ub = 2.2/nT*m*9.81/kt*ones(1, size(A,2));
lb = 0.6/nT*m*9.81/kt*ones(1, size(A,2));

[x, fval] = lsqlin(A, B, [], [], [] ,[], lb, ub);
Ur = sqrt(abs(x)); % convert from w^2 to w