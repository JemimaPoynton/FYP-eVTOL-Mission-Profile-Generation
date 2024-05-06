function J = trimCost_ga(aircraft, Z, u, coeff, rho, refGeo, m, thrust, cg, I, traj, rpitch, ub)
% function trimCost calculates the value of the cost function J given the
% states/inputs stored in Z = [X U]
%
% demand: demand velocity in the earth frame

%% Extract States
Rrb = transMatrix([0 rpitch 0]);
dist = (thrust.xyz_tr - cg)*Rrb; % distance from cg perpendicular to thrust vector
br = sqrt(abs(dist(3,1))/abs(dist(1,1)));

U = [Z(7:9) br*Z(10) 0 rpitch 0 ...
            br*Z(10) 0 rpitch 0 ...
         Z(10) 0 rpitch 0 ...
         Z(10) 0 rpitch 0]';

X = [Z(1) 0 Z(3) 0 0 0 Z(4) Z(5) Z(6)]';

%% Calculate Airflow
% Assuming no wind disturbance

V = sqrt(X(1)^2 + X(2)^2 + X(3)^2);
alpha = atan2(X(3), X(1));
beta = atan2(X(2), X(1));

%% Calculate State Derivatives
[Fb,Mcg,~,~,~,MTcg] = aeroDyn_ind(aircraft, coeff, U', rho, X, refGeo, m, thrust, cg);
xdot = explicitFO(Fb, Mcg, I, X, m);

V = sqrt(X(1)^2 + X(2)^2 + X(3)^2);

Rbe = body2earth(X(7), X(8), X(9));
uvw_e = Rbe*X(1:3);

gamma = X(8) - alpha;
ca = X(9) + beta;

Jtraj = (gamma - traj(1))^2 + (ca - traj(2))^2 + (V - u)^2;
Jtrim = sum(xdot(1:6).^2)+ 0.001*sum(MTcg.^2); % cost function

J = Jtrim + Jtraj;
end