function J = trimCost(U, X, coeff, rho, refGeo, m, thrust, cg, I)
% function trimCost calculates the value of the cost function J given the
% states/inputs stored in Z = [X U]
%
% demand: demand velocity in the earth frame

%% Calculate Airflow
% Assuming no wind disturbance

V = sqrt(X(1)^2 + X(2)^2 + X(3)^2);
alpha = atan2(X(3), X(1));
beta = asin(X(2)/V);

%% Calculate State Derivatives
[Fb,Mcg] = aeroDyn(coeff, U', alpha, beta, V, rho, X, refGeo, m, thrust, cg);
xdot = explicitFO(Fb, Mcg, I, X, m);

Rbe = body2earth(X(7), X(8), X(9));
uvw_e = Rbe*X(1:3);

J = sum(xdot(1:6).^2); % cost function