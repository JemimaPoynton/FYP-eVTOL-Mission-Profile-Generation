function xdot = explicitFO(Fb, Mcg, I, X, m)
% function explicitFO calculates the state derivatives in explicit first
% order form at state X under body forces Fb and moments Mcg

xdot = zeros(1,9); % Initiating x_dot matrix

uvw = X(1:3);
pqr = X(4:6);

W = [1, sin(X(7))*tan(X(8)),  cos(X(7))*tan(X(8));
        0, cos(X(7))          , -sin(X(7));
        0, sin(X(7))/cos(X(8)),  cos(X(7))/cos(X(8))];

xdot(1:3) = (1/m)*Fb - cross(pqr, uvw); % uvw_dot
xdot(4:6) = inv(I)*(Mcg - cross(pqr, I*pqr)); % pqr_dot
xdot(7:9) = W*pqr; % theta, psi, phi_dot