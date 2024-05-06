function xdot = getxd(func, I, m, X)
% function get xd derives the state derivative for the dynamic system
% specified by func

[Fb,Mcg] = func(X);
xdot = explicitFO(Fb, Mcg, I, X, m); % apply explicit first-order form