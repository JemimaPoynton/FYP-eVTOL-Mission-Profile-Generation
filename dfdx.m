function J = dfdx(func, xref, dx_s, aircraft, idx, highOrd)
% function dfdx returns the Jacobian element partial differential dfi/dxj
% or dfi/dUj (whichever is assigned to func)

%% Form dx Matrix
dx = zeros(size(xref));
dx(idx) = dx_s;

%% Calculate Derivative
% Calculate first perturbation points
x1 = xref + dx;
x_1 = xref - dx;

try
    xd1 = func(x1); % calculate corresponding xdot
    xd_1 = func(x_1);
catch
    disp('')
end

if highOrd == 1 % handle different Taylor series approximations
    J = (xd1 - xd_1)./(2*dx_s);

elseif highOrd == 3
    x2 = xref + 2*dx;
    x_2 = xref - 2*dx;

    xd2 = getxd(func, aircraft.I, aircraft.m, x2);
    xd_2 = getxd(func, aircraft.I, aircraft.m, x_2);

    % Jacobian go brr
    J = (8*(xd1 - xd_1) - (xd2 - xd_2))/(12*dx_s);
else
    % Limited range, but may later be expanded
    error('HighOrd setting invalid. Must be in range [1 3].') 
end