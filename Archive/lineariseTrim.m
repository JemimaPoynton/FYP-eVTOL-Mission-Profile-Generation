function [A, B] = lineariseTrim(func, xref, uref, dx, aircraft, highOrd)
% function lineariseTrim calculates the A and B matrices of the state-space
% form of the system linearised about the trim point xref, uref for the
% dynamics defined by func

nx = length(xref);
nu = length(uref);

if ~exist('highOrd', 'var') % default highest order of Taylors series
    highOrd = 1;
end

%% Calculate A
func_x = @(x)func(x, uref); % define function for variation in x

A = zeros(nx, nx);

for i = 1:nx
    A(:,i) = dfdx(func_x, xref, dx, aircraft, i, highOrd); 
end

%% Calculate B
func_u = @(u)func(xref, u); % define function for variation in u

B = zeros(nx, nu);

for j = 1:nu
    B(:,j) = dfdx(func_u, uref, dx, aircraft, j, highOrd); 
end
end
