function [z0, fval] = guess(func, lb, ub, n, Va)
% function guess uses a genetic algorithm to find the approximate location
% of the global minima as an intial guess for z0 

opts = optimoptions('particleswarm', 'FunctionTolerance', 1e-9, 'MaxIterations', 10000, 'SwarmSize', 300, 'ObjectiveLimit', 1e-3, 'MaxStallTime', 3);
opts.MinNeighborsFraction = 0.5;

[z0, fval] = particleswarm(func, n, lb(1:10), ub(1:10), opts);

for i = 1:5
    if fval > 1e-3
        [z0, fval] = particleswarm(func, n, lb(1:10), ub(1:10), opts);
    end
end

if fval > 1 % if still cannot be resolved, set a default start point
   z0 = zeros(size(z0));
   z0(1) = Va;
end

end