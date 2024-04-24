function [z0, fval] = guess(func, lb, ub, n, Va)
% function guess uses a genetic algorithm to find the approximate location
% of the global minima as an intial guess for z0 

opts = optimoptions('particleswarm', 'FunctionTolerance', 1e-12, 'MaxIterations', 10000, 'SwarmSize', 700, 'ObjectiveLimit', 1e-6, 'MaxStallTime', 3);
% opts.MinNeighborsFraction = 0.5;

fval = ones(1,6); z0 = ones(n,6);
[z0(:,1), fval(1)] = particleswarm(func, n, lb(1:10), ub(1:10), opts);

for i = 2:6
    if fval(i-1) > 1e-6
        [z0(:,i), fval(:,i)] = particleswarm(func, n, lb(1:10), ub(1:10), opts);
    else
        break
    end
end

[~,idx]= min(fval);
z0 = z0(:,idx);

if fval(idx) > 1
    z0 = zeros(size(z0)); z0(1) = Va;
end

end