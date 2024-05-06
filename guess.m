function [z0, fval] = guess(func, lb, ub, n, Va)
% function guess uses a genetic algorithm to find the approximate location
% of the global minima as an intial guess for z0 

opts = optimoptions('particleswarm', 'FunctionTolerance', 1e-6, 'MaxIterations', 10000, 'SwarmSize', 200, 'ObjectiveLimit', 1e-3, 'MaxStallTime', 3);
opts.InertiaRange = [0.4 1.9]; % high inertia range upper bound for better global search capability

z0 = zeros(n,6); fval = 1000*ones(1,6);
[z0(:,1), fval(:,1)] = particleswarm(func, n, lb([1:10 10]), ub([1:10 10]), opts);

for i = 2:5
    if fval(i-1) > 1e-3
        opts.SwarmSize = 200 + i*60;
        opts.InertiaRange = [0.4 2 + i*0.1];
        [z0(:,i), fval(:,i)] = particleswarm(func, n, lb([1:10 10]), ub([1:10 10]), opts);
    else
        break
    end
end

[~,idx] = min(fval);
z0 = z0(:,idx);

if fval(idx) > 1 % if still cannot be resolved, set a default start point
   z0 = zeros(size(z0));
   z0(1) = Va;
end

end