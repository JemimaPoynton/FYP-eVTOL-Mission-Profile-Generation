function [Xn, idx] = calcStateVF(mission, trim, Xc, p, kc, kg, ca_inf, g_inf)
% function calcStateVF calculates the required state to stay on the path
% defined in mission by applying vector fields
% 
% p: current aircraft postion
% cainf: maximum allowable difference between course angle of straight line
%        path and demand

if ~exist('ca_inf','var') % set default
    ca_inf = 0.2;
elseif isempty(ca_inf)
    ca_inf = 0.2;
end

if ~exist('g_inf','var') % set default
    g_inf = 0.2;
elseif isempty(g_inf)
    g_inf = 0.2;
end

%% Calculate Vector Field
[ca_d, gam_d] = createDemandVecVF(mission, p, kc, kg, ca_inf, g_inf);

%% Calculate Demand
Xn = []; % demand state

%% Evaluate Deviation From Current State and Set Index
idx = []; % index associated with closest demand-trim state