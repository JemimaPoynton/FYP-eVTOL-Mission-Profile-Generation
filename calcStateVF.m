function [Xn, idx] = calcStateVF(mission, trim, Xc, p, kc, kg, ca_inf, g_ainf)
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

%% Determine Current and Next Waypoint
i = []; % index in 'continuous mission path'
j = [];

%% Calculate Vector Field
wpi = mission.xyz(i); % current waypoint
wpj = mission.xyz(j); % next waypoint

ld = (wpj - wpi)/norm(wpj - wpi); % line direction
pl = (eye(3)-ld*ld')*(p - wpi); % path orthogonal error

cal = atan2([0 1 0]'*wpj, [1 0 0]'*wpi);  % course angle of line path

RIs = [ cos(cal), sin(cal), 0;
       -sin(cal), cos(cal), 0;
        0       , 0       , 1]; % rotation matrix from inertial to straight line path reference framer

cad = cal - ca_inf*(2/pi)*atan(kc*[0 1 0]'*RIs*pl); % course angle demand to stay on path

%% Calculate Demand
Xn = []; % demand state

%% Evaluate Deviation From Current State and Set Index
idx = []; % index associated with closest demand-trim state