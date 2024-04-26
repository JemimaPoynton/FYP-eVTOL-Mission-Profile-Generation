function [ca_d, gam_d] = createDemandVecVF(mission, p, kc, kg, ca_inf, g_inf)
% function createDemandVecVF determines the demand course angle and flight
% path to keep on a straight line path between waypoints idx = [i j] from
% position p, by applying vector fields
%
% kc: 
% kg:

%% Identify Nearest and Next Waypoint
[~, i] = min(sqrt(sum()))
j = i+1;

%% Caclulate Demands
wpi = mission.xyz(:,i); % current waypoint
wpj = mission.xyz(:,j); % next waypoint

ep = (wpj - wpi)/norm(wpj - wpi); % line direction
pl = (eye(3)-ep*ep')*(p - wpi); % path orthogonal error

ca_l = atan2([0 1 0]*wpj, [1 0 0]*wpi);  % course angle of line path

RIs = [ cos(ca_l), sin(ca_l), 0;
       -sin(ca_l), cos(ca_l), 0;
        0       , 0       , 1]; % rotation matrix from inertial to straight line path reference framer

ca_d = ca_l - ca_inf*(2/pi)*atan(kc*[0 1 0]'*RIs*pl); % course angle demand to stay on path

gam_l = atan(-ep*[0 0 1]'/sqrt((ep*[1 0 0])^2 + (ep*[0 1 0])^2));

gam_d = gam_l + g_inf*(2/pi)*atan(kg*[0 0 1]'*RIs*pl);