function [trim, gamma, courseAngle, totalDist] = optimiseTrimMission(aircraft, coeff, mission, Np, stg, ubR)

%% Get Trajectory
[gamma, courseAngle, dist, totalDist, idxf, ~] = getTrajStates(mission, Np, 1);

%% Solve Optimisation
prev = [];
for i = 1:stg
    for j = 1:Np
        traj = [gamma(i,j) courseAngle(i,j)];
        [U(:,j,i), X(:,j,i), forces(:,j,i), aero(:,j,i), uvw_e(:,j,i), alpha(:,j,i), MTcg(:,j,i), checkfail(:,j,i)] = trimSolver(aircraft, coeff, mission.rho, mission.vel(i,j), traj, mission.rdef(i,j), mission.alphaLim(i,j), prev, ubR(i,j));
        prev = U(:,j,i);
    end
end

trim = struct(); % store data in structure for easy input to function
trim.dist = dist; trim.idxf = idxf; trim.Np = Np; trim.stg = stg;
trim.U = U; trim.X = X; trim.forces = forces; trim.aero = aero; trim.uvw_e = uvw_e; trim.alpha = alpha;
trim.modes = mission.modes; % load in types

%% Create control mixed (VTOL) trim
Fx = forces(1,:,:); Fy = forces(2,:,:); Fz = forces(3,:,:); 
Tx = MTcg(1,:,:); Ty = MTcg(2,:,:); Tz = MTcg(3,:,:);

trim.Ut = [Fx;
           Fy;
           Fz;
           Tx;
           Ty;
           Tz];

%% Check for Failed Trim
if sum(sum(checkfail)) > 0 % handle failed trim
    [idx1, idx2] = find(squeeze(checkfail) == 1);
    disp(" ")
    fprintf(2, ['Trim failed at index ' mat2str([ones(size(idx1)), idx1, idx2]) ' and distance ' num2str(dist(idxf((idx2-1)*Np + idx1))) '\n'])
end