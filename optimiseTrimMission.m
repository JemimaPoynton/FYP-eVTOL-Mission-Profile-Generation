function [trim, gamma, courseAngle, totalDist] = optimiseTrimMission(aircraft, coeff, mission, Np, stg)

%% Get Trajectory
[gamma, courseAngle, dist, totalDist, idxf, ~] = getTrajStates(mission, Np, 1);

%% Solve Optimisation
for i = 1:stg
    for j = 1:Np
        traj = [gamma(i,j) courseAngle(i,j)];
        [U(:,j,i), X(:,j,i), forces(:,j,i), aero(:,j,i), uvw_e(:,j,i), alpha(:,j,i), checkfail(:,j,i)] = trimSolver(aircraft, coeff, mission.rho, mission.vel(i,j), traj, mission.rdef(i,j), mission.alphaLim(i,j));
    end
end

trim = struct(); % store data in structure for easy input to function
trim.dist = dist; trim.idxf = idxf; trim.Np = Np; trim.stg = stg;
trim.U = U; trim.X = X; trim.forces = forces; trim.aero = aero; trim.uvw_e = uvw_e; trim.alpha = alpha;
trim.modes = mission.modes; % load in types

if sum(sum(checkfail)) > 0 % handle failed trim
    [idx1, idx2] = find(squeeze(checkfail) == 1);
    disp(" ")
    fprintf(2, ['Trim failed at index ' mat2str([1, idx1, idx2]) ' and distance ' num2str(dist(idxf((idx2-1)*Np + idx1))) '\n'])
end