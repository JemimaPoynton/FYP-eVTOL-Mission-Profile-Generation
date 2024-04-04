function [gamma, courseAngle, dist, totalDist, idxf, idx] = getTrajStates(mission, Np, plotBool)
% function getTrajStates samples points on mission, and converts them to
% course angle and trajectory states

%% Setup Samping
N = mission.N; % number of analysis points for each stage of profile
idx = round(linspace(1, N-1, Np));

gamma = zeros(length(mission.secTime), Np);
courseAngle = gamma;

%% Calculate Course Angle and Trajectory
for i = 1:length(mission.secTime)
    for j = 1:Np
        step = mission.N*(i-1) + 1;

        dx = mission.xyz(1, step + idx(j)) - mission.xyz(1, step + idx(j) - 1);
        dy = mission.xyz(2, step + idx(j)) - mission.xyz(2, step + idx(j) - 1);
        dz = mission.xyz(3, step + idx(j)) - mission.xyz(3, step + idx(j) - 1);
    
        gamma(i,j) = atan(dz/dx);
        courseAngle(i,j) = atan(dy/dx);
    end
end

gamma(find(isnan(gamma))) = 0; % nan returned for angle = 0 (tan)
courseAngle(find(isnan(courseAngle))) = 0;

idxf = [idx idx+mission.N idx+mission.N*2 idx+mission.N*3 idx+mission.N*4];

%% Create Vector of Distance (Not Displacment)
steps = diff(mission.xyz,1,2); % steps in xyz for each plot point
vecSteps = sqrt(steps(1,:).^2 + steps(2,:).^2 + steps(3,:).^2); % vector magnitude (distance) between points
dist = [0 cumsum(vecSteps)];
totalDist = sum(vecSteps);

%% Plot Mission Course Variables
if plotBool == 1
    figure()
    plot3(mission.xyz(1,:), mission.xyz(2,:), mission.xyz(3,:), 'black')
    hold on
    grid on
    scatter3(mission.xyz(1,idxf), mission.xyz(2,idxf), mission.xyz(3,idxf), 'x', 'red')
    
    zlabel('Altitude [m]')
    xlabel('Longitudinal Disp. [m]')
    ylabel('Lateral Dis. [m]')
    
    figure()
    plot(dist(idxf), [courseAngle(1,:) courseAngle(2,:) courseAngle(3,:) courseAngle(4,:) courseAngle(5,:)].*(pi/180), 'black--')
    hold on
    grid on
    plot(dist(idxf), [gamma(1,:) gamma(2,:) gamma(3,:) gamma(4,:) gamma(5,:)].*(180/pi), 'black-')
    
    legend('Course Angle', 'Flight Path Angle')
    xlabel('Total Distance [m]')
    ylabel('Angle [deg]')
end