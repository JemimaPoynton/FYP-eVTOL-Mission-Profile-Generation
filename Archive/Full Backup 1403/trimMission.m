function trimMission(mission, aircraft, coeff, Ts)
% function trimMission calculates and plots the required control inputs to 
% trim aircraft at points on mission sampled at Ts.

% extract X from mission data
X = generateMissionStates(mission.xyz, cruiseVel, Ts)

for i = 1:size(X,2)
    [U(:,i), forces(:,i), aero(:,i)] = trimSolver(aircraft, coeff, rho, X(i));
end

% plot some stuff

P = rotorPower(U(rotorIdx)); % calculate rotor power from BEMT