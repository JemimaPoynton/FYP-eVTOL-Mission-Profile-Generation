function X = generateMissionStates(xyz, cruiseVel, Ns)

% convert xyz to velocities
uvw = [];
X = [uvw pqr ang];

figure()
plot3(time, xyz(1), xyz(2), xyz(3))