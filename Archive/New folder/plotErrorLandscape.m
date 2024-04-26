% plot elevator deflection against rotor thrust and then error
clear J dE rT u alpha

Va = 0.5; % analyse at a midpoint
traj = [0 0]; % analyse steady level flight
rpitch = 0;

nT = length(aircraft.thrust.rotors) + length(aircraft.thrust.ducts);
thrustIn = thrustobj2struct(aircraft, zeros(1,nT));

%% Vary control inputs
dE = linspace(-0.5, 0.5, 30);
rT = linspace(0, 3000, 30);

for i = 1:length(dE)
    for j = 1:length(rT)
        Z = [15 5 0   0 0 0   0 dE(i) 0 rT(j) rT(j) rT(j) rT(j)]';

        J(i,j) = trimCost(Z, Va, coeff, rho, VX4.refGeo, VX4.m, thrustIn, VX4.CG, VX4.I, traj, rpitch);
    end
end

figure()
surf(dE, rT, J)
xlabel('Elevator Deflection [rad]')
ylabel('Rotor Deflection [rad]')
zlabel('Error J')

%% Pitch-Elevator Deflection
pitch = linspace(0, pi/2, 30);
dE = linspace(-0.5, 0.5, 30);

for i = 1:length(dE)
    for j = 1:length(pitch)
        Z = [15 5 0   0 pitch(i) 0   dE(i) 0 0 300 300 300 300]';

        J(i,j) = trimCost(Z, Va, coeff, rho, VX4.refGeo, VX4.m, thrustIn, VX4.CG, VX4.I, traj, rpitch);
    end
end

figure()
surf(dE, pitch, J)
xlabel('Elevator Deflection [rad]')
ylabel('Pitch (theta) [rad]')
zlabel('Error J')

%% Angle of Attack
clear J
Va = 16;
alpha = linspace(-0.5, 0.7, 40);
u = linspace(0.1, 30, 40);
% change = linspace(-1, 1, 5);
scatter3(0, 0, 0)
zlim([0 1e5])
   
for i = 1:length(u)
    for j = 1:length(alpha)
        Z = [u(i) 0 tan(alpha(j))*u(i)   0 alpha(j)+traj(1) 0   0 0 0 300 300 300 300]';

        J(i,j) = trimCost(Z, Va, coeff, rho, VX4.refGeo, VX4.m, thrustIn, VX4.CG, VX4.I, traj, pi/4);
    end
end

surf(u, alpha, J)
xlabel('Elevator Deflection [rad]')
ylabel('Vertical Velocity [m/s]')
zlabel('Error J')

% ylim([-14.5*(pi/180) 14.5*(pi/180)])
% zlim([0 5050])

% Mark true min point
[Amins, idx] = min(J);
[Amin, Aj] = min(Amins);
Ai = idx(Aj);

hold on
scatter3(u(Aj), alpha(Ai), J(Ai, Aj))

