clear U forces aero
uvec = linspace(1, 25, 70);
rho = 1.225;
aircraft = VX4;

for i = 1:length(uvec)
    X = [uvec(i) 0 0 0 0 0 0 0 0]';
    [U(:,i), forces(:,i), aero(:,i)] = trimSolver(aircraft, coeff, rho, X);
end

%% Plot Control Inputs
% !NEED TO GENERALISE THIS PLOT!
figure()
grid on
hold on
plot(uvec, U(1:3,:))
legend('Elevator (deg)', 'Aileron (deg)', 'Rudder (deg)')

figure()
subplot(2,1,1)
grid on
hold on
plot(uvec, U([4 8 12 16],:))
legend('Thrust 1 (N)', 'Thrust 2 (N)', 'Thrust 3 (N)', 'Thrust 4 (N)')

subplot(2,1,2)
grid on
hold on
plot(uvec, U([6 10 14 18],:))
legend('Deflection 1 (deg)', 'Deflection 2 (deg)', 'Deflection 3 (deg)', 'Deflection 4 (deg)')

%% Plot Forces
% !SET SOLVER TO GET FORCES OUT AND PLOT!
figure()
grid on
hold on
plot(uvec, forces(1,:), 'black-')
plot(uvec, forces(2,:), 'black--')
plot(uvec, forces(3,:), 'black-.')
plot(uvec, forces(7,:), 'blue-')
plot(uvec, forces(8,:), 'blue--')
plot(uvec, forces(9,:), 'blue-.')

legend('Thrust (x) [N]', 'Thrust (y) [N]', 'Thrust (z) [N]', 'Aerodynamic Load (x) [N]', 'Aerodynamic Load (y) [N]', 'Aerodynamic Load (z) [N]')

%% Plot Aerodynamics
figure()
grid on
hold on
plot(uvec, aero(1,:), 'black-')
plot(uvec, aero(2,:), 'black--')
plot(uvec, aero(3,:), 'black-.')

legend('Lift Coefficient', 'Drag Coefficient', 'Pitching Moment Coefficient')