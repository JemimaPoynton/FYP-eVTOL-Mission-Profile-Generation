clear U forces aero uvw_e alpha X
uvec = linspace(1, 30, 40);
rho = 1.225;
aircraft = VX4;

%% Define Rotor Deflection
rpitch = (pi/2)*(1 - exp((-(uvec/18).^3)));
% plot(uvec, rpitch)

%%
for i = 1:length(uvec)
    u = uvec(i);
    traj = [0 0];
    [U(:,i), X(:,i), forces(:,i), aero(:,i), uvw_e(:,i), alpha(:,i)] = trimSolver(aircraft, coeff, rho, u, traj, rpitch(i), 14.5*(pi/180));
end

%% Plot Control Inputs
% !NEED TO GENERALISE THIS PLOT!
figure()
grid on
hold on
plot(uvec, U(1:3,:))
legend('Elevator (deg)', 'Aileron (deg)', 'Rudder (deg)')

xlabel('Airspeed [m/s]')
ylabel('Deflection [deg]')

figure()
subplot(2,1,1)
grid on
hold on
plot(uvec, U([4 8 12 16],:))
legend('Rotor 1 (deg)', 'Rotor 2 (deg)', 'Rotor 3 (deg)', 'Rotor 4 (deg)')

ylabel('Thrust [N]')

subplot(2,1,2)
grid on
hold on
plot(uvec, U([6 10 14 18],:))
legend('Rotor 1 (deg)', 'Rotor 2 (deg)', 'Rotor 3 (deg)', 'Rotor 4 (deg)')

xlabel('Airspeed [m/s]')
ylabel('Deflection [deg]')

%% Plot Forces (body)
figure()
grid on
hold on
plot(uvec, forces(1,:), 'black-')
plot(uvec, forces(2,:), 'black--')
plot(uvec, forces(3,:), 'black-.')

plot(uvec, forces(7,:), 'blue-')
plot(uvec, forces(8,:), 'blue--')
plot(uvec, forces(9,:), 'blue-.')

% plot(uvec, forces(4,:), 'red-')
% plot(uvec, forces(5,:), 'red--')
% plot(uvec, forces(6,:), 'red-.')

xlabel('Airspeed [m/s]')
ylabel('Force in Body Frame [N]')
legend('Thrust (x)', 'Thrust (y)', 'Thrust (z)', 'Aerodynamic Load (x)', 'Aerodynamic Load (y)', 'Aerodynamic Load (z)')

%% Plot Force (earth)
figure()
grid on
hold on
plot(uvec, forces(10,:), 'black-')
plot(uvec, forces(11,:), 'black--')
plot(uvec, forces(12,:), 'black-.')

legend('Total (x)', 'Total (y)', 'Total (z)')
xlabel('Airspeed [m/s]')
ylabel('Force in Earth Frame [N]')

%% Plot Aerodynamics
figure()
grid on
hold on
plot(uvec, aero(1,:), 'black-')
plot(uvec, aero(2,:), 'black--')
plot(uvec, aero(3,:), 'black-.')

legend('Lift Coefficient', 'Drag Coefficient', 'Pitching Moment Coefficient')
xlabel('Airspeed [m/s]')

%% Plot Velocities in Earth Frame
figure()
grid on
hold on
plot(uvec, uvw_e(1,:), 'black-')
plot(uvec, uvw_e(2,:), 'black--')
plot(uvec, uvw_e(3,:), 'black-.')

legend('u_e (x)', 'v_e (y)', 'w_e (z)')
xlabel('Airspeed [m/s]')
ylabel('Velocity in Earth Frame [m/s]')

%% Plot Angle of Attack
figure()
grid on
hold on
plot(uvec, alpha)
xlabel('Airspeed [m/s]')
ylabel('Angle of Attack [rad]')

%% Plot States
figure()
plot(uvec, X(1:3,:))
grid on
hold on
% scatter(dist(idxf), zeros(size(dist(idxf))), 'x', 'red')

xlabel('Distance [m]')
ylabel('uvw [N]')
legend('u (x)', 'v (y)', 'w (z)')

figure()
plot(uvec, X(7:9,:))
grid on
hold on
% scatter(dist(idxf), zeros(size(dist(idxf))), 'x', 'red')

xlabel('Distance [m]')
ylabel('angle [rad]')
legend('phi', 'theta', 'psi')