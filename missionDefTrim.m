clear courseAngle gamma U forces aero uvw_e alpha
aircraft = VX4;

%% Define a Mission Profile
mission = struct();
N = 1000;
Np = 30;

initTO = [zeros(1, N); zeros(1, N); linspace(0, 20, N)];
transitionTO = [linspace(0, 30, N); zeros(1, N); 20 + 5.*(1 - exp((-(linspace(0, 10, N)/6).^3)))];

cruise = [linspace(30, 120, N); linspace(0, 30, N); 25 + zeros(1, N)]; 

transitionL = [linspace(120, 220, N); 30 + zeros(1, N); 25 - 3.*(1 - exp((-(linspace(0, 10, N)/5).^3)))];
endL = [220 + zeros(1, N); 30 + zeros(1, N); linspace(22, 0, N)];

mission.xyz = [initTO transitionTO cruise transitionL endL];
mission.N = N; % points on mission sections
mission.secTime = [10 30 60 30 400];
mission.st = 5; % number of stages
% mission.rotorTilt = [];
mission.rho = 1.225;
mission.modes = ['v' 't' 'c' 't' 'v'];

[~, ~, dist, ~, idxf, ~] = getTrajStates(mission, Np, 1);

plot3(mission.xyz(1,:), mission.xyz(2,:), mission.xyz(3,:))
grid on
axis equal

mission.cruiseVel = 20;

idx = round(linspace(1, N-1, Np),0);
% idxf = [idx idx+mission.N idx+mission.N*2 idx+mission.N*3 idx+mission.N*4];

%% Define Rotor Deflection Through Profile
mission.rdef = zeros(size(dist(idx)));

mission.rdef([1 5],:) = 0;
mission.rdef(2,:) = (pi/2)*(1 - exp((-(dist(idx)/10).^3)));
mission.rdef(3,:) = pi/2;
mission.rdef(4,:) = (pi/2)*(exp((-(dist(idx)/10).^3)));

figure()
grid on
plot(dist(idxf), [mission.rdef(1,:) mission.rdef(2,:) mission.rdef(3,:) mission.rdef(4,:) mission.rdef(5,:)]*(180/pi))
xlabel('Distance along path [m]')
ylabel('Rotor Tilt [deg]')

%% Define Velocity Profile
mission.vel = zeros(size(dist(idx)));

mission.vel(1,:) = linspace(10, 0.001, length(idx));
mission.vel(2,:) = linspace(0.001, 25, length(idx));
mission.vel(3,:) = 25;
mission.vel(4,:) = linspace(25, 0.001, length(idx));
mission.vel(5,:) = linspace(0.001, 10, length(idx));

figure()
plot(dist(idxf), [mission.vel(1,:) mission.vel(2,:) mission.vel(3,:) mission.vel(4,:) mission.vel(5,:)])

%% Define Angle of Attack Limits (Stall)
mission.alphaLim = zeros(size(dist(idx))); % Assuming that stall effects are negiligible in hover

mission.alphaLim([1 5],:) = pi/2;
mission.alphaLim(2:4,:) = 14.5*(pi/180);

%% Calculate Mission Trim
trim = optimiseTrimMission(aircraft, coefficients, mission, Np, 5);

%% Plot
createTrimPlots(1, 1, 1, 1, 1, 1, trim)

%% Save Trim Data
save('trimUAM1','trim')