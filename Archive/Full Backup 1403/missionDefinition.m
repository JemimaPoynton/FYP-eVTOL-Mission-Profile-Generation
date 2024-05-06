mission = struct();

%% Define VTOL
mission.VTOL.time = 20;
% define xyz mission profile


%% Define Transition
mission.transitionTO.time = 10;
t = linspace(0, mission.transitionTO.time, 1000);
mission.transition.altitude = 1 - exp((-(t/20).^3));
mission.transition.rotorTilt = linspace(0, pi/2, 1000);

plot(t, mission.transition.rotorTilt)

%% Define Cruise
mission.cruise.time = [];

%% Check lift slope
alpha = linspace(14.5*(pi/180), pi/2, 1000);
CL = -768.6*alpha.^3 + 608.2*-155.2*alpha.^2 - 155.2*alpha + 15.2;

plot(alpha, CL)
