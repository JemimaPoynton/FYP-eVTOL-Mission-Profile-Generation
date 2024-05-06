mission = struct();

mission.simTime = 140;
mission.cruise.t = linspace(10, mission.simTime, 1000);
mission.trans.t = linspace(0, 10, 1000);

%% Define transition
mission.trans.maxAlt = 30;
mission.trans.altitude = -mission.trans.maxAlt*(1 - exp((-(mission.trans.t/5).^3))); % positive altitude, which will be converted to negative dynamic z coordinate
mission.cruise.altitude = -mission.trans.maxAlt-80*(1 - exp((-((mission.cruise.t-10)/5).^3)));

plot(mission.trans.t, mission.trans.altitude)

mission.fwdvel = 26; % find required forward velocity

mission.cruise.y = 10*sin(t);
plot(mission.cruise.t, mission.cruise.y)

%% 3D plot
figure()
plot3(mission.trans.t.*mission.fwdvel, zeros(size(mission.trans.t)), -mission.trans.altitude, '--')
hold on
plot3(mission.cruise.t.*mission.fwdvel, mission.cruise.y, -mission.cruise.altitude,'--')
grid on