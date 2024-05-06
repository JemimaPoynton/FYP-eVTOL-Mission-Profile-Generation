mission = struct();
N = 1000;

initTO = [zeros(1, N); zeros(1, N); linspace(0, 20, N)];
transitionTO = [linspace(0, 10, N); zeros(1, N); 20 + 30.*(1 - exp((-(linspace(0, 10, N)/5).^3)))];

cruise = [linspace(10, 90, N); linspace(0, 30, N); 50 + zeros(1, N)]; 

transitionL = [linspace(90, 100, N); 30 + zeros(1, N); 50 - 30.*(1 - exp((-(linspace(0, 10, N)/5).^3)))];
endL = [100 + zeros(1, N); 30 + zeros(1, N); linspace(20, 0, N)];

mission.xyz = [initTO transitionTO cruise transitionL endL];

plot3(mission.xyz(1,:), mission.xyz(2,:), mission.xyz(3,:))
grid on
axis equal

mission.cruiseVel = 20;