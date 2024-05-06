clear courseAngle gamma U forces aero uvw_e alpha
aircraft = VX4;

%% Define a Mission Profile
mission = struct();
N = 1000;
Np = 30;

initTO = [zeros(1, N); zeros(1, N); linspace(0, 20, N)];
transitionTO = [linspace(0, 30, N); zeros(1, N); 20 + 5.*(1 - exp((-(linspace(0, 10, N)/6).^3)))];

cruise = [linspace(30, 120, N); linspace(0, 30, N); 25 + zeros(1, N)]; 

transitionL = [linspace(120, 240, N); 30 + zeros(1, N); 25 - 3.*(1 - exp((-(linspace(0, 10, N)/3).^3)))];
endL = [240 + zeros(1, N); 30 + zeros(1, N); linspace(22, 0, N)];

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
mission.rdef(4,:) = (pi/2)*(exp((-(dist(idx)/8).^3)));

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
mission.alphaLim(2:4,:) = 10.5*(pi/180);

%% Calculate Mission Trim
trim = optimiseTrimMission(aircraft, coefficients, mission, Np, 5);

%% Plot
createTrimPlots(1, 1, 1, 1, 1, 1, trim, aircraft.thrust.rotors(1).kt)

%% Save Trim Data
save('trimUAM1','trim')

%% figure()
grid on
plot(dist(idxf), [mission.rdef(1,:) mission.rdef(2,:) mission.rdef(3,:) mission.rdef(4,:) mission.rdef(5,:)]*(180/pi))
% !Add drawn bounds!
hold on; grid on;
plot(dist(idxf), [squeeze(trim.U(6,:,1)) squeeze(trim.U(6,:,2)) squeeze(trim.U(6,:,3)) squeeze(trim.U(6,:,4)) squeeze(trim.U(6,:,5))]*(180/pi))
xlabel('Distance along path [m]')
ylabel('Rotor Tilt [deg]')

%% Post process power
clear vh lamh T lami_ lami Cpp Cp CT

U = reshape(trim.U,size(trim.U,1),[]);
X = reshape(trim.X,size(trim.X,1),[]);
alpha = reshape(trim.alpha,size(trim.alpha,1),[]);
nT = 4;

nC = 3;

for i = 1:nT
    rotor = aircraft.thrust.rotors(i);
    T(:,i) = U(nC + 4*(i - 1) + 1,:).^2*rotor.kt;
    T(find(T(:,i)==0),:) = 1e-15;
    VT = 448*rotor.radius*ones(size(-X(1,:)));
    CT = T(:,i)./(1.225*pi*(rotor.radius^2).*(VT)'.^2);

    vh(:,i) = sqrt(T(:,i)/(2*1.225*pi*(rotor.radius)^2))';
    lamh(:,i) = vh(:,i)./VT';

    idxhv = find(abs(X(1,:)) < 1e-3); % index of states axial climb

    idxaf1 = find(-X(3,idxhv)./vh(idxhv,i)' >= 0); % index of ascent
    idxaf3 = find((-X(3,idxhv)./vh(idxhv,i)' <= -2)); % index of fast descent

    idxaf2 = zeros(size(-X(1,:))); % index of slow descent
    idxaf2(idxhv) = ones(size(-X(1,idxhv)));
    idxaf2([idxaf1; idxaf3]) = 0; % find all other indices
    idxaf2 = find(idxaf2 == 1);

    idxff = ones(size(-X(1,:))); % remove axial flight cases
    idxff(idxhv) = 0; 
    idxff = find(idxff == 1);

    % Apply calculations for lami
    lami = zeros(size(X(1,:)));

    vc2vh = -X(3,idxaf1)./(2*vh(idxaf1,i)');
    lami(idxaf1) = lamh(idxaf1,i)'.*(-vc2vh + sqrt(vc2vh.^2 + 1));

    vc2vh = -X(3,idxaf3)./(2*vh(idxaf3,i)');
    lami(idxaf3) = lamh(idxaf3,i)'.*(-vc2vh - sqrt(vc2vh.^2 - 1));

    vcvh = -X(3,idxaf2)./(vh(idxaf2,i)');
    lami(idxaf2) = 1.15 -1.125*vcvh - 1.372*vcvh.^2 - 1.718*vcvh.^3 - 0.655*vcvh.^4;

    vinf = sqrt(X(3,idxff).^2 + X(2,idxff).^2 + X(1,idxff).^2);
    muxx_ = (vinf.*cos(alpha(idxff))./VT(idxff))'./lamh(idxff,i);
    muxz_ = (vinf.*sin(alpha(idxff))./VT(idxff))'./lamh(idxff,i);

    for n = 1:length(muxz_)
        syms x
        eqn = muxx_(n)^2 + (muxz_(n) + x)^2 == 1/x^2;
        lami_temp = double(solve(eqn,x, 'Real', true, 'ReturnConditions',false));
        try
        lami_(n) = lami_temp(find(lami_temp >= 0,1,'last'));
        catch
            disp('here')
        end
    end

    lami(idxff) = lamh(idxff,i).*lami_';
    Va = sqrt(X(3,:).^2 + X(2,:).^2 + X(1,:).^2);

    Cpp(:,i) = zeros(size(lami));
    Cpp(idxff,i) = (0.115*0.012/8)*(1 + 4.6*(vinf./VT(idxff)).^2');

    Cpi(:,i) = 1.15*CT.*lami';
    Cpnet(:,i) = CT.*Va/VT;
    Cp(:,i) = 1.15*CT.*lami' + Cpp(:,i) + Cpnet(:,i);
end

plot(dist(idxf), sum(Cp,2).*1.225.*VT'.^3.*pi.*(rotor.radius^2)/(1e3))
grid on
hold on
plot(dist(idxf), sum(Cpi,2).*1.225.*VT'.^3.*pi.*(rotor.radius^2)/(1e3))
plot(dist(idxf), sum(Cpnet,2).*1.225.*VT'.^3.*pi.*(rotor.radius^2)/(1e3))
ylabel('Power [kW]')
xlabel('Distance [m]')

figure(); grid on
plot(dist(idxf), sum(Cpp,2).*1.225.*VT'.^3.*pi.*(rotor.radius^2)/(1e3))
%!add individual rotor power, could also plot Cp, rather than power directly to be independent from rpm!
ylabel('Power [kW]')
xlabel('Distance [m]')