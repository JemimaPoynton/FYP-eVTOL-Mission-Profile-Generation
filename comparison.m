clear

VX4 = configDef;
VX4.CG = [1.71 0 0];
VX4.CoL = [2.04 0 0];

wing1 = wing(0.2, 1, 100, 1, 0.3, [3.1 0 0], [0 0 0], 2*pi, 3.5, 0.12, 1.625,"2412");
aileron = controlSurf(0.3, "TE", 2*pi, 1, 0.12);
aileron.deflection = 0*(pi/180); aileron.roll = 1;
wing1 = addCS(wing1, aileron, 2.2);

VX4 = addLift(VX4, wing1); 

wing2 = wing(0.2, -1, 100, 1, 0.3, [3.1 0 0], [-0 0 0], 2*pi, 3.5, 0.12, 1.625,"2412");
aileron = controlSurf(0.3, "TE", 2*pi, 1, 0.12);
aileron.deflection = 0*(pi/180); aileron.roll = 1;
wing2 = addCS(wing2, aileron, 2.2);

VX4 = addLift(VX4, wing2); 

tail1 = wing(0, -1, 100, 1, 0.9, [0.44 0 0.1], [-0 0 0], 2*pi, 1.5, 0.12, 0.5,"0012");
ruddervator = controlSurf(0.4, "TE", 2*pi, 0.8, 0.12);
ruddervator.deflection = 0*(pi/180);
tail1 = addCS(tail1, ruddervator, 0.5);

VX4 = addLift(VX4, tail1); 

tail2 = wing(0, 1, 100, 1, 0.9, [0.44 0 0.1], [0 0 0], 2*pi, 1.5, 0.12, 0.5,"0012");
tail2 = addCS(tail2, ruddervator, 0.5);

VX4 = addLift(VX4, tail2); 

rotor1 = rotor(0.4, 1, 1, 1, 2, [4 1.75 0.2], [0 0 0], 3000);
VX4 = addThrust(VX4, rotor1);

rotor2 = rotor(0.4, 1, 1, 1, 2, [4 -1.75 0.2], [0 0 0], 3000);
VX4 = addThrust(VX4, rotor2);

rotor3 = rotor(0.4, 1, 1, 1, 2, [0.25 -2 0.2], [0 0 0], 3000);
VX4 = addThrust(VX4, rotor3);

rotor4 = rotor(0.4, 1, 1, 1, 2, [0.25 2 0.2], [0 0 0], 3000);
VX4 = addThrust(VX4, rotor4);

rudder = controlSurf(0.4, "TE", 2*pi, 0.4, 0.12);
tail3 = wing(0, 1, 100, 1, 0.9, [3.6 1.75 0], [-pi/2 + 0.05 0 0], 2*pi, 0.9, 0.12, 0.4,"0012");
tail3 = addCS(tail3, rudder, 0.14);
VX4 = addLift(VX4, tail3); 

tail4 = wing(0, 1, 100, 1, 0.9, [3.6 -1.75 0], [-pi/2 - 0.05 0 0], 2*pi, 0.9, 0.12, 0.4,"0012");
tail4 = addCS(tail4, rudder, 0.14);
VX4 = addLift(VX4, tail4); 

plotConfiguration(VX4)

angles = pi*[1:3:90]/180;

airflow = struct();
airflow.U = 1;
airflow.alpha = 1*pi/180;
airflow.beta = 0;
airflow.p = 0;
airflow.q = 0;
airflow.r = 0;

%%
lift = [wing1 wing2];

lift = meshAll(lift, [3 3], 1,1);
[mesh, N] = combineWings(lift(1), lift(2)); %combined mesh

Sref = lift(1).S + lift(2).S;
Cref = (2*getChord(lift(1),0) + getChord(lift(1),lift(1).span) + getChord(lift(2),lift(2).span))/4; % mean aerodynamic chord

[lift, CL, CD, CM, CY, distribution, AC] = VLM(lift, mesh, airflow, N, 1, 0, Sref, Cref);

lift2 = [tail1 tail2];

lift2 = meshAll(lift2, [2 2], 1,1);
[mesh, N] = combineWings(lift2(1), lift2(2)); %combined mesh

% Sref = lift2(1).S + lift2(2).S;

[lift2, CL2, CD2, CM2, distribution2, AC2] = VLM(lift2, mesh, airflow, N, 1, 0, Sref, Cref);

%%
lift3 = [wing1 wing2 tail1 tail2 tail3 tail4];
Sref = 11.88;
cref = 1.836;
bref = 7;

referenceGeo = struct();
referenceGeo.Sref = Sref;
referenceGeo.cref = cref;
referenceGeo.bref = bref;

N_obj = [13 10; 13 10; 7 7; 7 7; 5 5; 5 5;]; % number of panels per component in lift3

% [lift3, coeff, distribution3, AC3] = VLMV3(lift3, mesh, airflow, N, 1, [0 0 0], Sref, cref, bref, N_obj);

alpha = ([0.1 0.21 0.3])*pi/180;
beta = [0 0.1 0.2]*pi/180;
pqr = [0.009 0.01 0.011]*pi/180;

stabilityDeriv = getStabilityDeriv(lift3, airflow, alpha, beta, referenceGeo, 0, N_obj, [0 1 2 3; 0 1 2 3; 0 1 2 3;]*pi/180, pqr, VX4.CG, VX4.CG);

%% Run Simulink Model
ICs = struct();
ICs.uvw = [0 0 0];
ICs.lmn = [0 0 0];
ICs.pqr = [0 0 0];

%% Covergence test
clear CL Cm CD
ny = 4:2:60;
nx = 5.*ones(size(ny));

for i = 1:length(ny)
    N_obj = [nx(i) ny(i); nx(i) ny(i); nx(i) ny(i); nx(i) ny(i);];

    [liftout, mesh, N] = meshAll3(lift3, N_obj, 1, 0);
    [~, coeffB, ~, ~] = VLMV3(liftout, mesh, airflow, N, 0, [0 0 0], referenceGeo.Sref, referenceGeo.cref, referenceGeo.bref, N_obj, VX4.CG);

    CL(i) = coeffB.CL
    CD(i) = coeffB.CD;
    Cm(i) = coeffB.Cm;
end


%% Plot
figure()
plot(ny, CL, 'black-')
hold on
plot(ny, CD, 'black--')
plot(ny, Cm, 'black:')
grid on
xlabel('Spanwise Elements')

figure()
plot(ny, CL - CL(end))
hold on
grid on
plot(ny, CD - CD(end))
plot(ny, Cm - Cm(end))
xlabel('Spanwise Elements')
ylabel('Error')



