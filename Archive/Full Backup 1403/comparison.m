clear

VX4 = configDef;

wing1 = wing(0, 1, 100, 1, 0.8181818, [2.2 0 0], [0 0 0], 2*pi, 7.5, 0.12, 2,"0012");
% aileron = controlSurf(0.3, "TE", 2*pi, 1, 0.12);
% aileron.deflection = pi*(1/180);
% wing1 = addCS(wing1, aileron, 0.5);

VX4 = addLift(VX4, wing1); 

wing2 = wing(0, -1, 100, 1, 0.8181818, [2.2 0 0], [0 0 0], 2*pi, 7.5, 0.12, 2,"0012");
% aileron = controlSurf(0.3, "TE", 2*pi, 1, 0.12);
% aileron.deflection = pi*(1/180);
% wing1 = addCS(wing1, aileron, 0.5);

VX4 = addLift(VX4, wing2); 

tail1 = wing(0, -1, 100, 1, 0.8181818, [5.1 0 0], [0 0 0], 2*pi, 2.5, 0.12, 1,"0012");
% ruddervator = controlSurf(0.2, "TE", 2*pi, 0.5, 0.12);
% tail1 = addCS(tail1, ruddervator, 0.1);

VX4 = addLift(VX4, tail1); 

tail2 = wing(0, 1, 100, 1, 0.8181818, [5.1 0 0], [0 0 0], 2*pi, 2.5, 0.12, 1,"0012");
% tail2 = addCS(tail2, ruddervator, 0.1);

VX4 = addLift(VX4, tail2); 

plotConfiguration(VX4)

angles = pi*[1:3:90]/180;

airflow = struct();
airflow.U = 1;
airflow.alpha = 1*pi/180;
airflow.beta = 0;

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
lift3 = [wing1 wing2 tail1 tail2];
Sref = 30;
cref = 2;
bref = 15;

referenceGeo = struct();
referenceGeo.Sref = Sref;
referenceGeo.cref = cref;
referenceGeo.bref = bref;

N_obj = [4 4; 4 4; 3 3; 3 3]; % number of panels per component in lift3

[lift3, mesh, N] = meshAll3(lift3, N_obj, 1,1);

% [lift3, coeff, distribution3, AC3] = VLMV3(lift3, mesh, airflow, N, 1, [0 0 0], Sref, cref, bref, N_obj);

alpha = [0 1 3 4]*pi/180;
beta = [0 5 10 15]*pi/180;

stabilityDeriv = getStabilityDeriv(lift3, mesh, airflow.U, alpha, beta, N,  referenceGeo, 0, N_obj);

%% Run Simulink Model
ICs = struct();
ICs.uvw = [0 0 0];
ICs.lmn = [0 0 0];
ICs.pqr = [0 0 0];





