clear

VX4 = configDef;
VX4.CG = [0.5 0 0];

wing1 = wing(0, 1, 100, 1, 0.818181, [2.2 0 0], [0 0 0], 2*pi, 7.5, 0.12, 2,"0012");
aileron = controlSurf(0.3, "TE", 2*pi, 5, 0.12);
aileron.deflection = -0*(pi/180);
wing1 = addCS(wing1, aileron, 0.9);

VX4 = addLift(VX4, wing1); 

wing2 = wing(0, -1, 100, 1, 0.818181, [2.2 0 0], [0 0 0], 2*pi, 7.5, 0.12, 2,"0012");
aileron = controlSurf(0.3, "TE", 2*pi, 5, 0.12);
aileron.deflection = -0*(pi/180);
wing2 = addCS(wing2, aileron, 0.9);

VX4 = addLift(VX4, wing2); 

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
lift3 = [wing1 wing2];
Sref = 30;
cref = 2;
bref = 15;

referenceGeo = struct();
referenceGeo.Sref = Sref;
referenceGeo.cref = cref;
referenceGeo.bref = bref;

N_obj = [14 14; 14 14; 13 13; 13 13]; % number of panels per component in lift3

% [lift3, coeff, distribution3, AC3] = VLMV3(lift3, mesh, airflow, N, 1, [0 0 0], Sref, cref, bref, N_obj);

alpha = [0.8 1 1.2 1.5]*pi/180;
beta = [0 0.3 0.6 1]*pi/180;
pqr = [0.009 0.01 0.011]*pi/180;

stabilityDeriv = getStabilityDeriv(lift3, airflow, alpha, beta, referenceGeo, 0, N_obj, [0 1 2 3]*pi/180, pqr, [0.5 0 0], VX4.CG);