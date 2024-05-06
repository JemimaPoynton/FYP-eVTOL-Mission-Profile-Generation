clear

VX4 = configDef;

wing1 = wing(0, 1, 100, 1, 0.8181818, [2.2 0 0], [0 0 0], 2*pi, 7.5, 0.12, 2,"0012");
aileron = controlSurf(0.3, "TE", 2*pi, 3, 0.12);
aileron.deflection = pi*(1/180);
wing1 = addCS(wing1, aileron, 3.5);

VX4 = addLift(VX4, wing1); 

wing2 = wing(0, -1, 100, 1, 0.8181818, [2.2 0 0], [0 0 0], 2*pi, 7.5, 0.12, 2,"0012");
aileron = controlSurf(0.3, "TE", 2*pi, 3, 0.12);
aileron.deflection = pi*(1/180);
wing2 = addCS(wing2, aileron, 3.5);

VX4 = addLift(VX4, wing2); 

tail1 = wing(0, -1, 100, 1, 0.8181818, [5.1 0 0], [-pi/6 0 0], 2*pi, 2.5, 0.12, 1,"0012");
ruddervator = controlSurf(0.2, "TE", 2*pi, 0.5, 0.12);
% tail1 = addCS(tail1, ruddervator, 0.1);

VX4 = addLift(VX4, tail1); 

tail2 = wing(0, 1, 100, 1, 0.8181818, [5.1 0 0], [pi/6 0 0], 2*pi, 2.5, 0.12, 1,"0012");
% tail2 = addCS(tail2, ruddervator, 0.1);

VX4 = addLift(VX4, tail2); 

fuselage = fuselage(1, 1, 1, 6.1, 1, 0, 1, 2);
fuselage.pos = [-1 0 0];
VX4 = addFuselage(VX4, fuselage); 

angles = pi*[1:3:90]/180;

airflow = struct();
airflow.U = 1;
airflow.alpha = 1*pi/180;
airflow.beta = 0;

VX4.I = [107,    0,  0.6971;
         0,      93, 0     ;
         0.6971, 0,  196   ];

VX4.m = 150;
VX4.Cd0 = 0.018;

rotor1 = rotor(0.7, 1, 1, 1, 2, [0 3 0.3], [0 pi/6 0], 3000);
VX4 = addThrust(VX4, rotor1);

rotor2 = rotor(0.7, 1, 1, 1, 2, [0 -3 0.3], [0 pi/6 0], 3000);
VX4 = addThrust(VX4, rotor2);

rotor3 = rotor(0.7, 1, 1, 1, 2, [2.2 3 -0.3], [0 pi/6 0], 3000);
VX4 = addThrust(VX4, rotor3);

rotor4 = rotor(0.7, 1, 1, 1, 2, [2.2 -3 -0.3], [0 pi/6 0], 3000);
VX4 = addThrust(VX4, rotor4);

VX4.CG = [1.1 0 0];
VX4 = setReferenceGeometry(VX4, 30, 15, 2);

Sref = 30;
cref = 2;
bref = 15;

referenceGeo = struct();
referenceGeo.Sref = Sref;
referenceGeo.cref = cref;
referenceGeo.bref = bref;

plotConfiguration(VX4)