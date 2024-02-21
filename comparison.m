clear

VX4 = configDef;

wing1 = wing(0, 1, 100, 1, 0.8181818, [1 0 0], [0 0 0], 2*pi, 7.5, 0.12, 2,"0012");
% aileron = controlSurf(0.3, "TE", 2*pi, 1, 0.12);
% aileron.deflection = pi*(1/180);
% wing1 = addCS(wing1, aileron, 0.5);

VX4 = addLift(VX4, wing1); 

wing2 = wing(0, -1, 100, 1, 0.8181818, [1 0 0], [0 0 0], 2*pi, 7.5, 0.12, 2,"0012");
% aileron = controlSurf(0.3, "TE", 2*pi, 1, 0.12);
% aileron.deflection = pi*(1/180);
% wing1 = addCS(wing1, aileron, 0.5);

VX4 = addLift(VX4, wing2); 

tail1 = wing(0, -1, 100, 0.4, 0.9, [2 0 0], [-pi/4 0 0], 2*pi, 0.6, 0.12, 0.5,"0012");
% ruddervator = controlSurf(0.2, "TE", 2*pi, 0.5, 0.12);
% tail1 = addCS(tail1, ruddervator, 0.1);

VX4 = addLift(VX4, tail1); 

tail2 = wing(0, 1, 100, 0.4, 0.9, [2 0 0], [pi/4 0 0], 2*pi, 0.6, 0.12, 0.5,"0012");
% tail2 = addCS(tail2, ruddervator, 0.1);

VX4 = addLift(VX4, tail2); 

plotConfiguration(VX4)

angles = pi*[1:3:90]/180;

airflow = struct();
airflow.U = 1;
airflow.alpha = 1*pi/180;
airflow.beta = 0.1;

lift = [wing1 wing2];

lift = meshAll(lift, [8 8; 8 8], 1,0);
[lift, CL, CD, CM, distribution, AC] = VLM(lift, airflow, [8 8; 8 8], 1, 0);