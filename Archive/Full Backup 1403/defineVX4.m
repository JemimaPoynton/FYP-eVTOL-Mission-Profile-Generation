clear

VX4 = configDef;

wing1 = wing(0.1, 1, 100, 1, 0.6, [1 0 0], [0.1 -0.1 0], 2*pi, 1.9, 0.12, 0.5,"4412");
aileron = controlSurf(0.3, "TE", 2*pi, 1, 0.12);
aileron.deflection = pi*(1/180);
wing1 = addCS(wing1, aileron, 0.5);

VX4 = addLift(VX4, wing1); 

wing2 = wing(0.1, -1, 100, 1, 0.6, [1 0 0], [-0.1 -0.1 0], 2*pi, 2, 0.12, 0.5, "2414");
wing2 = addCS(wing2, aileron, 0.5);

VX4 = addLift(VX4, wing2); 

tail1 = wing(0, -1, 100, 0.4, 0.9, [2 0 0], [-pi/4 0 0], 2*pi, 0.6, 0.12, 0.5,"4412");
ruddervator = controlSurf(0.2, "TE", 2*pi, 0.5, 0.12);
tail1 = addCS(tail1, ruddervator, 0.1);

VX4 = addLift(VX4, tail1); 

tail2 = wing(0, 1, 100, 0.4, 0.9, [2 0 0], [pi/4 0 0], 2*pi, 0.6, 0.12, 0.5,"4412");
tail2 = addCS(tail2, ruddervator, 0.1);

VX4 = addLift(VX4, tail2); 

rotor1 = rotor(0.2, 1, 1, 1, 1, [0.3 0.5 0.1], [0 pi/6 0]);
VX4 = addThrust(VX4, rotor1);

rotor2 = rotor(0.2, 1, 1, 1, 1, [0.3 -0.5 0.1], [0 pi/6 0]);
VX4 = addThrust(VX4, rotor2);

rotor3 = rotor(0.2, 1, 1, 1, 1, [0.3 1.5 0.1], [0 pi/6 0]);
VX4 = addThrust(VX4, rotor3);

rotor4 = rotor(0.2, 1, 1, 1, 1, [0.3 -1.5 0.1], [0 pi/6 0]);
VX4 = addThrust(VX4, rotor4);

rotor5 = rotor(0.2, 1, 1, 1, 1, [1.2 0.5 0.1], [0 0 0]);
VX4 = addThrust(VX4, rotor5);

rotor6 = rotor(0.2, 1, 1, 1, 1, [1.2 -0.5 0.1], [0 0 0]);
VX4 = addThrust(VX4, rotor6);

rotor7 = rotor(0.2, 1, 1, 1, 1, [1.2 1.5 0.1], [0 0 0]);
VX4 = addThrust(VX4, rotor7);

rotor8 = rotor(0.2, 1, 1, 1, 1, [1.2 -1.5 0.1], [0 0 0]);
VX4 = addThrust(VX4, rotor8);

fuselage = fuselage(1, 1, 1, 2, 0.3, 0, 1, 0.6);
VX4 = addFuselage(VX4, fuselage); 

VX4.CG = [0.4 0 0];
VX4.CoL = [0.8 0 0];

plotConfiguration(VX4)

angles = pi*[1:3:90]/180;

airflow = struct();
airflow.U = 30;
airflow.alpha = 0.1;
airflow.beta = 0.1;

for i = 1:length(angles)
    for j = 1:4
        VX4.thrust.rotors(j) = VX4.thrust.rotors(j).setAngle([0 angles(i) 0]);
        plotConfiguration(VX4)
    end
    VX4.lift(1, 1).CSs = deflect(VX4.lift(1, 1).CSs, angles(i)/4);
    drawnow
    exportgraphics(gcf,'testAnimated.gif', 'Resolution',600,'Append',true);
end
