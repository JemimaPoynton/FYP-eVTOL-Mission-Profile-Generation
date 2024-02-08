function fig = plotConfiguration(aircraft)
% Function plotConfiguration displays a simplified representation of the
% positions and mounting angles of each structural component defining a
% configuration. fig is the output figure.
%
% Jemima Poynton 06/02/24

%% Initialise plot
fig = scatter3([],[],[]);
hold on

%% Plot Thrust Components
thrust = aircraft.thrust;
nT = length(thrust);

% !Need to modify this such that it appends to the fig object!
for i = 1:nT
    scatter3(thrust(i).x, thrust(i).y, thrust(i).z, 'x')
    ang = thrust(i).ang;
    x_vec = 1;
    y_vec = x_vec*tan(-ang(1));
    z_vec = x_vec*tan(ang(2));

    quiver3(thrust(i).x, thrust(i).y, thrust(i).z, x_vec, y_vec, z_vec)
end

%% Plot Fuselage
% Just plot a straight line of a certain colour, and mark CG and CL
fuselage = aircraft.fuselage;
nF = length(fuselage);

for i = 1:nF

end

%% Plot Lift Surfaces
% Currently assume straight edges of wing - simplified representation

lift = aircraft.lift;
nL = length(lift);

for i = 1:nL
    ang = lift(i).ang;
    xyz1 = lift(i).pos; % !ADD SWEEP!

    Lt = transMatrix(ang);
    xyz2 = [0 lift(i).sideY*lift(i).span 0]*Lt; % transform a translation of span in the wing plane
    xyz3 = [getChord(lift(i), lift(i).span) lift(i).sideY*lift(i).span 0]*Lt;
    xyz4 = [getChord(lift(i), 0) 0 0]*Lt;

    plot3([xyz1(1) xyz2(1) xyz3(1) xyz4(1)], [xyz1(2) xyz2(2) xyz3(2) xyz4(2)], [xyz1(3) xyz2(3) xyz3(3) xyz4(3)])
end

%% Plot control surfaces
% Define plane
% Draw CS outline on plane

