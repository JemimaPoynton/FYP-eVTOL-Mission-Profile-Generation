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
nT = length(thrust);
thrust = aircraft.thrust;

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

%% Plot Lift Surfaces
% Currently assume straight edges of wing - simplified representation
% Define 4 corner points

lift = aircraft.lift;
nL = length(lift);

for i = 1:nL
    % plotTE
    xyz1 = lift(i).pos;
    xyz2(1) = lift(i).pos + getChord(lift(i), 0);
end


%% Plot control surfaces
% Define plane
% Draw CS outline on plane

