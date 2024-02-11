function plotConfiguration(aircraft)
% Function plotConfiguration displays a simplified representation of the
% positions and mounting angles of each structural component defining a
% configuration. fig is the output figure.
% NB: the geometric coordinate system is flipped in x when compared with
% flight dynamics, i.e. x is +ve aft of the nose. z is upwards
%
% Jemima Poynton 06/02/24

%% Initialise plot
scatter3([],[],[], 'HandleVisibility','off');
hold on
    % !Need to modify this such that it appends to the fig object!

%% Plot Thrust Components
thrust = struct2cell(aircraft.thrust); % readibility
handlevis = 'on';

for i = 1:length(thrust)
    comp = thrust(i); % set of components of one type (class)

    for j = 1:length(comp{1, 1})
        obj = comp{1, 1}(1, j); % current object

        ang = comp{1, 1}(1, j).ang.*[1 -1 1]; % flipping y to adhere to geometric coordinates
        Lt = transMatrix(ang);

        xyz_vec = [0 0 1]*Lt;

        colour = "magenta";
        marker = 'o';

        if j>1
            handlevis = 'off'; % Handling legend
        else
            handlevis = 'on';
        end

        if class(obj) == "rotor"
            colour = "blue";
            marker = 'x';

            r = obj.radius;
            x = linspace(- r, r, 1000);
            y1 = sqrt(r^2 - (x).^2);
            y2 = -sqrt(r^2 - (x).^2);

            % Transform from rotor plane to body coordinates
            xyz1 = [x' y1' zeros(size(x))']*Lt;
            xyz2 = [x' y2' zeros(size(x))']*Lt;
            plot3([xyz1(:,1) xyz2(:,1)] + obj.pos(1), [xyz1(:,2) xyz2(:,2)] + obj.pos(2), [xyz1(:,3) xyz2(:,3)] + obj.pos(3), colour, 'HandleVisibility','off')
        end

        scatter3(obj.pos(1), obj.pos(2), obj.pos(3), marker, colour, 'HandleVisibility', handlevis)
        quiver3(obj.pos(1), obj.pos(2), obj.pos(3), xyz_vec(1), xyz_vec(2), xyz_vec(3), colour, 'HandleVisibility','off')

    end
end

%% Plot Fuselage
% Just plot a straight line of a certain colour, and mark CG and CL

fuselage = aircraft.fuselage;
nF = length(fuselage);

for i = 1:nF
    r = fuselage(i).diameter/2;
    y = linspace(fuselage(i).pos(2) - r, fuselage(i).pos(2) + r, 1000);
    z1 = sqrt(r^2 - (y - fuselage(i).pos(2)).^2) + fuselage(i).pos(3);
    z2 = -sqrt(r^2 - (y - fuselage(i).pos(2)).^2) + fuselage(i).pos(3);

    plot3(ones(size([y y]))*0 + fuselage(i).pos(1), [y y], [z1 z2], 'red-','HandleVisibility','on')
    plot3(ones(size([y y]))*(fuselage.leng - fuselage.laft) + fuselage(i).pos(1), [y y], [z1 z2], 'red-', 'HandleVisibility','off')

    plot3([0 (fuselage.leng - fuselage.laft) + fuselage(i).pos(1)], (fuselage(i).pos(2) - r)*[1 1], fuselage(i).pos(3)*[1 1], 'red-', 'HandleVisibility','off')
    plot3([0 (fuselage.leng - fuselage.laft) + fuselage(i).pos(1)], (fuselage(i).pos(2) + r)*[1 1], fuselage(i).pos(3)*[1 1], 'red-', 'HandleVisibility','off')

    plot3([0 (fuselage.leng - fuselage.laft) + fuselage(i).pos(1)], fuselage(i).pos(2)*[1 1], (fuselage(i).pos(3) - r)*[1 1], 'red-', 'HandleVisibility','off')
    plot3([0 (fuselage.leng - fuselage.laft) + fuselage(i).pos(1)], fuselage(i).pos(2)*[1 1], (fuselage(i).pos(3) + r)*[1 1],'red-', 'HandleVisibility','off')

    % Plot conical aft (tail section)
    x = linspace((fuselage.leng - fuselage.laft), fuselage.leng, 1000);
    plot3(x, (fuselage(i).pos(2) - r)*ones(size(x)) + fuselage.laft - (r/(fuselage.laft)).*x, fuselage(i).pos(3)*ones(size(x)), 'red-', 'HandleVisibility','off')
    plot3(x, (fuselage(i).pos(2) + r)*ones(size(x)) - fuselage.laft + (r/(fuselage.laft)).*x, fuselage(i).pos(3)*ones(size(x)), 'red-', 'HandleVisibility','off')
    plot3(x, zeros(size(x)), (fuselage(i).pos(3) + r)*ones(size(y)) - fuselage.laft + (r/(fuselage.laft)).*x, 'red-', 'HandleVisibility','off')
    plot3(x, zeros(size(x)), (fuselage(i).pos(3) - r)*ones(size(y)) + fuselage.laft - (r/(fuselage.laft)).*x, 'red-', 'HandleVisibility','off')
end

%% Plot Lift Surfaces
% Currently assume straight edges of wing - simplified representation

lift = aircraft.lift;
nL = length(lift);

for i = 1:nL
    ang = lift(i).ang.*[1 -1 1];
    xyz1 = lift(i).pos;

    if i>1
        handlevis = 'off'; % Handling legend
    else
        handlevis = 'on';
    end

    Lt = transMatrix(ang);
    xyz2 = xyz1 + [lift(i).span*tan(lift(i).sweep) lift(i).sideY*lift(i).span 0]*Lt; % transform a translation of span in the wing plane
    xyz3 = xyz1 + [-getChord(lift(i), lift(i).span) + lift(i).span*tan(lift(i).sweep) lift(i).sideY*lift(i).span 0]*Lt;
    xyz4 = xyz1 + [-getChord(lift(i), 0) 0 0]*Lt;

    plot3([xyz1(1) xyz2(1) xyz3(1) xyz4(1)], [xyz1(2) xyz2(2) xyz3(2) xyz4(2)], [xyz1(3) xyz2(3) xyz3(3) xyz4(3)], 'black-', 'HandleVisibility', handlevis)
    
    CSs_vec = lift(i).CSs;
    for j = 1:length(CSs_vec)
        ang = lift(i).ang.*[1 -1 1];
        xyz12 = CSs_vec(j).pos;

        if j>1
            handlevis = 'off'; % Handling legend
        end
        
        CT = getChord(CSs_vec(j), CSs_vec(j).span, lift(i));
        CR = getChord(CSs_vec(j), 0, lift(i));

        Lt = transMatrix(ang);

        if CSs_vec(j).edge == "LE"
            taperAng = atan((getChord(lift(i),0) - getChord(lift(i),lift(i).span))/lift(i).span);

            xyz22 = xyz12 + [-CR + CT + CSs_vec(j).span*tan(taperAng) + (CSs_vec(j).span)*tan(lift(i).sweep), ...
                    lift(i).sideY*CSs_vec(j).span, ...
                    0]*Lt;

            xyz32 = xyz12 + [-CR + CSs_vec(j).span*tan(taperAng) + (CSs_vec(j).span)*tan(lift(i).sweep), ...
                    lift(i).sideY*CSs_vec(j).span, ...
                    0]*Lt;
            
            xyz42 = xyz12 + [-CR 0 0]*Lt;
        else
            xyz22 = xyz12 + [CSs_vec(j).span*tan(lift(i).sweep) lift(i).sideY*CSs_vec(j).span 0]*Lt; % transform a translation of span in the wing plane
            xyz32 = xyz12 + [-CT + CSs_vec(j).span*tan(lift(i).sweep) lift(i).sideY*CSs_vec(j).span 0]*Lt;
            xyz42 = xyz12 + [-CR 0 0]*Lt;
        end
        
        plot3([xyz12(1) xyz22(1) xyz32(1) xyz42(1) xyz12(1)], [xyz12(2) xyz22(2) xyz32(2) xyz42(2) xyz12(2)], [xyz12(3) xyz22(3) xyz32(3) xyz42(3) xyz12(3)], 'green-', 'HandleVisibility', handlevis)
    end
end


%% General
axis equal
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

scatter3(aircraft.CoL(:,1), aircraft.CoL(:,2), aircraft.CoL(:,3), '<', 'green')
scatter3(aircraft.CG(:,1), aircraft.CG(:,2), aircraft.CG(:,3), 'filled','^', 'green')

if isempty(aircraft.thrust.ducts)
    legend('Rotors', 'fuselage', 'Lifting Surfaces', 'Control Surface', 'Centre of Lift', 'Centre of Gravity')
elseif isempty(aircraft.thrust.rotors)
    legend('Ducts', 'fuselage', 'Lifting Surfaces', 'Control Surface', 'Centre of Lift', 'Centre of Gravity')
else
    legend('Rotors', 'Ducts', 'fuselage', 'Lifting Surfaces', 'Control Surface', 'Centre of Lift', 'Centre of Gravity')
end

