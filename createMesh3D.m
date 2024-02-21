function wing = createMesh3D(wing, N, plotmesh, iteration)
% function createMesh creates a mesh for calculating aerodynamic properties
% via the vortex lattice method.
%
% wing: wing to be meshed.
% N: number of nodes in [x y]

%% Preallocation
X = zeros(N(1), N(2)); Xc = zeros(N(1), N(2)-1); 
z1 = zeros(N(1)-1, N(2)-1); z2 = z1; surface_x = z1; surface_y = z1; surface_z = z1; dP = z1;
unitVec = zeros(N(1)-1, N(2)-1, 3);

%% Mesh points
mesh = struct();

if wing.sideY == 1
    Y = linspace(0, wing.span, N(2));
elseif wing.sideY == -1
    Y = wing.sideY*wing.span + linspace(0, wing.span, N(2));
end


mesh.y = zeros(N(1),N(2));
mesh.x = zeros(N(1),N(2));

ang = wing.ang.*[1 -1 1]; % flipping y to adhere to geometric coordinates
Lt = transMatrix(ang);

for j = 1:N(2) % !translate angles into body frame!
    offset = Y(j)*tan(wing.sweep)*wing.sideY; % x offset from the nose due to sweep
    
    for i = 1:N(1)
        % in wing frame
        X(:,j) = (offset - getChord(wing, Y(j)) + linspace(0, getChord(wing, Y(j)), N(1)))';
        xyz = [X(i,j) Y(j) 0]*Lt + wing.pos;

        mesh.x(i,j) = xyz(1); 
        mesh.y(i,j) = xyz(2);
        mesh.z(i,j) = xyz(3);
    end
end

%% Generate Control points
% nb: the end points of Yc and Xc are out of bounds, but kept for
%     readibility and are not carried forward.

if wing.sideY == 1
    Yc = linspace(0, wing.span, N(2)) + 0.5*(wing.span/(N(2)-1));
elseif wing.sideY == -1
    Yc = wing.sideY*wing.span + linspace(0, wing.span, N(2)) + 0.5*(wing.span/(N(2)-1));
end

[~,~,~, shape] = getNACA(wing.aerofoil,0);
mid = (length(shape)-1)/2 + 1;

for j = 1:N(2)-1
    offset = Yc(j)*tan(wing.sweep)*wing.sideY; % x offset from the nose due to sweep
    
    for i = 1:N(1)-1
        % rotate to body frame
        Xc(:,j) = (offset - getChord(wing, Yc(j)) + linspace(0, getChord(wing, Yc(j)), N(1)) + (getChord(wing, Yc(j))/(N(1)-1))*(3/4))';
        xyz_c = [Xc(i,j) Yc(j) 0]*Lt + wing.pos;

        mesh.control.xc(i,j) = xyz_c(1); 
        mesh.control.yc(i,j) = xyz_c(2);
        mesh.control.zc(i,j) = xyz_c(3);

        dx = 0.005*getChord(wing, Yc(j))/(N(1)-1);
        
        % get x positions of boundaries as % chord
        xc_LE = (getChord(wing, Yc(j))/(N(1)-1))*(i - 1/4); % distance of control point from LE 

        LE = [offset - getChord(wing, Yc(j)) Yc(j) 0]*Lt + wing.pos;

        x1 = xc_LE + dx/2;
        x2 = xc_LE - dx/2;
        
        c = getChord(wing, Yc(j));

        % get thickness as % chord by interpolating data
        z1(i,j) = (interp1(shape(1:mid,1),shape(1:mid,2), x1/c,'makima') ...
                   + interp1(shape(mid:end,1),shape(mid:end,2),x1/c,'makima'))/2; % find z of camber line as %chord

        z1(i,j) = z1(i,j)*getChord(wing, Yc(j)); % scale up

        z2(i,j) = (interp1(shape(1:mid,1),shape(1:mid,2),x2/c,'makima') + ...
                   interp1(shape(mid:end,1),shape(mid:end,2),x2/c,'makima'))/2;

        z2(i,j) = z2(i,j)*getChord(wing, Yc(j)); % scale up

%         zsurf(i,j) = interp1(shape(mid:end,1),shape(mid:end,2), x2/c)*c;

        surface = LE + [x2 0 z1(i,j)]*Lt;
        surface_x(i,j) = surface(1);
        surface_y(i,j) = surface(2);
        surface_z(i,j) = surface(3);

        dz = z1(i,j) - z2(i,j);
        dP(i,j) = atan(dz/dx); % slope of mean camber
        
        % NB: r1 and r2 are in the body frame of reference
        r2 = [Xc(i,j) + wing.pos(1), Yc(j), 0] - [X(i,j) + wing.pos(1), Y(1,j), 0];
        r1 = [Xc(i,j) + wing.pos(1), Yc(j), 0] - [X(i,j+1) + wing.pos(1), Y(1,j+1), 0];

        % calculate unit vector in body frame of reference
        unitVec_b = cross(r1,r2)/norm(cross(r1,r2));
        
        % convert unit vector to aerofoil frame of reference
        A = [cos(dP(i,j)) 0 -sin(dP(i,j));...
             0                1  0;...
             sin(dP(i,j)) 0  cos(dP(i,j))]; % conversion to aerofoil frame

        unitVec(i,j,:) = (A*unitVec_b')';
    end
end

mesh.unitVec = unitVec;

%% Generate Quarter Chord Line (Bound Vortex)

for j = 1:N(2)
    for i = 1:N(1)-1
        xyz = [getChord(wing, mesh.y(i,j)*wing.sideY)/(N(1)-1)*0.25 0 0]*Lt; % translation in wing plane converted to body frame
        mesh.nodes.xq(i,j) = mesh.x(i,j) + xyz(1);
        mesh.nodes.zq(i,j) = mesh.z(i,j) + xyz(3);
    end
end

mesh.nodes.yq = mesh.y; % no change in y, but included for consistency

%% Plot
if plotmesh == 1
   if iteration == 1 % handle legend visibility
       handlevis = 'on';
   else
       handlevis = 'off';
   end
   
   scatter3([],[],[], 'HandleVisibility','off'); % initiate plot
   hold on
   axis equal
   grid on

   plotLift(wing,0)
   scatter3(mesh.x, mesh.y, mesh.z, 'black', '.', 'HandleVisibility','off')

   % plotting some data twice for easy handling of legend
   plot3([mesh.x(1,1)'; mesh.x(1,end)'], [mesh.y(1,1)'; mesh.y(1,end)'], [mesh.z(1,1)'; mesh.z(1,end)'],  'black-', 'HandleVisibility',handlevis)
   plot3([mesh.x(2:end,1)'; mesh.x(2:end,end)'], [mesh.y(2:end,1)'; mesh.y(2:end,end)'], [mesh.z(2:end,1)'; mesh.z(2:end,end)'],  'black-', 'HandleVisibility',"off")
   plot3([mesh.x(1,:); mesh.x(end,:)], [mesh.y(1,:); mesh.y(end,:)], [mesh.z(1,:); mesh.z(end,:)], 'black-', 'HandleVisibility','off')

   quiver3(mesh.control.xc, mesh.control.yc, mesh.control.zc, unitVec(:,:,1), unitVec(:,:,2), unitVec(:,:,3),0.5, 'red-', 'HandleVisibility',handlevis)
   
   scatter3(mesh.control.xc(1,:), mesh.control.yc(1,:), mesh.control.zc(1,:), 10, 'o', 'red', 'HandleVisibility',handlevis)
   scatter3(mesh.control.xc(2:end,:), mesh.control.yc(2:end,:), mesh.control.zc(2:end,:), 10, 'o', 'red', 'HandleVisibility',"off")

   plot3(mesh.nodes.xq(1,:)', mesh.y(1, :)', mesh.nodes.zq(1,:)', 'blue--', 'HandleVisibility',handlevis)
   plot3(mesh.nodes.xq(2:end,:)', mesh.y(2:end-1, :)', mesh.nodes.zq(2:end,:)', 'blue--', 'HandleVisibility',"off")

   xlabel('x [m]')
   ylabel('y [m]')
   zlabel('y [m]')

   plot3(surface_x, surface_y, surface_z, 'green-')

   legend("", "Lattice", "Perpendicular Unit Vector (Aerofoil)", "Control Points", "1/4 Chord Line", "Aerofoil Camber")
end

%% Set far downstream points (to estimate vectors going to -> inf)
mesh.nodes.xq = [mesh.nodes.xq; (mesh.nodes.xq(N(1)-1,:) - mesh.nodes.xq(N(1)-2,:))*10000];
mesh.nodes.zq = [mesh.nodes.zq; (mesh.nodes.zq(N(1)-1,:) - mesh.nodes.zq(N(1)-2,:))*10000];

%% Set mesh
wing.mesh = mesh;

%% (Plot aerofoil)
% figure()
% plot(shape(:,1), shape(:,2))
% hold on
% % plot([x1(:,1) x2(:,1)],[z1(:,:) z2(:,:)])
% grid on
