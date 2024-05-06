function [wing, nCx, nCy, ny] = createMesh3D_CS(wing, N, plotmesh, iteration)
% function createMesh creates a mesh for calculating aerodynamic properties
% via the vortex lattice method.
%
% wing: wing to be meshed.
% N: number of nodes in [x y]
mesh = struct();

%% Preallocation
X = zeros(N(1), N(2)); Xc = zeros(N(1)-1, N(2)-1); 
z1 = zeros(N(1)-1, N(2)-1); z2 = z1; mesh.surface_x = z1; mesh.surface_y = z1; mesh.surface_z = z1; dP = z1;
unitVec = zeros(N(1)-1, N(2)-1, 3);

%% Shape Reference Points to Control Surface (Y, span)
if ~isempty(wing.CSs)
    [Ydiv, nCy] = meshAroundCS(wing, N(2), wing.CSs(1).span, wing.span, 2);
else
    Ydiv = linspace(0, wing.span, N(2));
    nCy = [];
    nCx = [];
end

%% Mesh points
if wing.sideY == 1
    Y = Ydiv;
elseif wing.sideY == -1
    Y = -flip(Ydiv,2);
end

mesh.y = zeros(N(1),N(2));
mesh.x = zeros(N(1),N(2));

%% Continue
ang = wing.ang.*[1 -1 1]; % flipping y to adhere to geometric coordinates
Lt = transMatrix(ang);

xit = 0;

if ~isempty(wing.CSs)
    [~, ~, nx] = meshAroundCS(wing, N(1), wing.CSs(1).chordRatio*getChord(wing, max(abs(Y))), getChord(wing, max(abs(Y))), 1, xit);
end

for j = 1:N(2) % !translate angles into body frame!
    try
    offset = Y(j)*tan(wing.sweep)*wing.sideY; % x offset from the nose due to sweep
    catch
        disp('here')
    end
    
    for i = 1:N(1)
        % in wing frame
        wingc = getChord(wing, Y(j));
        if ~isempty(wing.CSs)
            [Xdiv, nCx, nx] = meshAroundCS(wing, N(1), wing.CSs(1).chordRatio*wingc, wingc, 1, 1, nx);
        else
            Xdiv = linspace(0, wingc, N(1));
        end

        X(:,j) = (offset - getChord(wing, Y(j)) + Xdiv)';
        xyz = [X(i,j) Y(j) 0]*Lt + wing.pos;

        mesh.x(i,j) = xyz(1); 
        mesh.y(i,j) = xyz(2);
        mesh.z(i,j) = xyz(3);
    end
end

%% Generate Control points
% nb: the end points of Yc and Xc are out of bounds, but kept for
%     readibility and are not carried forward.

Yc = Y(1:end-1) + 0.5*diff(Y);

[~,~,~, shape] = getNACA(wing.aerofoil,0);
mid = (length(shape)-1)/2 + 1;

for j = 1:N(2)-1
    offset = Yc(j)*tan(wing.sweep)*wing.sideY; % x offset from the nose due to sweep
    
    for i = 1:N(1)-1
        % rotate to body frame

        Xc(:,j) = ((X(1:end-1,j) + X(1:end-1,j+1))/2 + (3/4)*diff(X(:,j)))';
        xyz_c = [Xc(i,j) Yc(j) 0]*Lt + wing.pos;

        mesh.control.xc(i,j) = xyz_c(1); 
        mesh.control.yc(i,j) = xyz_c(2);
        mesh.control.zc(i,j) = xyz_c(3);

        chordLength = (diff(X(:,j+1)) + diff(X(:,j)))/2;
        dx = 0.005*chordLength(i);
        
        % get x positions of boundaries as % chord  
        wingc = (getChord(wing, Y(j)) + getChord(wing, Y(j+1)))/2;

        LE = [offset - wingc Yc(j) 0]*Lt + wing.pos;
        xc_temp = xyz_c/Lt;
        xc_LE = xc_temp(1) - LE(1); % distance of control point from LE 

        x1 = xc_LE + dx/2;
        x2 = xc_LE - dx/2;
        
        c = wingc;

        % get thickness as % chord by interpolating data
        z1(i,j) = (interp1(shape(1:mid,1),shape(1:mid,2), x1/c,'makima') ...
                   + interp1(shape(mid:end,1),shape(mid:end,2),x1/c,'makima'))/2; % find z of camber line as %chord

        z1(i,j) = z1(i,j)*wingc; % scale up

        z2(i,j) = (interp1(shape(1:mid,1),shape(1:mid,2),x2/c,'makima') + ...
                   interp1(shape(mid:end,1),shape(mid:end,2),x2/c,'makima'))/2;

        z2(i,j) = z2(i,j)*wingc; % scale up

%         zsurf(i,j) = interp1(shape(mid:end,1),shape(mid:end,2), x2/c)*c;

        surface = LE + [xc_LE 0 (z1(i,j)+z2(i,j))/2]*Lt;
        mesh.surface_x(i,j) = surface(1);
        mesh.surface_y(i,j) = surface(2);
        mesh.surface_z(i,j) = surface(3);

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

        unitVec(i,j,:) = (A*unitVec_b')'*Lt;
    end
end

mesh.unitVec = unitVec;

%% Generate Quarter Chord Line (Bound Vortex)

for j = 1:N(2)
    for i = 1:N(1)-1
        chordLength = diff(X(:,j)); 
        xyz = [chordLength(i).*0.25 0 0]*Lt; % translation in wing plane converted to body frame
        mesh.nodes.xq(i,j) = mesh.x(i,j) + xyz(1);
        mesh.nodes.zq(i,j) = mesh.z(i,j) + xyz(3);
    end
end

mesh.nodes.yq = mesh.y; % no change in y, but included for consistency

%% Seperate Control Surface Panels
% due to the discontinuous nature of a control surface panel, it must be
% stored as a seperate surface. Similarly either side must be stored
% seperately

if ~isempty(wing.CSs)
    [meshs1, meshs2, meshs3] = seperateCS(mesh, nCx, nCy, wing, N);
end

%% Plot
if plotmesh == 1
    scatter3([],[],[], 'HandleVisibility','off'); % initiate plot
    hold on
    axis equal
    grid on
    
    if ~isempty(wing.CSs)
        plotMeshSurface(wing, meshs1, iteration)
        iteration = 0;
        plotMeshSurface(wing, meshs2, iteration)
        plotMeshSurface(wing, meshs3, iteration)
    else
        plotMeshSurface(wing, mesh, iteration)
    end

end

%% Set far downstream points (to estimate vectors going to -> inf)
mesh.nodes.xq = [mesh.nodes.xq; (mesh.nodes.xq(N(1)-1,:) - mesh.nodes.xq(N(1)-2,:))*2000];
mesh.nodes.zq = [mesh.nodes.zq; (mesh.nodes.zq(N(1)-1,:) - mesh.nodes.zq(N(1)-2,:))*2000];

%% Seperate Control Surface Panels
% due to the discontinuous nature of a control surface panel, it must be
% stored as a seperate surface. Similarly either side must be stored
% seperately

if ~isempty(wing.CSs)
    [meshs1, meshs2, meshs3] = seperateCS(mesh, nCx, nCy, wing, N);
end
nCx = N(1) - nCx;

%% Set mesh
if ~isempty(wing.CSs)
    meshs1.ny = nCy;
    meshs2.ny = 1:nCy-1;
    meshs3.ny = nCy(end)+1:N(2)-1;
    if wing.sideY == 1
        wing.mesh = [meshs2; meshs1; meshs3];
    else
        wing.mesh = [meshs3; meshs1; meshs2];
    end
else
    wing.mesh = mesh;
    wing.mesh.ny = 1:N(2)-1;
end

%% (Plot aerofoil)
% figure()
% plot(shape(:,1), shape(:,2))
% hold on
% % plot([x1(:,1) x2(:,1)],[z1(:,:) z2(:,:)])
% grid on
