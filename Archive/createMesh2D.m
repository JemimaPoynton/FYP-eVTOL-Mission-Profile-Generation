function wing = createMesh2D(wing, N, plotmesh)
% function createMesh creates a mesh for calculating aerodynamic properties
% via the vortex lattice method.
%
% wing: wing to be meshed.
% N: number of nodes in [x y]

mesh = struct();
mesh.nodes.y = linspace(0,wing.span,N(2));
mesh.nodes.x = zeros(N(2),N(1));

for i = 1:N(2)
    offset = mesh.nodes.y(i)*tan(wing.sweep); % x offset from the nose due to sweep
    mesh.nodes.x(i,:) = wing.pos(1) + offset - linspace(0, getChord(wing, mesh.nodes.y(i)), N(1)); % !need to adjust y along sweep/taper line!
end

wing.mesh = mesh;

% Generate Control points

if plotmesh == 1
   scatter3([],[],[], 'HandleVisibility','off');
   hold on
   axis equal
   grid on

   plotLift(wing,0)
   scatter3(mesh.nodes.x, mesh.nodes.y, zeros(N(2),N(1)), 'black', '.')
   plot([mesh.nodes.x(:,1)'; mesh.nodes.x(:,end)'], [mesh.nodes.y; mesh.nodes.y], 'black-')
   plot([mesh.nodes.x; mesh.nodes.x], [mesh.nodes.y(1,:)'; mesh.nodes.y(end,:)'], 'black-')
   % plot lined grid
end