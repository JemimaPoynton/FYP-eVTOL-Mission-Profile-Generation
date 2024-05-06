function plotMeshSurface(wing, mesh, iteration)

   if iteration == 1 % handle legend visibility
       handlevis = 'on';
   else
       handlevis = 'off';
   end

   % plotLift(wing,0)
   scatter3(mesh.x, mesh.y, mesh.z, 'black', '.', 'HandleVisibility','off')

   % plotting some data twice for easy handling of legend
   plot3([mesh.x(1,1)'; mesh.x(1,end)'], [mesh.y(1,1)'; mesh.y(1,end)'], [mesh.z(1,1)'; mesh.z(1,end)'],  'black-', 'HandleVisibility',handlevis)
   plot3([mesh.x(2:end,1)'; mesh.x(2:end,end)'], [mesh.y(2:end,1)'; mesh.y(2:end,end)'], [mesh.z(2:end,1)'; mesh.z(2:end,end)'],  'black-', 'HandleVisibility',"off")

   for i = 1:size(mesh.x,1)-1
        plot3([mesh.x(i,:); mesh.x(i+1,:)], [mesh.y(i,:); mesh.y(i+1,:)], [mesh.z(i,:); mesh.z(i+1,:)], 'black-', 'HandleVisibility','off')
   end

   quiver3(mesh.control.xc, mesh.control.yc, mesh.control.zc, mesh.unitVec(:,:,1), mesh.unitVec(:,:,2), mesh.unitVec(:,:,3),0.5, 'red-', 'HandleVisibility',handlevis)
   
   scatter3(mesh.control.xc(1,:), mesh.control.yc(1,:), mesh.control.zc(1,:), 10, 'o', 'red', 'HandleVisibility',handlevis)
   scatter3(mesh.control.xc(2:end,:), mesh.control.yc(2:end,:), mesh.control.zc(2:end,:), 10, 'o', 'red', 'HandleVisibility',"off")

   plot3(mesh.nodes.xq(1,:)', mesh.y(1, :)', mesh.nodes.zq(1,:)', 'blue--', 'HandleVisibility',handlevis)
   if size(mesh.y,1) > 2
       plot3(mesh.nodes.xq(2:end,:)', mesh.y(2:end-1, :)', mesh.nodes.zq(2:end,:)', 'blue--', 'HandleVisibility',"off")
   else
       plot3(mesh.nodes.xq(2:end,:)', mesh.y(2:end, :)', mesh.nodes.zq(2:end,:)', 'blue--', 'HandleVisibility',"off")
   end
   
   xlabel('x [m]')
   ylabel('y [m]')
   zlabel('y [m]')

   plot3(mesh.surface_x, mesh.surface_y, mesh.surface_z, 'green-')

   legend("Lattice", "Perpendicular Unit Vector (Aerofoil)", "Control Points", "1/4 Chord Line", "Aerofoil Camber")