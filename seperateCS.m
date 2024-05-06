function [meshs1, meshs2, meshs3] = seperateCS(mesh, nCx, nCy, wing, N)

meshs1.x = mesh.x(:, nCy(1):nCy(end)+1);
meshs1.y = mesh.y(:, nCy(1):nCy(end)+1);
meshs1.z = mesh.z(:, nCy(1):nCy(end)+1);
meshs1.control.xc = mesh.control.xc(:, nCy(1):nCy(end)); 
meshs1.control.yc = mesh.control.yc(:, nCy(1):nCy(end)); 
meshs1.control.zc = mesh.control.zc(:, nCy(1):nCy(end));
meshs1.nodes.xq = mesh.nodes.xq(:, nCy(1):nCy(end)+1); 
meshs1.nodes.yq = mesh.nodes.yq(:, nCy(1):nCy(end)+1);
meshs1.nodes.zq = mesh.nodes.zq(:, nCy(1):nCy(end)+1);
meshs1.unitVec = mesh.unitVec(:, nCy(1):nCy(end),:);

meshs1.surface_x = mesh.surface_x(:, nCy(1):nCy(end));
meshs1.surface_y = mesh.surface_y(:, nCy(1):nCy(end));
meshs1.surface_z = mesh.surface_z(:, nCy(1):nCy(end));

meshs2.x = mesh.x(:, 1:nCy(1));
meshs2.y = mesh.y(:, 1:nCy(1));
meshs2.z = mesh.z(:, 1:nCy(1));
meshs2.control.xc = mesh.control.xc(:, 1:nCy(1)-1);
meshs2.control.yc = mesh.control.yc(:, 1:nCy(1)-1);
meshs2.control.zc = mesh.control.zc(:, 1:nCy(1)-1);
meshs2.nodes.xq = mesh.nodes.xq(:, 1:nCy(1)); 
meshs2.nodes.yq = mesh.nodes.yq(:, 1:nCy(1));
meshs2.nodes.zq = mesh.nodes.zq(:, 1:nCy(1));
meshs2.unitVec = mesh.unitVec(:, 1:nCy(1)-1,:);

meshs2.surface_x = mesh.surface_x(:, 1:nCy(1)-1);
meshs2.surface_y = mesh.surface_y(:, 1:nCy(1)-1);
meshs2.surface_z = mesh.surface_z(:, 1:nCy(1)-1);

meshs3.x = mesh.x(:, nCy(end)+1:end); 
meshs3.y = mesh.y(:, nCy(end)+1:end);
meshs3.z = mesh.z(:, nCy(end)+1:end);
meshs3.control.xc = mesh.control.xc(:, nCy(end)+1:end);
meshs3.control.yc = mesh.control.yc(:, nCy(end)+1:end);
meshs3.control.zc = mesh.control.zc(:, nCy(end)+1:end);
meshs3.nodes.xq = mesh.nodes.xq(:, nCy(end)+1:end);
meshs3.nodes.yq = mesh.nodes.yq(:, nCy(end)+1:end);
meshs3.nodes.zq = mesh.nodes.zq(:, nCy(end)+1:end);
meshs3.unitVec = mesh.unitVec(:, nCy(end)+1:end,:);

meshs3.surface_x = mesh.surface_x(:, nCy(end)+1:end);
meshs3.surface_y = mesh.surface_y(:, nCy(end)+1:end);
meshs3.surface_z = mesh.surface_z(:, nCy(end)+1:end);

%% Deflect Control Surface Panels
if ~isempty(wing.CSs)
    Lt = transMatrix([0 -wing.CSs(1).deflection 0]);

    if wing.CSs(1).edge == "TE"

        hinge = [meshs1.x(N(1) - nCx(end), :); meshs1.y(N(1) - nCx(end), :); meshs1.z(N(1) - nCx(end), :)];
        % modify lattice points

        for ii = 1:nCx(end)
            meshp = [meshs1.x(N(1) - nCx(end) + ii, :); meshs1.y(N(1) - nCx(end) + ii, :); meshs1.z(N(1) - nCx(end) + ii, :)]; 
            meshp_rot = hinge + ((meshp - hinge)'*Lt)';

            meshs1.x(N(1) - nCx(end) + ii, :) = meshp_rot(1,:);
            meshs1.y(N(1) - nCx(end) + ii, :) = meshp_rot(2,:);
            meshs1.z(N(1) - nCx(end) + ii, :) = meshp_rot(3,:);
        end

        for ii = 1:nCx(end)
            meshpq = [meshs1.nodes.xq(N(1) - nCx(end) + ii - 1, :); meshs1.nodes.yq(N(1) - nCx(end) + ii - 1, :); meshs1.nodes.zq(N(1) - nCx(end) + ii - 1, :)]; 
            meshp_rotq = hinge + ((meshpq - hinge)'*Lt)';

            meshs1.nodes.xq(N(1) - nCx(end) + ii - 1, :) = meshp_rotq(1,:);
            meshs1.nodes.yq(N(1) - nCx(end) + ii - 1, :) = meshp_rotq(2,:);
            meshs1.nodes.zq(N(1) - nCx(end) + ii - 1, :) = meshp_rotq(3,:);

            hingexc = hinge(:,1:end-1) + [diff(hinge(1,:),1); diff(hinge(1,:),1); diff(hinge(1,:),1)]/2;

            meshp = [meshs1.control.xc(N(1) - nCx(end) + ii - 1, :); meshs1.control.yc(N(1) - nCx(end) + ii - 1, :); meshs1.control.zc(N(1) - nCx(end) + ii - 1, :)]; 
            meshp_rot = hingexc + ((meshp - hingexc)'*Lt)';

            meshs1.control.xc(N(1) - nCx(end) + ii - 1, :) = meshp_rot(1,:);
            meshs1.control.yc(N(1) - nCx(end) + ii - 1, :) = meshp_rot(2,:);
            meshs1.control.zc(N(1) - nCx(end) + ii - 1, :) = meshp_rot(3,:);
            
            meshuv = [meshs1.unitVec(N(1) - nCx(end) + ii - 1, :,1); meshs1.unitVec(N(1) - nCx(end) + ii - 1, :,2); meshs1.unitVec(N(1) - nCx(end) + ii - 1, :,3)];
            meshs1.unitVec(N(1) - nCx(end) + ii - 1, :, :) = meshuv'*Lt;

            meshsurf = [meshs1.surface_x(N(1) - nCx(end) + ii - 1, :); meshs1.surface_y(N(1) - nCx(end) + ii - 1, :); meshs1.surface_z(N(1) - nCx(end) + ii - 1, :)]; 
            meshp_surf = hingexc + ((meshsurf - hingexc)'*Lt)';

            meshs1.surface_x(N(1) - nCx(end) + ii - 1, :) = meshp_surf(1,:);
            meshs1.surface_y(N(1) - nCx(end) + ii - 1, :) = meshp_surf(2,:);
            meshs1.surface_z(N(1) - nCx(end) + ii - 1, :) = meshp_surf(3,:);
        end

    else
    end

end