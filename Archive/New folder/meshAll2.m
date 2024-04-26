function [lift, mesh, N] = meshAll2(lift, n, meshBool, plotMesh)
% ![DESC]! Lift must contain inputs in symmetrical pairs

N = n+1;

if size(N,1) < length(lift)
    N = [N;
         ones(size(lift,2) - size(N,1),2).*N(size(N,1), 1:2)];
end

for a = 1:length(lift) % iterate through lift object
    if meshBool == 1
        lift(a) = createMesh3D(lift(a), N(a,:), plotMesh, a);
    end
end

%% Group in terms sideY for correct Y direction
liftL = []; liftR = [];

for a = 1:length(lift)
    if lift(a).sideY == -1
        liftL = [liftL lift(a)];
    else
        liftR = [liftR lift(a)];
    end
end

lift = [liftL' liftR'];

%% Combine Mesh
mesh.x = []; mesh.y = []; mesh.z = []; mesh.control.xc = []; mesh.control.yc = []; mesh.control.zc = [];
mesh.nodes.xq = []; mesh.nodes.yq = []; mesh.nodes.zq = []; mesh.unitVec = [];

for i = 1:size(lift,1) % rows
    for j = 1:size(lift,2) % columns
        
        if mod(j,2)==0 % if j is even
            ii = N(1,1)*(i-1) + 1;
            jj = N(1,2)*(j-1);

            mesh.x(ii:ii+N(1,1)-1, jj:jj+N(1,2)-1) = lift(i,j).mesh.x;
            mesh.y(ii:ii+N(1,1)-1, jj:jj+N(1,2)-1) = lift(i,j).mesh.y;
            mesh.z(ii:ii+N(1,1)-1, jj:jj+N(1,2)-1) = lift(i,j).mesh.z;

            ii = (N(1,1)-1)*(i-1) + 1;

            mesh.nodes.xq(ii:ii+N(1,1)-2, jj:jj+N(1,2)-1) = lift(i,j).mesh.nodes.xq(1:end-1,:);
            mesh.nodes.yq(ii:ii+N(1,1)-2, jj:jj+N(1,2)-1) = lift(i,j).mesh.nodes.yq(1:end-1,:);
            mesh.nodes.zq(ii:ii+N(1,1)-2, jj:jj+N(1,2)-1) = lift(i,j).mesh.nodes.zq(1:end-1,:);

        else
            ii = N(1,1)*(i-1) + 1;
            jj = (N(1,2)-1)*(j-1) + 1;

            mesh.x(ii:ii+N(1,1)-1, jj:jj+N(1,2)-2) = lift(i,j).mesh.x(:,1:end-1);
            mesh.y(ii:ii+N(1,1)-1, jj:jj+N(1,2)-2) = lift(i,j).mesh.y(:,1:end-1);
            mesh.z(ii:ii+N(1,1)-1, jj:jj+N(1,2)-2) = lift(i,j).mesh.z(:,1:end-1);

            ii = (N(1,1)-1)*(i-1) + 1;

            mesh.nodes.xq(ii:ii+N(1,1)-2, jj:jj+N(1,2)-2) = lift(i,j).mesh.nodes.xq(1:end-1,1:end-1);
            mesh.nodes.yq(ii:ii+N(1,1)-2, jj:jj+N(1,2)-2) = lift(i,j).mesh.nodes.yq(1:end-1,1:end-1);
            mesh.nodes.zq(ii:ii+N(1,1)-2, jj:jj+N(1,2)-2) = lift(i,j).mesh.nodes.zq(1:end-1,1:end-1);
        end

            ii = (N(1,1)-1)*(i-1) + 1;
            jj = (N(1,2)-1)*(j-1) + 1;
            
            mesh.control.xc(ii:ii+N(1,1)-2, jj:jj+N(1,2)-2) = lift(i,j).mesh.control.xc;
            mesh.control.yc(ii:ii+N(1,1)-2, jj:jj+N(1,2)-2) = lift(i,j).mesh.control.yc;
            mesh.control.zc(ii:ii+N(1,1)-2, jj:jj+N(1,2)-2) = lift(i,j).mesh.control.zc; 
            
            mesh.unitVec(ii:ii+N(1,1)-2, jj:jj+N(1,2)-2,:) = lift(i,j).mesh.unitVec;
    end
end

unitVec = mesh.unitVec;
N = size(mesh.control.xc);
n = N - 1;

mesh.nodes.xq = [mesh.nodes.xq; (mesh.nodes.xq(N(1)-1,:) - mesh.nodes.xq(N(1)-2,:))*40];
mesh.nodes.yq = [mesh.nodes.yq; mesh.nodes.yq(end,:)];
mesh.nodes.zq = [mesh.nodes.zq; (mesh.nodes.zq(N(1)-1,:) - mesh.nodes.zq(N(1)-2,:))*40];

for i = 1:length(lift)
    span = mesh.control.yc((i-1)*n(1)+1,:);

    for j = 1:length(span)
        mesh.chordDist((i-1)*n(1)+1:i*n(1),j) = getChord(lift(i), span(1,j))*ones(n(1),1);
    end
end

end


