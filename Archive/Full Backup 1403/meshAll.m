function [lift, combinedmesh, N] = meshAll(lift, n, meshBool, plotMesh)
% ![DESC]!

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

%% Combine mesh
% Group in terms for correct Y direction
liftL = []; liftR = [];

for a = 1:length(lift)
    if lift(a).sideY == -1
        liftL = [liftL lift(a)];
    else
        liftR = [liftR lift(a)];
    end
end

lift = [liftL liftR];

for a = 1:length(lift)
    mesh.x = [lift(1).mesh.x(:,1:end-1) lift(2).mesh.x];
    mesh.y = [lift(1).mesh.y(:,1:end-1) lift(2).mesh.y];
    mesh.z = [lift(1).mesh.z(:,1:end-1) lift(2).mesh.z];
    
    mesh.control.xc = [lift(1).mesh.control.xc lift(2).mesh.control.xc];
    mesh.control.yc = [lift(1).mesh.control.yc lift(2).mesh.control.yc];
    mesh.control.zc = [lift(1).mesh.control.zc lift(2).mesh.control.zc];
    
    mesh.nodes.xq = [lift(1).mesh.nodes.xq(:,1:end-1) lift(2).mesh.nodes.xq];
    mesh.nodes.yq = [lift(1).mesh.nodes.yq(:,1:end-1) lift(2).mesh.nodes.yq];
    mesh.nodes.zq = [lift(1).mesh.nodes.zq(:,1:end-1) lift(2).mesh.nodes.zq];
    
    for i = 1:size(lift(1).mesh.unitVec,1)
        mesh.unitVec(i,:,:) = [lift(1).mesh.unitVec(i,:,:) lift(2).mesh.unitVec(i,:,:)];
    end

    N = size(mesh.control.xc);
end

