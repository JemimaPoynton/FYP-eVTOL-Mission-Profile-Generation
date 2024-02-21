function mesh = combineWings(wing1, wing2)
    lift = [wing1 wing2];
%     lift = meshAll(lift, [6 4; 6 4], 1);
    
    if lift(1).sideY == 1
        if lift(2).sideY == -1
            lift = [lift(2) lift(1)]; % flip such that negative y is first
        else
            error('Wings overlap. Please check the object definition sideY.')
        end
    end
    
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
end