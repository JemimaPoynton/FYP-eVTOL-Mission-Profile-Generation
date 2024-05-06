lift = [wing1 wing2]; % combined planform of opposite wings
lift = meshAll(lift, [6 4; 6 4], 1);

if lift(1).sideY == 1
    if lift(2).sideY == -1
        lift = [lift(2) lift(1)]; % flip such that negative y is first
    else
        error('Wings overlap. Please check the object definition sideY.')
    end
end

combPlanf.mesh.x = [lift(1).mesh.x lift(2).mesh.x];
combPlanf.mesh.y = [lift(1).mesh.y lift(2).mesh.y];
combPlanf.mesh.z = [lift(1).mesh.z lift(2).mesh.z];

combPlanf.mesh.control.xc = [lift(1).mesh.control.xc lift(2).mesh.control.xc];
combPlanf.mesh.control.yc = [lift(1).mesh.control.yc lift(2).mesh.control.yc];
combPlanf.mesh.control.zc = [lift(1).mesh.control.zc lift(2).mesh.control.zc];

combPlanf.mesh.nodes.xq = [lift(1).mesh.nodes.xq lift(2).mesh.nodes.xq];
combPlanf.mesh.nodes.yq = [lift(1).mesh.nodes.xq lift(2).mesh.nodes.xq];
combPlanf.mesh.nodes.zq = [lift(1).mesh.nodes.xq lift(2).mesh.nodes.xq];

for i = 1:size(lift(1).mesh.unitVec,1)
    combPlanf.mesh.unitVec(i,:,:) = [lift(1).mesh.unitVec(i,:,:) lift(2).mesh.unitVec(i,:,:)];
end

combPlanf.S = lift(1).S + lift(2).S;

VLM(combPlanf, airflow, [6 4; 6 4], 1)