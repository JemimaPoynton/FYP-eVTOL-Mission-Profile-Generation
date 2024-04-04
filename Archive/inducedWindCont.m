function wind = inducedWindCont(mesh, circ, Uinf, N)

sz = size(mesh.nodes.xq,1);

vs1 = [mesh.nodes.xq(:,N(2)/2) mesh.nodes.yq(:,N(2)/2) mesh.nodes.zq(:,N(2)/2)];
vs2 = [mesh.nodes.xq(:,N(2)/2 + 1) mesh.nodes.yq(:,N(2)/2 + 1) mesh.nodes.zq(:,N(2)/2 + 1)];

mesh.control.xyz = (vs2 + vs1)./2;

Vind = getInducedVel(mesh.control, mesh.nodes, N);

indV_cont(:,1) = Vind(:,:,1)*circ; % induced velocity contribution to wind
indV_cont(:,2) = Vind(:,:,2)*circ;
indV_cont(:,3) = Vind(:,:,3)*circ;

for i = 1:sz
    wind(i,:) = Uinf - squeeze(indV_cont(i,:));
end
