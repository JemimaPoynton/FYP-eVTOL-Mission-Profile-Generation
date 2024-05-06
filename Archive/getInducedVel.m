function Vind = getInducedVel(control, nodes, N)

sizing = ones(1, N(1));

pc(:,:,1) = control.xyz(:,1)*sizing;
pc(:,:,2) = control.xyz(:,2)*sizing;
pc(:,:,3) = control.xyz(:,3)*sizing;

for j = 1:N(2)-1
    AC(:,:,1) = (nodes.xq(:,j)*sizing)'; % matrix of points A&C (depends on reference point i)
    AC(:,:,2) = (nodes.yq(:,j)*sizing)';
    AC(:,:,3) = (nodes.zq(:,j)*sizing)';

    BD(:,:,1) = (nodes.xq(:,j+1)*sizing)'; % matrix of points B&D (depends on reference point i)
    BD(:,:,2) = (nodes.yq(:,j+1)*sizing)';
    BD(:,:,3) = (nodes.zq(:,j+1)*sizing)';

    r1n = AC - pc;
    r2n = BD - pc;
    r0n = r2n - r1n;

    Vind(:,:,j,:) = inducedV_1Vortex_matrix(r0n, r1n, r2n); % solve for induced velocity
    
end

Vind(find(isnan(Vind))) = 0; % remove nan due to /0 in vector function
Vind = -squeeze(sum(Vind,3));

