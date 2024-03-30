function [lift, coeff, distribution, AC] = VLMV3(lift, mesh, airflow, N, plotBool, xref, Sref, Cref)
% function VLM applies the Vortex Lattice Method to the object 'wing' or 
% group of wings 'lift' to get the total lift and drag coeffient. Version 3
% handles the mesh as a linear matrix i.e. meshes 1 -> n, rather than (1,1)
% -> (m,n) to allow for variation in the number of panels spanwise for each
% component.
% 
% airflow: structure containing angle of attack (alpha), sideslip angle
% xref: reference point for moments, usually the CG.
% CL: lift coefficient
% CD: drag coefficient
% distribution: struct containing local lift and drag coefficients
% n: number of panels in [x and y]
% m,n: indices for panel m steps along x (chord), and n steps along y (span)
% i,j: general indexing, or panel number along span left-to-right and down
%      from LE, e.g.  1 2 3 4  5  6     y->
%                     7 8 9 10 11 12    x↓

%% Initalise sizing
% pc = zeros(N(1), N(2), 3);
% A = zeros(N(1), N(2), 3); B = A; C = A; D = A;

%% Calculate Induced Velocity
% extract data for readibility
control = mesh.control;
nodes = mesh.nodes;

sizing = ones(1, N(1));

nVec(:,:,1) = mesh.unitVec(:,1)*sizing;
nVec(:,:,2) = mesh.unitVec(:,2)*sizing;
nVec(:,:,3) = mesh.unitVec(:,3)*sizing;

Vind = -getInducedVel(control, nodes, N);
Ainfl = sum(Vind.*nVec,3); % influence coefficient

%% Calculate Circulation
unitVec = mesh.unitVec; % extract for readibility

Rbw = body2wind_aerofoil(airflow.alpha, airflow.beta); % rotation matrix from wind to body frame
Uinf = (airflow.U*Rbw)'; % aligning with wind

for m = 1:N(1) % Calculate circulation of each panel i
    nUinf = Uinf.*(unitVec(m,:)); % get normal component of Uinf 
    w(m,:) = -nUinf;
end

circ = Ainfl\w(:,3); % circulation on panel i

%% Calculate Resultant Force
F_n = 0; F_b = 0; L = 0; D = 0;

wind = inducedWindCont(mesh, circ, Uinf, N);

% Vortex span
vs1 = [mesh.nodes.xq(:,N(2)/2) mesh.nodes.yq(:,N(2)/2) mesh.nodes.zq(:,N(2)/2)];
vs2 = [mesh.nodes.xq(:,N(2)/2 + 1) mesh.nodes.yq(:,N(2)/2 + 1) mesh.nodes.zq(:,N(2)/2 + 1)];

b = sqrt(sum((vs2 - vs1).^2,2)); % vector norm

vortVec = (vs2 - vs1)./b; % align along panel
Rbw = body2wind(airflow.alpha, airflow.beta); % convert to wind frame 

for i = 1:N(1)
    dF_b(:,:,i) = cross(wind(i,:), 1.225*circ(i)*vortVec(i,:).*b(i)); % force on panel i in body frame
    dF_w(:,:,i) = squeeze(Rbw*dF_b(:,:,i)');

    F_n = F_n + dF_b(:,:,i)*unitVec(i,:)'; % normal forces
    F_b = F_b + dF_b(:,:,i); % body forces
end

q = 0.5*1.225*airflow.U^2*Sref; % dynamic pressure

F_w = Rbw*F_b'; % total force in wind frame

D = -F_w(1); Y = F_w(2); L = F_w(3);
CD = D/q; CY = Y/q; CL = L/q;

dD = dF_w(:,:,1); dY = dF_w(:,:,2); dL = dF_w(:,:,1); % local
cd = dD/q; cy = dY/q; cl = dL/q;

%% Calculate Pitching Moment
% m = zeros(N(1), N(2));
% M = 0;
% 
% for i = 1:N(1)
%     x_ma = mesh.control.xyz(i,:) - xref; % x moment arm (local to panel m,n only)  
% 
%     m(i) = cross(x_ma, L, 3) ; % multiply forces by moment arm in [x y z]
%     M = M + m(i); % running total moment
% end
% 
% CM = M./q; % non-dimensionalise by dynamic pressure (NOT COMPLETELY)
% 
% Cl = CM(:,1)/bref; Cm = CM(:,2)/cref; Cn = CM(:,3)/bref; 
% 
% AC = (CM/CL)*Cref; % distance of aerodynamic centre from reference point

%% Store local distribution
distribution.cd = [control.yc(1,1:N(2))', cd']; % store local distribution
distribution.cl = [control.yc(1,1:N(2))', cl'];
distribution.cy = [control.yc(1,1:N(2))', cy']; % store local distribution

%% Store coefficients
coeff.CD = CD; coeff.CY = CY; coeff.CL = CL; 
coeff.Cl = Cl; coeff.Cm = Cm; coeff.Cn = Cn;

%% Plots
if plotBool == 1
    figure()
%     subplot(2,1,1)
    plot(control.yc(1,1:N(2)), squeeze(dF_b(:,3,:)))
    grid on
    ylabel('Local Lift Coefficient'); xlabel('Span')

%     subplot(2,1,2)
%     plot(control.yc(1,1:N(2)), cd)
%     grid on
%     ylabel('Local Drag Coefficient'); xlabel('Span')
end


