function [lift, coeff, distribution, AC] = VLMV3(lift, mesh, airflow, N, plotBool, xyzref, Sref, Cref, bref, N_obj)
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

q = 0.5*1.225*airflow.U^2; % dynamic pressure

F_w = Rbw*F_b'; % total force in wind frame

D = F_w(1); Y = F_w(2); L = F_w(3);
CD = D/(q*Sref); CY = Y/(q*Sref); CL = L/(q*Sref);

dD = squeeze(dF_w(1,:,:)); dY = squeeze(dF_w(2,:,:)); dL = squeeze(dF_w(3,:,:)); % local
cd = dD/q; cy = dY/q; cl = dL/q; %% local to both span and chord

%% Calculate Pitching Moment
m = zeros(N(1), 3);
M = zeros(1, 3);

for i = 1:N(1) % get moment arm from bound vortex to x ref for each panel
    x_ma(i,:) = (mesh.nodes.xq(i,N(2)/2) + mesh.nodes.xq(i,N(2)/2+1))/2 - xyzref(1); % local moment arm from vortex (mean) to xref 
    y_ma(i,:) = (mesh.nodes.yq(i,N(2)/2) + mesh.nodes.yq(i,N(2)/2+1))/2 - xyzref(2);
    z_ma(i,:) = (mesh.nodes.zq(i,N(2)/2) + mesh.nodes.zq(i,N(2)/2+1))/2 - xyzref(2);

    m_arm = [x_ma y_ma z_ma];
end

M = sum(cross(m_arm,squeeze(dF_b)'));
CM = M./(q*Sref); % non-dimensionalise by dynamic pressure (NOT COMPLETELY)

Cl = CM(:,1)/bref; Cm = CM(:,2)/Cref; Cn = CM(:,3)/bref; 

AC = M/L; % distance of aerodynamic centre from reference point

%% Store local distribution
distribution.cd = [control.xyz, cd]; % store local distribution 
distribution.cl = [control.xyz, cl];
distribution.cy = [control.xyz, cy]; % store local distribution

%% Store coefficients
coeff.CD = CD; coeff.CY = CY; coeff.CL = CL; 
coeff.Cl = Cl; coeff.Cm = Cm; coeff.Cn = Cn;

%% Plots
if plotBool == 1
    figure()
    subplot(3,1,1)
    scatter(control.xyz(:,2), cl)
    hold on

%     for m = 1:size(lift,1)
%         for n = 1:size(lift,2)
%             idx = (m-1)*size(lift,2) + n;
% 
%             plot(lift(m,n).mesh.control.yc(1,:), cl_n(1:N_obj(idx,2),idx))
%         end
%     end

    grid on
    ylabel('Local Lift Coefficient'); xlabel('Span')

    subplot(3,1,2)
    scatter(control.xyz(:,2), cd)
    grid on
    ylabel('Local Drag Coefficient'); xlabel('Span')

    subplot(3,1,3)
    scatter(control.xyz(:,2), cd)
    grid on
    ylabel('Local Drag Coefficient'); xlabel('Span')
end


