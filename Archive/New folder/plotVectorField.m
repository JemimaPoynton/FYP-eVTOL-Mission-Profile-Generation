%% Set Variables
kc = 0.02;
kg = 0.02;
ca_inf = pi/4;
g_inf = pi/4;

%% Generate Reference Mesh
Nmp = 20; % Number of mesh points in each direction

latMesh = zeros(Nmp, 2);
latMesh(:,1) = linspace(0, 220, Nmp);
latMesh(:,2) = linspace(0, 30, Nmp);

longMesh = [];

%% Calculate Demand Points
% Lateral 
for i = 1:size(latMesh,1)
    for j = 1:size(latMesh,2)
        pos = [latMesh(i,1); latMesh(j,2); 0];
        [ca_d(i,j), ~] = createDemandVecVF(mission, pos, kc, kg, ca_inf, g_inf);
    end
end

% Longitudinal
% for i = 1:length(longMesh,1)
%     for j = 1:length(longMesh,2)
%         [~, gam_d] = createDemandVecVF(mission, longMesh(i,j), kc, kg, ca_inf, g_inf);
%     end
% end

%% Create Vector Plot
figure()
scatter(latMesh(:,1), latMesh(:,2))
hold on; grid on;
quiver
