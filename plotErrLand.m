clear J
aircraft = VX4;

idx = 34;
dE = linspace(-0.7, 0, 30);
rpitch = X(6,idx) + linspace(-0.3, 0.3, 30);
uv = X(3,idx) + linspace(-15, 15, 30);
rT = linspace(0, 1000, 30);
rTy = linspace(0, 1000, 30);

% for n = 1:length(uv)
    for i = 1:length(rT)
        for j = 1:length(dE)
            
            Z = [12.5005000000000
0
3.06569575161722
-5.14525921392220e-08
0.130830528368235
0
0.367355776124015
0
0
0
0
0
0
0.892076813142133]';
            Z([10 11]) = 0 + rT(:,i);
            Z([12 13]) = 0 + rTy(:,j);
            % Z(5) = dE(:,j);
            u = 12.5005;
            J(i,j) = trimCost(aircraft, Z', u, coefficients, 1.225, VX4.refGeo, VX4.m,thrustobj2struct(aircraft, zeros(1,4)), VX4.CG, VX4.I, [0.3137 0], 0.9912);
        end
    end
    % surf(dE, rT(1,:), J)
    % pause(0.01)
% end

figure()
surf(rT(1,:), rTy(1,:), J)

figure()
plot(rT(1,:), J(:,1))

figure()
plot(dE, J(1,:))


