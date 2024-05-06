function [lift, CL, CD, CM, distribution, AC] = VLM(lift, mesh, airflow, N, plotBool, xref, Sref, Cref)
% function VLM applies the Vortex Lattice Method to the object 'wing' or 
% group of wings 'lift' to get the total lift and drag coeffient
% 
% airflow: structure containing angle of attack (alpha), sideslip angle
% xref: reference point for moments, usually the CG.
% CL: lift coefficient
% CD: drag coefficient
% distribution: struct containing local lift and drag coefficients
% N: number of panels in [x and y]
% m,n: indices for panel m steps along x (chord), and n steps along y (span)
% i,j: general indexing, or panel number along span left-to-right and down
%      from LE, e.g.  1 2 3 4  5  6     y->
%                     7 8 9 10 11 12    x↓

%% Initalise sizing
pc = zeros(N(1), N(2), 3);
A = zeros(N(1), N(2), 3); B = A; C = A; D = A;

%% Calculate Induced Velocity
% A(m,n): coordinates of inboard forward point on panel [m n], corresponding
%          to [x y] direction.
%
% B(m,n): coordinates of outboard forward point on panel [m n], corresponding
%          to [x y] direction.
% 
% C(m,n): coordinates of inboard forward point on panel [ji jj], 
%          (including -> inf downstream) corresponding to [x y] direction.
%
% D(m,n): coordinates of inboard forward point on panel [ji jj], 
%          (including -> inf downstream) corresponding to [x y] direction.


% extract data for readibility
control = mesh.control;
nodes = mesh.nodes;

for m = 1:N(1) % panel i position in x 
    for n = 1:N(2) % panel i position in y
        
        pc(m,n,:) = [control.xc(m,n) control.yc(m,n) control.zc(m,n)]; % current control point

        for ji = 1:N(1) % i index of panel j (vortex influencing panel mn (i))
            for jj = 1:N(2) % j index of panel j
                
                % Define reference points for vectors
                A(m,n,:) = [nodes.xq(ji,   jj)   nodes.yq(ji,   jj)   nodes.zq(ji,   jj)  ];
                B(m,n,:) = [nodes.xq(ji,   jj+1) nodes.yq(ji,   jj+1) nodes.zq(ji,   jj+1)];
                C(m,n,:) = [nodes.xq(ji+1, jj+1) nodes.yq(ji+1, jj+1) nodes.zq(ji+1, jj+1)];
                D(m,n,:) = [nodes.xq(ji+1, jj)   nodes.yq(ji+1, jj)   nodes.zq(ji+1, jj)  ];

                % Bound vortex
                r0_BV = squeeze(A(m,n,:))' - squeeze(B(m,n,:))';
                r1_BV = squeeze(A(m,n,:) - pc(m,n,:))';
                r2_BV = squeeze(B(m,n,:) - pc(m,n,:))';

                V(1,1:3) = inducedV_1Vortex(r0_BV, r1_BV, r2_BV);

                % Right Trailing vortex
                r0_RT = squeeze(B(m,n,:) - C(m,n,:))';
                r1_RT = r2_BV;
                r2_RT = squeeze(C(m,n,:) - pc(m,n,:))';

                V(2,1:3) = inducedV_1Vortex(r0_RT, r1_RT, r2_RT);

                % Free Stream Vortex
                r0_FV = squeeze(C(m,n,:) - D(m,n,:))';
                r1_FV = r2_RT;
                r2_FV = squeeze(D(m,n,:) - pc(m,n,:))';
 
                V(3,1:3) = inducedV_1Vortex(r0_FV, r1_FV, r2_FV);

                % Left Trailing Vortex
                r0_LT = squeeze(D(m,n,:) - A(m,n,:))';
                r1_LT = r2_FV;
                r2_LT = squeeze(A(m,n,:) - pc(m,n,:))';

                V(4,1:3) = inducedV_1Vortex(r0_LT, r1_LT, r2_LT);

                % Induced velocity
                Vind = sum(V);

                i = m + N(1)*(n-1); % count along m, in 'row' n
                j = ji + N(1)*(jj-1); % count along ji, in 'row' jj

                Ainfl(i,j) = Vind(end); % influence coefficient representing the induced flow on panel i due to the vortex on panel j

%                 scatter3(pc(m,n,1), pc(m,n,2), pc(m,n,3))
% %                 quiver3(pc(m,n,1), pc(m,n,2), pc(m,n,3), r1_BV(1), r1_BV(2), r1_BV(3))
%                 quiver3(pc(m,n,1), pc(m,n,2), pc(m,n,3), r1_FV(1), r1_FV(2), r1_FV(3))
% %                 quiver3(pc(m,n,1), pc(m,n,2), pc(m,n,3), r1_LT(1), r1_LT(2), r1_LT(3))
% %                 quiver3(pc(m,n,1), pc(m,n,2), pc(m,n,3), r1_RT(1), r1_RT(2), r1_RT(3))
% %                 scatter3(A(m,n,1), A(m,n,2), A(m,n,3))
% %                 scatter3(B(m,n,1), B(m,n,2), B(m,n,3))
%                 scatter3(C(m,n,1), C(m,n,2), C(m,n,3))
% %                 scatter3(D(m,n,1), D(m,n,2), D(m,n,3))
            end
        end
    end
end

%% Calculate Circulation
unitVec = mesh.unitVec; % extract for readibility

Rwb = body2wind(airflow.alpha, airflow.beta); % rotation matrix from wind to body frame
Uinf = (airflow.U*Rwb*[1; 0; 0])'; % get Uinf from U in body frame (forward flight speed in x only)

for m = 1:N(1) % Calculate circulation of each panel i
    for n = 1:N(2)
        nUinf = Uinf*squeeze(unitVec(m,n,:)); % get normal component of Uinf 

        i = m + N(1)*(n-1);
        w(i,:) = -nUinf;
    end
end

circ = Ainfl\w;

% Reshape circulation corresponding to mn
for m = 1:N(1)
    for n = 1:N(2)
        circnew(m,n) = circ(m + N(1)*(n-1));
    end
end

circ = [circnew(1,:); circnew(2:end,:) - circnew(1:end-1,:)];

%% Calculate Local Lift and Drag (both spanwise and chordwise)
for n = 1:N(2)
    for m = 1:N(1)
        uVec = squeeze(unitVec(m,n,:)); % current unit vector

        c_inb = mesh.x(m+1,n) - mesh.x(m,n); % inboard chord on panel m,n
        c_ob = mesh.x(m+1,n+1) - mesh.x(m,n+1);
        c_mn(m,n) = (c_inb + c_ob)/2; % Mean chord on panel i,j

        if control.yc(m,n) > 1 && length(lift) > 1
            c_ = getChord(lift(2), control.yc(m,n)); % full chord at span position n
        else
            c_ = getChord(lift(1), control.yc(m,n));
        end

        sab = (Uinf/airflow.U); % angles between resultant force and drag i.e. [sin(alpha) sin(beta) ~]
        
        cp(m,n) = 2*circ(m,n)/airflow.U

        % sc: local to panel mn
        cd_mn(m,n) = (cp(m,n)/c_mn(m,n))*sab*uVec; % non-dimensional perpendicular (pressure) force * resolve drag components
        cl_mn(m,n) = norm((cp(m,n)/c_mn(m,n))*uVec' - cd_mn(m,n)*sab); % magnitude of non-dimensional perpendicular (pressure) force - drag component
        
        % non dimensionalise wrt full chord at span position n
        cd_mn_nd(m,n) = cd_mn(m,n)*(c_mn(m,n)/c_);
        cl_mn_nd(m,n) = cl_mn(m,n)*(c_mn(m,n)/c_);
    end

    cd(n) = sum(cd_mn_nd(:,n));
    cl(n) = sum(cl_mn_nd(:,n));

end

%% Calculate Total Lift and Drag
% calculate area of each section of span
CL = 0;
CD = 0;

for j = 1:N(2)
    b(j) = mesh.y(1,j+1) - mesh.y(1,j); % change in span

    if control.yc(1,j) > 1 && length(lift) > 1
        c = getChord(lift(2), control.yc(1,j)); % full chord at span position n
    else
        c = getChord(lift(1), control.yc(1,j));
    end

    area(j) = b(j)*c;

    CL = CL + (1/Sref)*area(j)*cl(j);
    CD = CD + (1/Sref)*area(j)*cd(j);
end

%% Calculate Pitching Moment
m0 = zeros(N(1), N(2));

for m = 1:N(1)
    for n = 1:N(2)
        c_inb = mesh.x(m+1,n) - mesh.x(m,n); % inboard chord on panel m,n
        c_ob = mesh.x(m+1,n+1) - mesh.x(m,n+1);
        c_mn(m,n) = (c_inb + c_ob)/2; % Mean chord on panel m,n

        A = b(n)*c_mn(m,n); % area of panel

        x_ma = ((mesh.nodes.xq(m,n) - mesh.x(1,N(2)/2+1)) + (mesh.nodes.xq(m,n+1) - mesh.x(1,N(2)/2+1)))/2; % x moment arm (local to panel m,n only)  
        x_ma = x_ma - xref;

        m0(m,n) = -cl_mn(m,n)*x_ma*A; % pitching moment of panel m,n
    end
end

M = sum(sum(m0)); % total pitching moment about LE

CM = M*(1/(Sref*Cref)); % correcting non-dimensionalisation

AC = (CM/CL)*Cref; % distance of aerodynamic centre from LE

%% Store local distribution
distribution.cd = [control.yc(1,1:N(2))', cd']; % store local distribution
distribution.cl = [control.yc(1,1:N(2))', cl'];

%% Plots
if plotBool == 1
    figure()
%     subplot(2,1,1)
    plot(control.yc(1,1:N(2)), cl)
    grid on
    ylabel('Local Lift Coefficient'); xlabel('Span')

%     subplot(2,1,2)
%     plot(control.yc(1,1:N(2)), cd)
%     grid on
%     ylabel('Local Drag Coefficient'); xlabel('Span')
end


