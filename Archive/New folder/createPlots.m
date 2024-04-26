function createPlots(f, s, a, ev, trim)
% function createPlots generates plot relating to optimisation of mission
% trim. The inputs are boolean corresponding to plots of
%
% f: aerodynamic and thrust loads
% s: the aircraft states
% a: angle of attack
% ev: earth frame velocities

%% Extract Data for Readibility
dist = trim.dist; Np = trim.Np; stg = trim.stg; idxf = trim.idxf;
forces = trim.forces; U = trim.U; uvw_e = trim.uvw_e; X = trim.X; alpha = trim.alpha;

%% Force plots
if f == 1
    figure()
    plot(dist(idxf(1:Np*stg)), [squeeze(forces([1:3 7:9],1,:)) squeeze(forces([1:3 7:9],2,:)) squeeze(forces([1:3 7:9],3,:)) squeeze(forces([1:3 7:9],4,:)) squeeze(forces([1:3 7:9],5,:))])
    grid on
    hold on
    % scatter(dist(idxf), zeros(size(dist(idxf))), 'x', 'red')
    
    xlabel('Distance [m]')
    ylabel('Force in Body Frame [N]')
    legend('Thrust (x)', 'Thrust (y)', 'Thrust (z)', 'Aerodynamic Load (x)', 'Aerodynamic Load (y)', 'Aerodynamic Load (z)')
end

%% Plot states
if s == 1
    figure()
    plot(dist(idxf(1:Np*stg)), [squeeze(X(1:3,1,:)) squeeze(X(1:3,2,:)) squeeze(X(1:3,3,:)) squeeze(X(1:3,4,:)) squeeze(X(1:3,5,:))])
    grid on
    hold on
    % scatter(dist(idxf), zeros(size(dist(idxf))), 'x', 'red')
    
    xlabel('Distance [m]')
    ylabel('uvw [N]')
    legend('u (x)', 'v (y)', 'w (z)')
    
    figure()
    plot(dist(idxf(1:Np*stg)), [squeeze(X(7:9,1,:)) squeeze(X(7:9,2,:)) squeeze(X(7:9,3,:)) squeeze(X(7:9,4,:)) squeeze(X(7:9,5,:))])
    grid on
    hold on
    % scatter(dist(idxf), zeros(size(dist(idxf))), 'x', 'red')
    
    xlabel('Distance [m]')
    ylabel('angle [rad]')
    legend('phi', 'theta', 'psi')
end

%% Plot alpha
if a == 1
    figure()
    plot(dist(idxf(1:Np*stg)), [squeeze(alpha(1,1,:))' squeeze(alpha(1,2,:))' squeeze(alpha(1,3,:))' squeeze(alpha(1,4,:))' squeeze(alpha(1,5,:))'])
    grid on
    hold on
    % scatter(dist(idxf), zeros(size(dist(idxf))), 'x', 'red')
    
    xlabel('Distance [m]')
    ylabel('alpha [rad]')
end

%% Earth frame velocity plot
if ev == 1
    figure()
    grid on
    hold on
    plot(dist(idxf(1:Np*stg)), [squeeze(uvw_e(1,1,:))' squeeze(uvw_e(1,2,:))' squeeze(uvw_e(1,3,:))' squeeze(uvw_e(1,4,:))' squeeze(uvw_e(1,5,:))'], 'black-')
    plot(dist(idxf(1:Np*stg)), [squeeze(uvw_e(2,1,:))' squeeze(uvw_e(2,2,:))' squeeze(uvw_e(2,3,:))' squeeze(uvw_e(2,4,:))' squeeze(uvw_e(2,5,:))'], 'black--')
    plot(dist(idxf(1:Np*stg)),[squeeze(uvw_e(3,1,:))' squeeze(uvw_e(3,2,:))' squeeze(uvw_e(3,3,:))' squeeze(uvw_e(3,4,:))' squeeze(uvw_e(3,5,:))'], 'black-.')

    legend('u_e (x)', 'v_e (y)', 'w_e (z)')
    xlabel('Distance [m]')
    ylabel('Velocity in Earth Frame [m/s]')
end