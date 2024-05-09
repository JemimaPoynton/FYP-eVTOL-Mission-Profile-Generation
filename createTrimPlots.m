function createTrimPlots(f, s, a, ev, u, t, trim, kt)
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
    plot(dist(idxf(1:Np*stg)), [squeeze(forces([1:3 7:9],:,1)) squeeze(forces([1:3 7:9],:,2)) squeeze(forces([1:3 7:9],:,3)) squeeze(forces([1:3 7:9],:,4)) squeeze(forces([1:3 7:9],:,5))], 'black:')
    grid on
    hold on
    % scatter(dist(idxf), zeros(size(dist(idxf))), 'x', 'red')
    
    xlabel('Distance [m]')
    ylabel('Force in Body Frame [N]')
    legend('Thrust (x)', 'Thrust (y)', 'Thrust (z)', 'Aerodynamic Load (x)', 'Aerodynamic Load (y)', 'Aerodynamic Load (z)')
    % 
    %     figure()
    % plot(dist(idxf(1:Np*stg)), [squeeze(forces([10:15],:,1)) squeeze(forces([10:15],:,2)) squeeze(forces([10:15],:,3)) squeeze(forces([10:15],:,4)) squeeze(forces([10:15],:,5))],'black:')
    % grid on
    % hold on
    % % scatter(dist(idxf), zeros(size(dist(idxf))), 'x', 'red')
    % 
    %         figure()
    % plot(dist(idxf(1:Np*stg)), [squeeze(forces([16:18],:,1)) squeeze(forces([16:18],:,2)) squeeze(forces([16:18],:,3)) squeeze(forces([16:18],:,4)) squeeze(forces([16:18],:,5))],'black:')
    % grid on
    % hold on
    % % scatter(dist(idxf), zeros(size(dist(idxf))), 'x', 'red')
    % 
    % xlabel('Distance [m]')
    % ylabel('Force in Body Frame [N]')
    % legend('Thrust (x)', 'Thrust (y)', 'Thrust (z)', 'Aerodynamic Load (x)', 'Aerodynamic Load (y)', 'Aerodynamic Load (z)')
end

%% Plot states
if s == 1
    figure()
    plot(dist(idxf(1:Np*stg)), [squeeze(X(1:3,:,1)) squeeze(X(1:3,:,2)) squeeze(X(1:3,:,3)) squeeze(X(1:3,:,4)) squeeze(X(1:3,:,5))])
    grid on
    hold on
    % scatter(dist(idxf), zeros(size(dist(idxf))), 'x', 'red')
    
    xlabel('Distance [m]')
    ylabel('uvw [N]')
    legend('u (x)', 'v (y)', 'w (z)')
    
    figure()
    plot(dist(idxf(1:Np*stg)), [squeeze(X(7:9,:,1)) squeeze(X(7:9,:,2)) squeeze(X(7:9,:,3)) squeeze(X(7:9,:,4)) squeeze(X(7:9,:,5))])
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
    plot(dist(idxf(1:Np*stg)), [squeeze(alpha(1,:,1)) squeeze(alpha(1,:,2)) squeeze(alpha(1,:,3)) squeeze(alpha(1,:,4)) squeeze(alpha(1,:,5))]*(180/pi))
    grid on
    hold on
    % scatter(dist(idxf), zeros(size(dist(idxf))), 'x', 'red')
    
    xlabel('Distance [m]')
    ylabel('alpha [deg]')
end

%% Earth frame velocity plot
if ev == 1
    figure()
    grid on
    hold on
    plot(dist(idxf(1:Np*stg)), [squeeze(uvw_e(1,:,1)) squeeze(uvw_e(1,:,2)) squeeze(uvw_e(1,:,3)) squeeze(uvw_e(1,:,4)) squeeze(uvw_e(1,:,5))], 'black-')
    plot(dist(idxf(1:Np*stg)), [squeeze(uvw_e(2,:,1)) squeeze(uvw_e(2,:,2)) squeeze(uvw_e(2,:,3)) squeeze(uvw_e(2,:,4)) squeeze(uvw_e(2,:,5))], 'black--')
    plot(dist(idxf(1:Np*stg)),[squeeze(uvw_e(3,:,1)) squeeze(uvw_e(3,:,2)) squeeze(uvw_e(3,:,3)) squeeze(uvw_e(3,:,4)) squeeze(uvw_e(3,:,5))], 'black-.')

    legend('u_e (x)', 'v_e (y)', 'w_e (z)')
    xlabel('Distance [m]')
    ylabel('Velocity in Earth Frame [m/s]')
end

%% Plot Control Inputs
if u == 1
    figure()
    grid on
    hold on
    plot(dist(idxf(1:Np*stg)), [squeeze(U(1:3,:,1)) squeeze(U(1:3,:,2)) squeeze(U(1:3,:,3)) squeeze(U(1:3,:,4)) squeeze(U(1:3,:,5))]*(180/pi))
    legend('Elevator (deg)', 'Aileron (deg)', 'Rudder (deg)')
    
    xlabel('Distance [m]')
    ylabel('Deflection [deg]')
    
    figure()
    subplot(2,1,1)
    grid on
    hold on
    plot(dist(idxf(1:Np*stg)), ([squeeze(U(4,:,1)) squeeze(U(4,:,2)) squeeze(U(4,:,3)) squeeze(U(4,:,4)) squeeze(U(4,:,5))]).^2*kt, 'black:')
    plot(dist(idxf(1:Np*stg)), ([squeeze(U(8,:,1)) squeeze(U(8,:,2)) squeeze(U(8,:,3)) squeeze(U(8,:,4)) squeeze(U(8,:,5))]).^2*kt, 'black--')
    plot(dist(idxf(1:Np*stg)), ([squeeze(U(12,:,1)) squeeze(U(12,:,2)) squeeze(U(12,:,3)) squeeze(U(12,:,4)) squeeze(U(12,:,5))]).^2*kt, 'blue--')
    plot(dist(idxf(1:Np*stg)), ([squeeze(U(16,:,1)) squeeze(U(16,:,2)) squeeze(U(16,:,3)) squeeze(U(16,:,4)) squeeze(U(16,:,5))]).^2*kt, 'b:')
    legend('Rotor 1', 'Rotor 2', 'Rotor 3', 'Rotor 4')
    ylabel('Thrust [N]')
    xlabel('Distance [m]')
    
    subplot(2,1,2)
    grid on
    hold on
    plot(dist(idxf(1:Np*stg)), [squeeze(U([6 10 14 18],:,1)) squeeze(U([6 10 14 18],:,2)) squeeze(U([6 10 14 18],:,3)) squeeze(U([6 10 14 18],:,4)) squeeze(U([6 10 14 18],:,5))]*(180/pi))
    legend('Rotor 1 (deg)', 'Rotor 2 (deg)', 'Rotor 3 (deg)', 'Rotor 4 (deg)')
    
    xlabel('Distance [m]')
    ylabel('Deflection [deg]')
end

%% Plot torque
if t == 1
    figure()
    plot(dist(idxf(1:Np*stg)), [squeeze(trim.Ut(4:6,:,1)) squeeze(trim.Ut(4:6,:,2)) squeeze(trim.Ut(4:6,:,3)) squeeze(trim.Ut(4:6,:,4)) squeeze(trim.Ut(4:6,:,4))])
    grid on
    xlabel('Distance [m]')
    ylabel('Torque [Nm]')
    legend('Torque x', 'Torque y', 'Torque z')
end