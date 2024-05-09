out = sim('Cruise_Model_V16_LQI.slx');

subplot(2,1,1)
grid on
hold on
plot(out.simout.Time, out.simout2.Data(:,1:3));
xlabel('Time [s]')
ylabel('Velocity in Body Frame [m/s]')
legend('u', 'v', 'w')

subplot(2,1,2)
grid on
hold on
plot(out.simout.Time, out.simout.Data(:,7:9).*180/pi);
plot(out.simout.Time, out.simout1.Data(:,7:9).*180/pi);
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('Roll reference','Pitch reference','Yaw reference','Roll', 'pitch', 'yaw')

figure()
subplot(3,1,1)
grid on
hold on
plot(out.simout.Time, out.simout3.Data(:,[1 2]).*180/pi);
plot(out.simout.Time, out.simout4.Data(:,[1 2]).*180/pi);
xlabel('Time [s]')
ylabel('Deflection [deg]')
legend('Elevator Trim', 'Aileron Trim', 'Rudder Trim', 'Elevator Full', 'Aileron Full', 'Rudder Full')

subplot(3,1,2)
grid on
hold on
plot(out.simout.Time, out.simout3.Data(:,[4 6]));
plot(out.simout.Time, out.simout4.Data(:,[4 6]));
xlabel('Time [s]')
ylabel('Rotor Thrust (body frame) [N]')
legend('F_x Trim', 'F_y Trim', 'F_z Trim', 'F_x Full', 'F_y Full', 'F_z Full')

subplot(3,1,3)
grid on
hold on
plot(out.simout.Time, out.simout3.Data(:,2));
plot(out.simout.Time, out.simout4.Data(:,2));
xlabel('Time [s]')
ylabel('Rotor Moment (body frame) [Nm]')
legend('t_x Trim', 't_y Trim', 't_z Trim', 't_x Full', 't_y Full', 't_z Full')