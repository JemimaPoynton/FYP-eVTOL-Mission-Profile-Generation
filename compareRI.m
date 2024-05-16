%% Plot Thrust
figure(); hold on; grid on;
load('trimUAM2.mat')
kt = aircraft.thrust.rotors(1).kt; stg = trim.stg; U = trim.U; Np = trim.Np; idxf = trim.idxf;
plot(dist(idxf(1:Np*stg)), ([squeeze(sum(U([19 23],:,1))) squeeze(sum(U([19 23],:,2))) squeeze(sum(U([19 23],:,3))) squeeze(sum(U([19 23],:,4))) squeeze(sum(U([19 23],:,5)))]).^2*kt, 'red-')


load('trimUAM2_RI.mat')
kt = aircraft.thrust.rotors(1).kt; stg = trim.stg; U = trim.U; Np = trim.Np; idxf = trim.idxf;
plot(dist(idxf(1:Np*stg)), ([squeeze(sum(U([19 23],:,1))) squeeze(sum(U([19 23],:,2))) squeeze(sum(U([19 23],:,3))) squeeze(sum(U([19 23],:,4))) squeeze(sum(U([19 23],:,5)))]).^2*kt, 'black-')

%% Plot Alpha
figure(); hold on; grid on;
load('trimUAM2.mat')
kt = aircraft.thrust.rotors(1).kt; stg = trim.stg; U = trim.U; Np = trim.Np; idxf = trim.idxf; alpha = trim.alpha;
plot(dist(idxf(1:Np*stg)), [squeeze(alpha(1,:,1)) squeeze(alpha(1,:,2)) squeeze(alpha(1,:,3)) squeeze(alpha(1,:,4)) squeeze(alpha(1,:,5))]*(180/pi))

load('trimUAM2_RI.mat')
kt = aircraft.thrust.rotors(1).kt; stg = trim.stg; U = trim.U; Np = trim.Np; idxf = trim.idxf; alpha = trim.alpha;
plot(dist(idxf(1:Np*stg)), [squeeze(alpha(1,:,1)) squeeze(alpha(1,:,2)) squeeze(alpha(1,:,3)) squeeze(alpha(1,:,4)) squeeze(alpha(1,:,5))]*(180/pi))

%% Plot Deflection

figure(); hold on; grid on;
load('trimUAM2.mat')
kt = aircraft.thrust.rotors(1).kt; stg = trim.stg; U = trim.U; Np = trim.Np; idxf = trim.idxf; alpha = trim.alpha;
plot(dist(idxf(1:Np*stg)), [squeeze(U(1:2,:,1)) squeeze(U(1:2,:,2)) squeeze(U(1:2,:,3)) squeeze(U(1:2,:,4)) squeeze(U(1:2,:,5))]*(180/pi))

load('trimUAM2_RI.mat')
kt = aircraft.thrust.rotors(1).kt; stg = trim.stg; U = trim.U; Np = trim.Np; idxf = trim.idxf; alpha = trim.alpha;
plot(dist(idxf(1:Np*stg)), [squeeze(U(1:2,:,1)) squeeze(U(1:2,:,2)) squeeze(U(1:2,:,3)) squeeze(U(1:2,:,4)) squeeze(U(1:2,:,5))]*(180/pi))