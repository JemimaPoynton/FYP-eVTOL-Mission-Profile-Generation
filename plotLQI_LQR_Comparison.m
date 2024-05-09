idx = [10, 31, 40];
st = [3, 1, 1];
Kpif = createGainMat(sysMat_LQI, 'QRvalsLQI.mat', 'trimUAM1');
Kf = createGainMat(sysMat, 'QRValsLQR', 'trimUAM1');

figure(); 

for i = 1:length(idx)
    subplot(3,1,i); grid on; hold on;

    Ue = Ut(:,idx(i)+1);ca_inf
    Xe = X(:,idx(i)+1);
    Xes = X(:,idx(i)); 
    K = Kf(:,:,idx(i));
    Kpi = Kpif(:,:,idx(i));

    out1 = sim('Cruise_Model_V8_QRTuning_LQI.slx');
    out2 = sim('Cruise_Model_V8_QRTuning_FM.slx');

    plot(out1.simout.Time, out1.simout.Data(:,st(i)), '-')
    plot(out2.simout.Time, out2.simout.Data(:,st(i)),'black--')
    plot(out2.simout.Time, ones(size(out2.simout.Time))*Xe(st(i)), 'black:')

    legend('LQI', 'LQR', 'Reference')
end