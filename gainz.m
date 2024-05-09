idx = 121;
Kpi = createGainMat(sysMat_LQI, 'QRvalsLQI_new.mat', 'trimUAM1');

Xes = X(:,idx);
Xe = X(:,idx+1);
Ue = Ut(:,idx+1);
Kpi = Kpi(:,:,idx+1);