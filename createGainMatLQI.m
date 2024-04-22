function K = createGainMatLQI(sysMat, QRfile, trimfile)
% function createGainMat creates longitudinal and lateral gain matrices for
% a reference scheduled controller

stages = dictionary('v',1,'t',2,'c',4); % dictionary to enable lettered modes, without if statements

for i = 1:length(sysMat)
    sys = sysMat{i};

    load(QRfile, 'Q', 'R')
    load(trimfile, 'trim')

    stg = ceil(i/trim.Np); % select stage
    QR_idx = stages(trim.modes(stg));

    if QR_idx == 2 % if in transition stage
        tstg = ceil((i-(stg-1)*trim.Np)/(trim.Np/2));

        if trim.modes(stg-1) == 'v' % if previous stage is VTOL
            QR_idx = QR_idx + tstg - 1;
        else
            QR_idx = QR_idx - tstg + 2;
        end
    end

    %% Augment System
    C = zeros(3,size(sys.C,2)); C(1:3,1:3) = eye(3); C(4:6,7:9) = eye(3);

    Aa = [sys.A zeros(9,6);
          -C    zeros(6,6)];

    Ba = [sys.B; zeros(6,size(sys.B,2))];
    Ca = [C, zeros(6,6)];

    Qa = eye(12,12); Qa(1:9,1:9) = Q(:,:,QR_idx); Qa(10:12, 10:12) = Q(1:3,1:3,QR_idx);
    Ra = eye(size(sys.B,1),size(sys.B,2));

    sys_a = ss(Aa, Ba, Ca, []);

    k = lqr(sys_a, Qa, Ra);
    K(:,:,i) = k;
end