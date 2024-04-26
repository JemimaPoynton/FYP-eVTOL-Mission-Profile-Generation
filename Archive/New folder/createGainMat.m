function K = createGainMat(sysMat, QRfile, trimfile)
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

    k = lqr(sys, Q(:,:,QR_idx), R(:,:,QR_idx));
    K(:,:,i) = k;
end