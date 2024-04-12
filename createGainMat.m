function K = createGainMat(sysMat, QRfile, trimfile)
% function createGainMat creates longitudinal and lateral gain matrices for
% a reference scheduled controller

stages = dictionary('v',1,'t',2, 'T', 3,'c', 4); % dictionary to enable lettered modes, without if statements

for i = 1:length(sysMat)
    sys = sysMat(i);

    load(QRfile, 'Q', 'R')
    load(trimfile, 'trim')

    stg = ceil(i/trim.Np); % select stage

    if stg == 2 % split transition stage
        tstg = ceil((i-(stg-1)*trim.Np)/7);
        stg = stg + (tstg-1);
    end

    QR_idx = stages(trim.modes(stg));

    k = lqr(sys{1,1}, Q(:,:,QR_idx), R(:,:,QR_idx));
    K(:,:,i) = k;
end