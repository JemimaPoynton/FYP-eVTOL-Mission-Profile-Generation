function [Klong, Klat] = createGainMatDecoup(sysMat, QRfile)
% function createGainMat creates longitudinal and lateral gain matrices for
% a reference scheduled controller at the systems linearised about the trim
% points contained in sysMat

for i = 1:length(sysMat)
    %% Decouple Lateral and Longitudinal Control
    sys = sysMat(i);

    A = sys{1,1}.A; % extract only relevent states to longitudinal
    B = sys{1,1}.B; % assuming all inputs have the possibility of effecting states

    %% Create Q and R
    
    
    %% Solve gain
    Klg = lqr(Alg, Blg, Qlg, Rlg);
    Klt = lqr(Alt, Blt, Qlt, Rlt);

    Klong(:,:,i) = Klg;
    Klat(:,:,i) = Klt;
end
