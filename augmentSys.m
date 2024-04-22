function sysMat_LQI = augmentSys(sysMat)

for i = 1:length(sysMat)
    sys = sysMat{i};
    k = size(sys.B,1);

    C = zeros(3,size(sys.C,2)); C(1:3,1:3) = eye(3); C(4:6,7:9) = eye(3);
    
    Aa = [sys.A zeros(k,6);
          -C    zeros(6,6)];
    
    Ba = [sys.B; zeros(6, k)];
    Ca = zeros(15,15); Ca(1:9,1:9) = eye(9);
    Da = [];
    
    sysMat_LQI{i,:} = ss(Aa, Ba, Ca, Da);
end
