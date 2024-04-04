function sysMat = lineariseTrimFull(aircraft, referenceGeo, coeff, filename, dx)
% function lineariseTrimFull creates a matrix of state-based systems
% corresponsing to the trim points defined in filename

%% Load Data
load(filename, 'X', 'U')

X = reshape(X,size(X,1),[]); % reshape to linear indices (no stages)
U = reshape(U,size(U,1),[]);

sysMat = cell(size(X,2),1);

%% Iterate
for i = 1:size(X,2)
    xref = X(:,i);
    uref = U(:,i);
    
    % define dynamics as an anonymous function for easy modification/reuse
    func = @(X,u)aeroDyn_ind(coeff, u', 1.225, X, referenceGeo, aircraft.m, thrustobj2struct(aircraft, zeros(1,4)), aircraft.CG); 
    [A, B] = lineariseTrim(func, xref, uref, dx, aircraft, 3);

    C = eye(9,9); % all states output
    D = []; % no controls included in output

    sysMat(i) = {ss(A,B,C,D)};
end