function sysMat = lineariseTrimFull(aircraft, referenceGeo, coeff, filename, dx)
% function lineariseTrimFull creates a matrix of state-based systems
% corresponsing to the trim points defined in filename

%% Load Data
load(filename, 'trim')

X = reshape(trim.X,size(trim.X,1),[]); % reshape to linear indices (no stages)
U = reshape(trim.U,size(trim.U,1),[]);
Ut = [U(1:3,:); reshape(trim.Ut,size(trim.Ut,1),[])];

sysMat = cell(size(X,2),1);

%% Iterate

for i = 1:size(X,2)
    xref = X(:,i);
    uref = Ut(:,i);
    
    % define dynamics as an anonymous function for easy modification/reuse
    func = @(X,u)aeroDyn_MF(coeff, u', 1.225, X, referenceGeo, aircraft.m, thrustobj2struct(aircraft, zeros(1,4)), aircraft.CG, aircraft.I); 
    [A, B] = lineariseTrim(func, xref, uref, dx, aircraft, 3);

    C = eye(size(A,1),size(A,1)); % all states output
    D = []; % no controls included in output

    sysMat(i) = {ss(A,B,C,D)};
end