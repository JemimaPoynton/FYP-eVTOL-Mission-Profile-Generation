function sysMat = lineariseTrimFull(aircraft, referenceGeo, coeff, filename, dx, fr)
% function lineariseTrimFull creates a matrix of state-based systems
% corresponsing to the trim points defined in filename
% 
% fr: boolean indicating a free rotor (rotation in all DoF)

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
     
    % if exist('fr', 'var') % check free rotor case
    %     if fr ~= 1 % seperate lines to prevent exists error
    %         Btemp = B; % remove Fy control
    %         B = zeros(9, size(B,2) - 1); B(:,1:end-5) = Btemp(:,1:end-6); B(:,end-4:end) = Btemp(:,[end-5, end-3:end]);
    %     end
    % else
    %         Btemp = B; % remove Fy control
    %         B = zeros(9, size(B,2) - 1); B(:,1:end-5) = Btemp(:,1:end-6); B(:,end-4:end) = Btemp(:,[end-5, end-3:end]);
    % end

    sysMat(i) = {ss(A,B,C,D)};
end