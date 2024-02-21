function stabilityDeriv = getStabilityDeriv(lift, U, alpha, beta, N, plotBool)
% function getStabilityDeriv applies VLM and resolves the results to obtain
% stability derivatives to represent aircraft dynamics.
% 
% airflow: structure containing angle of attack (alpha), sideslip angle
% CL: lift coefficient
% CD: drag coefficient
% distribution: struct containing local lift and drag coefficients
% N: number of panels in [x and y]

%% Preallocation
stabilityDeriv = struct();
airflow = struct();
airflow.U = U;

%% Apply Vortex Lattice Method
for i = 1:length(alpha)
    airflow.alpha = alpha(i);

    for j = 1:length(beta)
        airflow.beta = beta(j);

        [lift, CL(i,j), CD(i,j), CM(i,j), distribution(i,j)] = VLM(lift, airflow, N, plotBool);
    end
end

%% Resolve Components