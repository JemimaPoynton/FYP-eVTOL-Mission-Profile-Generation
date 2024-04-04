function stabilityDeriv = getStabilityDeriv(lift, mesh, U, alpha, beta, N, ref, plotBool, N_obj)
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
airflow.beta = 0;

%% Apply Vortex Lattice Method
for i = 1:length(alpha)
    airflow.alpha = alpha(i);

    [~, coeffA(i), ~, ~] = VLMV3(lift, mesh, airflow, N, 0, [0 0 0], ref.Sref, ref.cref, ref.bref, N_obj);
end

for j = 1:length(beta)
    airflow.alpha = 0.1;
    airflow.beta = beta(j);

    [~, coeffB(j), ~, ~] = VLMV3(lift, mesh, airflow, N, 0, [0 0 0], ref.Sref, ref.cref, ref.bref, N_obj);
end

%% Plot
if plotBool == 1
    figure(); grid on; hold on
    plot(alpha, [coeffA.CL])
    plot(alpha, [coeffA.CD])
    plot(alpha, [coeffA.CY])

    figure(); grid on; hold on
    plot(alpha, [coeffA.Cl])
    plot(alpha, [coeffA.Cm])
    plot(alpha, [coeffA.Cn])

    figure(); grid on; hold on
    plot(beta, [coeffB.CL])
    plot(beta, [coeffB.CD])
    plot(beta, [coeffB.CY])

    figure(); grid on; hold on
    plot(beta, [coeffB.Cl])
    plot(beta, [coeffB.Cm])
    plot(beta, [coeffB.Cn])

end

%% Get Derivatives
CLa = meanDeriv(alpha, [coeffA.CL]);
CDa = meanDeriv(alpha, [coeffA.CD]);
CYa = meanDeriv(alpha, [coeffA.CY]);

Cla = meanDeriv(alpha, [coeffA.Cl]);
Cma = meanDeriv(alpha, [coeffA.Cm]);
Cna = meanDeriv(alpha, [coeffA.Cn]);

CLb = meanDeriv(beta, [coeffB.CL]);
CDb = meanDeriv(beta, [coeffB.CD]);
CYb = meanDeriv(beta, [coeffB.CY]);

Clb = meanDeriv(beta, [coeffB.Cl]);
Cmb = meanDeriv(beta, [coeffB.Cm]);
Cnb = meanDeriv(beta, [coeffB.Cn]);

%% Store Derivatives
stabilityDeriv.CLa = CLa;
stabilityDeriv.CDa = CDa;
stabilityDeriv.CYa = CYa;

stabilityDeriv.Cla = Cla;
stabilityDeriv.Cma = Cma;
stabilityDeriv.Cna = Cna;

stabilityDeriv.CLb = CLb;
stabilityDeriv.CDb = CDb;
stabilityDeriv.CYb = CYb;

stabilityDeriv.Clb = Clb;
stabilityDeriv.Cmb = Cmb;
stabilityDeriv.Cnb = Cnb;

