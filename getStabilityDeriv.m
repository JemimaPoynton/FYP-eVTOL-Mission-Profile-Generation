function stabilityDeriv = getStabilityDeriv(liftin, airflow, alpha, beta, ref, plotBool, N_obj, def, pqr, xyzref, CG)
% function getStabilityDeriv applies VLM and resolves the results to obtain
% stability derivatives to represent aircraft dynamics.
% 
% airflow: structure containing angle of attack (alpha), sideslip angle
% CL: lift coefficient
% CD: drag coefficient
% distribution: struct containing local lift and drag coefficients
% N: number of panels in [x and y]
% def: deflection of control surfaces [d1_1 ... d1_k; d2_1 .. d1_k] where 
%      d1 is deflection of the control surface on the 1st lifting surface 
%      ('wing'). Where no CS is present, the prior surfaces should be set 
%      to zero, i.e. [0; d2]. 1 to k represents a range of values over
%      which to sweep

%% Preallocation
stabilityDeriv = struct();
airflow.beta = 0;

[liftout, mesh, N] = meshAll3(liftin, N_obj, 1,1);

%% Apply Vortex Lattice Method
for i = 1:length(alpha)
    airflow.alpha = alpha(i);

    [~, coeffA(i), ~, AC] = VLMV3(liftout, mesh, airflow, N, 0, xyzref, ref.Sref, ref.cref, ref.bref, N_obj, CG);
end

for j = 1:length(beta)
    airflow.alpha = alpha(1);
    airflow.beta = beta(j);

    [~, coeffB(j), ~, ~] = VLMV3(liftout, mesh, airflow, N, 0, xyzref, ref.Sref, ref.cref, ref.bref, N_obj, CG);
end

airflow.alpha = alpha(1);
airflow.beta = beta(1);

for n = 1:size(def,1)
    for j = 1:length(def(n,:))
        liftdef = liftin; 
        if ~isempty(liftdef(n*2-1).CSs)
            liftdef(n*2-1).CSs.deflection = -def(n,j); % switch sign to meet convention (different to geometry)
            if liftdef(n*2).CSs.roll == 0
                liftdef(n*2).CSs.deflection = -def(n,j); % assuming symmetry and 1 control surface per wing - !expand this in future!
            else
                liftdef(n*2).CSs.deflection = def(n,j);
            end
        end

        % re-evaluate mesh
        [liftout_def, mesh, N] = meshAll3(liftdef, N_obj, 1,0);

        [~, coeffd(j,n), ~, ~] = VLMV3(liftout_def, mesh, airflow, N, 0, xyzref, ref.Sref, ref.cref, ref.bref, N_obj, CG);
    end
end

for j = 1:length(pqr)
    airflow.p = pqr(j);

    [~, coeffp(j), ~, ~] = VLMV3(liftout, mesh, airflow, N, 0, xyzref, ref.Sref, ref.cref, ref.bref, N_obj, CG);
end

for j = 1:length(pqr)
    airflow.q = pqr(j);

    [~, coeffq(j), ~, ~] = VLMV3(liftout, mesh, airflow, N, 0, xyzref, ref.Sref, ref.cref, ref.bref, N_obj, CG);
end

for j = 1:length(pqr)
    airflow.r = pqr(j);

    [~, coeffr(j), ~, ~] = VLMV3(liftout, mesh, airflow, N, 0, xyzref, ref.Sref, ref.cref, ref.bref, N_obj, CG);
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

Clp = meanDeriv(pqr, [coeffp.Cl]);
Cnp = meanDeriv(pqr, [coeffp.Cn]);
CYp = meanDeriv(pqr, [coeffp.CY]);

CLq = meanDeriv(pqr, [coeffq.CL]);
Cmq = meanDeriv(pqr, [coeffq.Cm]);

Clr = meanDeriv(pqr, [coeffr.Cl]);
Cnr = meanDeriv(pqr, [coeffr.Cn]);
CYr = meanDeriv(pqr, [coeffr.CY]);

for m = 1:size(def,1)
    CLn(1,m) = meanDeriv(def(m,:), [coeffd(:,m).CL]);
    CYn(1,m) = meanDeriv(def(m,:), [coeffd(:,m).CY]);
    CDn(1,m) = meanDeriv(def(m,:), [coeffd(:,m).CD]);

    Cmn(1,m) = meanDeriv(def(m,:), [coeffd(:,m).Cm]);
    Cln(1,m) = meanDeriv(def(m,:), [coeffd(:,m).Cl]);
    Cnn(1,m) = meanDeriv(def(m,:), [coeffd(:,m).Cn]);
end

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

%% Control
stabilityDeriv.CLn = CLn;
stabilityDeriv.CDn = CDn;
stabilityDeriv.CYn = CYn;

stabilityDeriv.Cmn = Cmn;
stabilityDeriv.Cln = Cln;
stabilityDeriv.Cnn = Cnn;

%% Rotational Velocities
stabilityDeriv.Clp = Clp;
stabilityDeriv.Cnp = Cnp;
stabilityDeriv.CYp = CYp;

stabilityDeriv.CLq = CLq;
stabilityDeriv.Cmq = Cmq;

stabilityDeriv.Clr = Clr;
stabilityDeriv.Cnr = Cnr;
stabilityDeriv.CYr = CYr;

