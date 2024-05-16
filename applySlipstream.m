function [CL, CD, Cm] = applySlipstream(aircraft, thrust, alpha, rpitch, Va, CLfunc, Cmfunc, CDfunc, w)
% function applySlipstream calculates key aerodynamic derivatives with the
% inclusion of a semi-empirical model of rotor-wing interference
S = aircraft.refGeo.Sref;
c = aircraft.refGeo.cref;

for i = 1:length(thrust.rotors)
    [Ss0, Ln, cref] = rotorSweptArea(aircraft, thrust.rotors(i)); % get swept area

    T = thrust.rotors(i).kt*w(i)^2;

    CT = thrust.rotors(i).kt/(1.225*pi*(thrust.rotors(i).radius^2)*thrust.rotors(i).radius);
    Rw = thrust.rotors(i).radius*(0.78 + 0.22*exp(-0.3 - 2*Ln*sqrt(CT) - 60*CT));

    if rpitch <= 2*pi/3
        Ss(i) = Ss0 + (2*Rw*cref - Ss0)*rpitch/60;
    else
        Ss(i) = 2*Rw*cref;
    end

    vss(i) = sqrt(2*T/(1.225*pi*Rw^2) + (Va*cos(alpha)*sin(rpitch))^2) - Va*cos(alpha)*sin(rpitch);
    Vxss(i) = vss(i)*sin(rpitch) + Va*cos(alpha);
    Vzss(i) = -vss(i)*cos(rpitch) + Va*sin(alpha);

    alpha_ss(i) = atan(Vzss(i)/Vxss(i));
    Va_ss = sqrt(Vzss(i)^2 + Vxss(i)^2);

    if abs(alpha_ss) > 14.5*(pi/180)
        dCL(i) = 0;
        dCD(i) = 0;
        dCm(i) = 0;
    else 
        dCL(i) = 0.5*(Va_ss^2)*CLfunc(alpha_ss(i))*Ss(i);
        dCD(i) = 0.0310*3;
        dCm(i) = 0.5*(Va_ss^2)*Cmfunc(alpha_ss(i))*Ss(i)*c;
    end
end
L = sum(dCL) + 0.5*(Va^2)*CLfunc(alpha)*(S - sum(Ss));
D = sum(dCD) + 0.5*(Va^2)*CDfunc(alpha)*(S - sum(Ss));
m = sum(dCm) + 0.5*(Va^2)*Cmfunc(alpha)*(S - sum(Ss))*c;

CL = L/(0.5*1.225*S*Va^2); CD = D/(0.5*1.225*S*Va^2); Cm = m/(0.5*1.225*S*c*Va^2); % re-non dimensionalise

end

