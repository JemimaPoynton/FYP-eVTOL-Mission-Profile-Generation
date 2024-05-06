function V = inducedV_1Vortex(r0, r1, r2)
% Function inducedV_1Vortex calculates the velocity induced by the vortex
% characterised by r0, r1, r2. Applies equation 7.37 in:
%
% REF: J. J. Bertin, R. M. Cummings. Aerodynamics for engineers, 5th
%      Edition. Pearson Education International (2009). ISBN:
%      978-0-12-235521-6.

    T1 = (1/(4*pi)); % Term 1 of V
    T2 = cross(r1, r2)./(norm(cross(r1, r2).^2,3)); % Term 2 of V
    T3 = r0*(r1/norm(r1) - r2/norm(r2))';

    V = T1*T2*T3;
end