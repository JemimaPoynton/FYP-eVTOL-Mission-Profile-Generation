function V = inducedV_1Vortex_matrix(r0, r1mn, r2mn)
% Function inducedV_1Vortex calculates the velocity induced by the vortex
% characterised by r0, r1, r2. Applies equation 7.37 in:
%
% REF: J. J. Bertin, R. M. Cummings. Aerodynamics for engineers, 5th
%      Edition. Pearson Education International (2009). ISBN:
%      978-0-12-235521-6.

    T1 = (1/(4*pi)); % Term 1 of V
    T2 = cross(r1mn, r2mn)./(sum(cross(r1mn, r2mn).^2,3)); % Term 2 of V

    T3a = (r2mn./sqrt(sum(r2mn.^2,3)) - r1mn./sqrt(sum(r1mn.^2,3))); % nb: sqrt->sum effectively applies norm
    T3 = r0(:,:,1).*T3a(:,:,1) ...
         + r0(:,:,2).*T3a(:,:,2) ...
         + r0(:,:,3).*T3a(:,:,3);

    V = T1.*T2.*T3;

    rd = sqrt((sum(cross(r1mn, r2mn).^2,3)./(sum(r0.^2,3)))); % REF: Tomas Melin, Tornado (2007)

    if rd < 1e-7
        V = V.*0;
    end
end