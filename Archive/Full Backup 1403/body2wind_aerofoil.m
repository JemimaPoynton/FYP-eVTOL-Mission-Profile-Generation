function Rbw = body2wind_aerofoil(a, b)
% function wind2body gets the transformation matrix from the body to wind
% frame. Aerofoil reference axis.
%
% a: angle of attack (alpha)
% b: sideslip angle (beta)

Rbw = [ cos(a)*cos(b);
       -cos(a)*sin(b);
        sin(a)         ];