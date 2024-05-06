function Rbw = body2wind(a, b)
% function wind2body gets the transformation matrix from the body to wind
% frame
%
% a: angle of attack (alpha)
% b: sideslip angle (beta)

Rbw = [ cos(a)*cos(b)   sin(b)   sin(a)*cos(b);
       -cos(a)*sin(b)   cos(b)  -sin(a)*sin(b);
       -sin(a)          0        cos(a)       ];