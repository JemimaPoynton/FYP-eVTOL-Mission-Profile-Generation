function Rbw = wind2body(a, b)
% function wind2body gets the transformation matrix from the wind to body
% frame
%
% a: angle of attack (alpha)
% b: sideslip angle (beta)

Rbw = [cos(a)*cos(b) -cos(a)*sin(b) -sin(a);
       sin(b)         cos(b)         0     ;
       cos(b)*sin(a) -sin(a)*sin(b)  cos(a)];