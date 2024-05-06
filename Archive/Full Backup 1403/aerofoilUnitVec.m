function [un, uc] = aerofoilUnitVec(twist)
% function tangentialVector outputs the transformation matrix to
% get the normal unit vectors on the X-Z aerofoil coordinate
% system. 
% 
% twist: twist on aerofoil at point of interest
% un: normal vector +ve lift (apply a -1 factor for negative lift)
% uc: tangential vector

un = [sin(twist);
      0         ;
      cos(twist)];

uc = [ cos(twist);
       0         ;
      -sin(twist)];
end