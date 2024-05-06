function [Ss0, Ln, cref] = rotorSweptArea(aircraft, rotor)
% function rotor swept area calculates the key geometric parameters for
% estimating the effect of rotor interference
% 
% currently assumes that the rotor sits out side of LE (not mounted on topp)

%% Identify closest wing
% Get leading edge points of wings at span point
for i = 1:length(aircraft.lift)

    % only count if the wing is within 30 degrees deflection of the rotor
    % plane - approximation is likely not valid
    if ~(aircraft.lift(i).ang(1) > pi/6 || aircraft.lift(i).ang(2) > pi/6)
        if aircraft.lift(i).sideY == sign(rotor.pos(2))
            offset = [tan(aircraft.lift(i).sweep)*rotor.pos(2) 0 0]; % offset due to sweep
            LE = offset + [aircraft.lift(i).pos(1) 0 0] - ...
                [getChord(aircraft.lift(i), rotor.pos(2)) 0 0]; % assume dihedral negligible for approximation of area

            TE = aircraft.lift(i).pos + offset;

            vecl = sqrt(sum(abs(LE - [rotor.pos(1) 0 0]).^2));
            vect = sqrt(sum(abs(TE - [rotor.pos(1) 0 0]).^2));

            if vecl < vect
                dist(i) = sqrt(sum(vecl.^2));
            else
                dist(i) = sqrt(sum(vect.^2));
            end
        else
            dist(i) = 10*aircraft.lift(i).c;
        end
    else
        dist(i) = 10*aircraft.lift(i).c; % big value go brr
    end
end

[~, idx] = min(dist);
Ln = dist(idx)/rotor.radius;

%% Get area of segment
if Ln <= rotor.radius
    theta_seg = 2*acos(Ln/rotor.radius);
    Ss0 = 0.5*(rotor.radius^2)*(theta_seg - sin(theta_seg));
else
    Ss0 = 0;
end

%% Get mean chord of wing at mounting point
cref = getChord(aircraft.lift(i), rotor.pos(2));