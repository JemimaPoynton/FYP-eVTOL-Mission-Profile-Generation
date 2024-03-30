function thrustIn = thrustobj2struct(aircraft, IC)

thrustIn = struct();
thrustObj = aircraft.thrust; % readibility

for i = 1:length(thrustObj.rotors)
    thrustIn.xyz_tr(i,1:3) = thrustObj.rotors(i).pos;
end

thrustIn.Tinit = IC;