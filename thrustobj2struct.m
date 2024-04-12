function thrustIn = thrustobj2struct(aircraft, IC)

thrustIn = struct();
thrustObj = aircraft.thrust; % readibility

for i = 1:length(thrustObj.rotors)
    thrustIn.xyz_tr(i,1:3) = thrustObj.rotors(i).pos;
    thrustIn.kt(i,1) = thrustObj.rotors(i).kt;
    thrustIn.kb(i,1) = thrustObj.rotors(i).kb;
    thrustIn.dir(i,1) = thrustObj.rotors(i).dir;
end

thrustIn.Tinit = IC;