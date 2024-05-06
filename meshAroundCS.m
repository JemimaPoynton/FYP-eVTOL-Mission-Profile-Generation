function [XY, nC, cntrl_n_s] = meshAroundCS(wing, N, totalVar_CS, totalVar_wing, dir, xit, cntrl_n_s)


if ~exist('xit', 'var')
    div_s = totalVar_wing/(N-1);
    cntrl_n_s = floor(totalVar_CS/div_s);
    cntrl_div_s = totalVar_CS./cntrl_n_s;
else
    if xit == 0
        div_s = totalVar_wing/(N-1);
        cntrl_n_s = floor(totalVar_CS/div_s);
        cntrl_div_s = totalVar_CS./cntrl_n_s;
    end
end

Lt = transMatrix(wing.ang);
wingpos = wing.pos/Lt;
cspos = wing.CSs(1).pos/Lt;

if dir == 2
    weight_inb = abs(cspos(dir) - wingpos(dir))/(totalVar_wing - totalVar_CS); % weighting of distance inboard of control surface
else
    weight_inb = 0;
end
weight_out = 1 - weight_inb;

n_inb = ceil((N-1 - cntrl_n_s)*weight_inb); % inboard division based on weighting distribution of remanining panels
n_out = N-1 - cntrl_n_s - n_inb;

if n_out == 0 && weight_out ~= 0
    n_out = 1; 
    if n_inb > cntrl_n_s
        n_inb = n_inb - 1;
    else
        cntrl_n_s = cntrl_n_s - 1;
    end
end

if n_inb == 0
    sec1 = [];
else
    sec1 = linspace(0, abs(cspos(dir)), n_inb+1);
end

if dir == 2
    sec2 = linspace(abs(cspos(dir)), abs(cspos(dir)) + totalVar_CS, cntrl_n_s+1);
    sec3 = linspace(abs(cspos(dir)) + totalVar_CS, totalVar_wing, n_out+1);

    XY = [sec1, sec2(2:end), sec3(2:end)];
else
    sec2 = linspace(abs(totalVar_wing), totalVar_wing - totalVar_CS, cntrl_n_s+1);
    sec3 = linspace(totalVar_wing - totalVar_CS, 0, n_out+1);

    XY = flip([sec1, sec2, sec3(2:end)],2);
end

if wing.sideY == -1 && dir == 2 % swap for symmetrical wing
    nC = n_out+1:1:n_out+cntrl_n_s;
else
    nC = n_inb+1:1:n_inb+cntrl_n_s;
end
