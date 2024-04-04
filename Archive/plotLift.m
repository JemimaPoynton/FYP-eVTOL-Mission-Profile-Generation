function plotLift(lift, plotCS)
% function plotLift [!DESC!] 

    nL = length(lift);
    
    for i = 1:nL
        ang = lift(i).ang.*[1 -1 1];
        xyz1 = lift(i).pos;
    
        if i>1
            handlevis = 'off'; % Handling legend
        else
            handlevis = 'on';
        end
    
        Lt = transMatrix(ang);
        xyz2 = xyz1 + [lift(i).span*tan(lift(i).sweep) lift(i).sideY*lift(i).span 0]*Lt; % transform a translation of span in the wing plane
        xyz3 = xyz1 + [-getChord(lift(i), lift(i).span) + lift(i).span*tan(lift(i).sweep) lift(i).sideY*lift(i).span 0]*Lt;
        xyz4 = xyz1 + [-getChord(lift(i), 0) 0 0]*Lt;
    
        plot3([xyz1(1) xyz2(1) xyz3(1) xyz4(1)], [xyz1(2) xyz2(2) xyz3(2) xyz4(2)], [xyz1(3) xyz2(3) xyz3(3) xyz4(3)], 'black-', 'HandleVisibility', handlevis)
        
        if plotCS == 1 % plot control surfaces
            CSs_vec = lift(i).CSs;
            for j = 1:length(CSs_vec)
                ang = lift(i).ang.*[1 -1 1];
                xyz12 = CSs_vec(j).pos;
        
                if j>1
                    handlevis = 'off'; % Handling legend
                end
                
                CT = getChord(CSs_vec(j), CSs_vec(j).span, lift(i));
                CR = getChord(CSs_vec(j), 0, lift(i));
        
                Lt = transMatrix(ang);
        
                if CSs_vec(j).edge == "LE"
                    taperAng = atan((getChord(lift(i),0) - getChord(lift(i),lift(i).span))/lift(i).span);
        
                    xyz22 = xyz12 + [-CR + CT + CSs_vec(j).span*tan(taperAng) + (CSs_vec(j).span)*tan(lift(i).sweep), ...
                            lift(i).sideY*CSs_vec(j).span, ...
                            0]*Lt;
        
                    xyz32 = xyz12 + [-CR + CSs_vec(j).span*tan(taperAng) + (CSs_vec(j).span)*tan(lift(i).sweep), ...
                            lift(i).sideY*CSs_vec(j).span, ...
                            0]*Lt;
                    
                    xyz42 = xyz12 + [-CR 0 0]*Lt;
                else
                    xyz22 = xyz12 + ([CSs_vec(j).span*tan(lift(i).sweep) lift(i).sideY*CSs_vec(j).span 0]+ CSs_vec(j).c*[cos(CSs_vec(j).deflection) - 1 0 sin(CSs_vec(j).deflection)])*Lt; % transform a translation of span in the wing plane
                    xyz32 = xyz12 + ([-CT + CSs_vec(j).span*tan(lift(i).sweep) lift(i).sideY*CSs_vec(j).span 0])*Lt;
                    xyz42 = xyz12 + ([-CR 0 0] )*Lt;
                    
                    xyz12 = xyz12+ CSs_vec(j).c*[cos(CSs_vec(j).deflection) - 1 0 sin(CSs_vec(j).deflection)]*Lt; % adjusting for deflection
                end
                
                plot3([xyz12(1) xyz22(1) xyz32(1) xyz42(1) xyz12(1)], [xyz12(2) xyz22(2) xyz32(2) xyz42(2) xyz12(2)], [xyz12(3) xyz22(3) xyz32(3) xyz42(3) xyz12(3)], 'green-', 'HandleVisibility', handlevis)
            end
        end
    end