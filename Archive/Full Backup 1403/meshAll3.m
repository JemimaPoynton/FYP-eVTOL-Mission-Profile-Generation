function [lift, mesh, N] = meshAll3(lift, n, meshBool, plotMesh)
% ![DESC]! Lift must contain inputs in symmetrical pairs

N = n+1;

if size(N,1) < length(lift)
    N = [N;
         ones(size(lift,2) - size(N,1),2).*N(size(N,1), 1:2)];
end

for a = 1:length(lift) % iterate through lift object
    if meshBool == 1
        lift(a) = createMesh3D(lift(a), N(a,:), plotMesh, a);
    end
end

%% Group in terms sideY for correct Y direction
liftL = []; liftR = [];

for a = 1:length(lift)
    if lift(a).sideY == 1
        liftL = [liftL lift(a)];
    else
        liftR = [liftR lift(a)];
    end
end

lift = [liftL' liftR'];

%% Combine Mesh - New Format
% leng = sum(N(1,1).^2 + N(2,1).^2);
mesh.x = []; mesh.y = []; mesh.z = []; mesh.control.xc = []; mesh.control.yc = []; mesh.control.zc = [];
mesh.nodes.xq = []; mesh.nodes.yq = []; mesh.nodes.zq = []; mesh.unitVec = [];
 count1 = 1; count2 = 0; fac = 0; xn = []; yn = []; zn = [];

%% Reformat Control Points
for i = 1:size(lift,1) % rows
     for j = 1:size(lift,2) % columns
         lidx = (i-1)*size(lift,1) + j; % linear 1D index

         for num1 = 1:n(lidx,2)
             % indices are weird :(
             if lift(i,j).sideY == -1
                if num1 == 1
                    count2 = count2 + n(lidx,1)*n(lidx,2);
                    fac = count2;
                else
                    count2 = count2 - n(lidx,1);
                end
             else
                count2 = count2 + n(lidx,1);
             end

             count1 = count2 - n(lidx,1)+1;

            mesh.control.xyz(count1:count2, 2) = lift(i,j).mesh.control.yc(:,num1);
            mesh.control.xyz(count1:count2, 1) = lift(i,j).mesh.control.xc(:,num1);
            mesh.control.xyz(count1:count2, 3) = lift(i,j).mesh.control.zc(:,num1);

            mesh.unitVec(count1:count2, 1) = lift(i,j).mesh.unitVec(:,num1,1);
            mesh.unitVec(count1:count2, 2) = lift(i,j).mesh.unitVec(:,num1,2);
            mesh.unitVec(count1:count2, 3) = lift(i,j).mesh.unitVec(:,num1,3);

             if num1 == n(lidx,2) && fac > 0 % reset counter when backwards
                 count2 = fac;
                 fac = 0; % reset
             end
         end
     end 
end

%% Reformat Node Points
for i = 1:size(lift,1) % rows
    for j = 1:size(lift,2) % columns
    lidx = (i-1)*size(lift,1) + j; % linear 1D index

        xq = lift(i,j).mesh.nodes.xq;
        yq = lift(i,j).mesh.nodes.yq;
        zq = lift(i,j).mesh.nodes.zq;

        for num1 = 1:size(lift(i,j).mesh.nodes.xq,2)-1

            if lift(i,j).sideY == -1
                num1 = size(lift(i,j).mesh.nodes.xq,2) - num1; % flip if on other side
            end

            boundx  = lift(i,j).mesh.nodes.xq(end,end)*ones(n(lidx,1),1);
            newx = [boundx xq(1:n(lidx,1), num1:num1+1) boundx];

            boundy = [yq(1:n(lidx), num1) yq(1:n(lidx,1), num1+1)];
            newy = [boundy(:,1) yq(1:n(lidx,1), num1:num1+1) boundy(:,end)];

            boundz = [zq(1:n(lidx), num1) zq(1:n(lidx,1), num1+1)];
            newz = [boundz(:,1) zq(1:n(lidx,1), num1:num1+1) boundz(:,end)];

            xn = [xn; newx];
            yn = [yn; newy];
            zn = [zn; newz];
        end 
    end 
end

mesh.nodes.xq = xn;
mesh.nodes.yq = yn;
mesh.nodes.zq = zn;

%% Reformat Lattice Points
xn = []; yn = []; zn = [];

for i = 1:size(lift,1) % rows
%         idx = size(yn,1);

    x = [lift(i,1).mesh.x flip(lift(i,2).mesh.x,2)];
    y = [lift(i,1).mesh.y flip(lift(i,2).mesh.y,2)];
    z = [lift(i,1).mesh.z flip(lift(i,2).mesh.z,2)];

    for num2 = 1:size(x,2)/2-1
        for num1 = 1:size(x,1)-1
            
            newx = [x(num1,num2) x(num1, num2+1) x(num1+1, num2+1) x(num1+1, num2) x(num1,num2)];
            newy = [y(num1,num2) y(num1, num2+1) y(num1+1, num2+1) y(num1+1, num2) y(num1,num2)];
            newz = [z(num1,num2) z(num1, num2+1) z(num1+1, num2+1) z(num1+1, num2) z(num1,num2)];

            xn = [xn; newx];
            yn = [yn; newy];
            zn = [zn; newz];
        end
    end

    for num2 = size(x,2)/2+1:size(x,2)-1
        for num1 = 1:size(x,1)-1
            
            newx = [x(num1,num2+1) x(num1,num2) x(num1+1,num2) x(num1+1,num2+1) x(num1,num2+1)];
            newy = [y(num1,num2+1) y(num1,num2) y(num1+1,num2) y(num1+1,num2+1) y(num1,num2+1)];
            newz = [z(num1,num2+1) z(num1,num2) z(num1+1,num2) z(num1+1,num2+1) z(num1,num2+1)];

            xn = [xn; newx];
            yn = [yn; newy];
            zn = [zn; newz];
        end
    end
    
%         if lift(i,j).sideY == -1
%             yn(idx+1:idx+n(lidx)^2, 1:n(1)) = flip(yn(idx+1:idx+n(lidx)^2, 1:n(1)),1);
%         end

end 

mesh.x = xn;
mesh.y = yn;
mesh.z = zn;

%% Set N
N = size(mesh.nodes.xq);




