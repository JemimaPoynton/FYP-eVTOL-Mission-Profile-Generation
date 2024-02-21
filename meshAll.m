function lift = meshAll(lift, n, meshBool, plotMesh)
% ![DESC]!

N = n+1;

if length(N) < length(lift)
    N = [N;
         ones(size(lift,2) - size(N,1),2).*N(size(N,1), 1:2)];
end

for a = 1:length(lift) % iterate through lift object
    if meshBool == 1
        lift(a) = createMesh3D(lift(a), N(a,:), plotMesh, a);
    end
end