P = 'C:\Users\Jem\Downloads\NACA 4 digit airfoils';
S = dir(fullfile(P,'*.dat')); 

for k = 1:numel(S)
    F = fullfile(P,S(k).name);
    S(k).data = table2array(readtable(F));
end