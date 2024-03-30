function Lt = transMatrix(ang)

L1 = [1  0            0          ;
      0  cos(ang(1))  sin(ang(1));
      0 -sin(ang(1))  cos(ang(1))]; % transformation matrices

L2 = [cos(ang(2)) 0 -sin(ang(2));
      0           1  0          ;
      sin(ang(2)) 0  cos(ang(2))];

L3 = [ cos(ang(3)) sin(ang(3)) 0;
      -sin(ang(3)) cos(ang(3)) 0;
       0           0           1];

Lt = L1*L2*L3;